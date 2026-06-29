#include "calibration.h"
#include "loadcell.h"
#include "display.h"
#include "config.h"
#include "torque_angle.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// ===== Flash EEPROM =====
// Використовуємо останній сектор (Sector 7, 128KB, 0x08060000)

#define FLASH_EEPROM_SECTOR  FLASH_SECTOR_7
#define FLASH_EEPROM_ADDR    0x08060000UL
#define FLASH_MAGIC_WITH_ANGLE 0xAD

#pragma pack(1)
typedef struct {
    float   scale;
    int32_t offset;
    float   target_force;
    uint8_t magic;
    float   angle_target;
} FlashData_t;
#pragma pack()

bool flash_load(float *scale, int32_t *offset, float *target, float *angle_target)
{
    const FlashData_t *p = (const FlashData_t *)FLASH_EEPROM_ADDR;
    if (p->magic != 0xAB && p->magic != 0xAC && p->magic != 0xAD) return false;
    if (!isfinite(p->scale) || p->scale <= 0.0f) return false;
    if (p->offset < (-8388608) || p->offset > 8388607) return false;
    if (!isfinite(p->target_force)) return false;
    if (scale)  *scale  = p->scale;
    if (offset) *offset = p->offset;
    if (target) *target = p->target_force;
    if (angle_target) {
        float a = (p->magic == FLASH_MAGIC_WITH_ANGLE) ? p->angle_target : ANGLE_DEFAULT_DEG;
        *angle_target = isfinite(a) ? a : ANGLE_DEFAULT_DEG;
    }
    return true;
}

bool flash_save(float scale, int32_t offset, float target, float angle_target)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = FLASH_EEPROM_SECTOR,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t sector_error = 0;
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    FlashData_t data = {
        .scale        = scale,
        .offset       = offset,
        .target_force = target,
        .magic        = FLASH_MAGIC_WITH_ANGLE,
        .angle_target = angle_target
    };

    // Записуємо побайтово (HAL_FLASH_Program підтримує BYTE)
    const uint8_t *src  = (const uint8_t *)&data;
    uint32_t       addr = FLASH_EEPROM_ADDR;
    for (size_t i = 0; i < sizeof(FlashData_t); i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + i, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}

// ===== Калібровка =====

static CalibStep s_step           = CALIB_STEP_IDLE;
static float     s_known_kg       = 1.0f;             // маса еталону (вводить оператор)
static int32_t   s_raw_tare       = 0;                // сирий відлік при нулі
static int32_t   s_raw_load       = 0;                // сирий відлік під навантаженням
static float     s_target_for_save = FORCE_DEFAULT_KG; // задане зусилля для збереження у Flash

static void update_display(void)
{
    char line[21];

    switch (s_step) {
        case CALIB_STEP_TARE:
            display_set_calib_text(0, "=== CALIBRATION ===");
            display_set_calib_text(1, "Remove all weight");
            display_set_calib_text(2, "then press ENC BTN");
            display_set_calib_text(3, "");
            break;

        case CALIB_STEP_LOAD:
            display_set_calib_text(0, "=== CALIBRATION ===");
            display_set_calib_text(1, "Place known weight");
            display_set_calib_text(2, "then press ENC BTN");
            display_set_calib_text(3, "");
            break;

        case CALIB_STEP_INPUT_MASS:
            snprintf(line, sizeof(line), "Mass: %.2f kg", (double)s_known_kg);
            display_set_calib_text(0, "=== CALIBRATION ===");
            display_set_calib_text(1, "Set weight mass:");
            display_set_calib_text(2, line);
            display_set_calib_text(3, "ENC=adjust  BTN=ok");
            break;

        case CALIB_STEP_CALCULATE:
        case CALIB_STEP_SAVE:
            display_set_calib_text(0, "=== CALIBRATION ===");
            display_set_calib_text(1, "Saving...");
            display_set_calib_text(2, "");
            display_set_calib_text(3, "");
            break;

        case CALIB_STEP_VERIFY: {
            float sc = loadcell_get_scale();
            snprintf(line, sizeof(line), "Scale: %.1f", (double)sc);
            display_set_calib_text(0, "Calibration done!");
            display_set_calib_text(1, line);
            snprintf(line, sizeof(line), "Force: %.2f kg", (double)loadcell_get_kg());
            display_set_calib_text(2, line);
            display_set_calib_text(3, "BTN=exit");
            break;
        }

        default:
            break;
    }
}

void calib_init(void)
{
    s_step     = CALIB_STEP_IDLE;
    s_known_kg = 1.0f;
}

void calib_start(float target_kg)
{
    s_step           = CALIB_STEP_TARE;
    s_known_kg       = 1.0f;
    s_target_for_save = target_kg;
    display_set_screen(SCREEN_CALIBRATION);
    update_display();
}

bool calib_is_active(void)
{
    return (s_step != CALIB_STEP_IDLE && s_step != CALIB_STEP_DONE);
}

CalibStep calib_get_step(void)
{
    return s_step;
}

void calib_confirm(void)
{
    switch (s_step) {
        case CALIB_STEP_TARE:
            loadcell_tare();
            s_raw_tare = loadcell_read_raw();
            s_step = CALIB_STEP_LOAD;
            update_display();
            break;

        case CALIB_STEP_LOAD:
            s_raw_load = loadcell_read_raw();
            s_step = CALIB_STEP_INPUT_MASS;
            update_display();
            break;

        case CALIB_STEP_INPUT_MASS:
            s_step = CALIB_STEP_CALCULATE;
            break;

        case CALIB_STEP_VERIFY:
            s_step = CALIB_STEP_DONE;
            display_set_screen(SCREEN_MAIN);
            break;

        default:
            break;
    }
}

void calib_adjust(int8_t delta)
{
    if (s_step == CALIB_STEP_INPUT_MASS) {
        s_known_kg += (float)delta * 0.1f;
        if (s_known_kg < 0.1f) s_known_kg = 0.1f;
        if (s_known_kg > 50.0f) s_known_kg = 50.0f;
        update_display();
    }
}

void calib_update(void)
{
    switch (s_step) {
        case CALIB_STEP_CALCULATE: {
            int32_t delta = s_raw_load - s_raw_tare;
            if (delta != 0 && s_known_kg > 0.0f) {
                float new_scale = (float)delta / s_known_kg;
                loadcell_set_scale(new_scale);
                loadcell_set_offset(s_raw_tare);
            }
            s_step = CALIB_STEP_SAVE;
            update_display();
            break;
        }

        case CALIB_STEP_SAVE: {
            if (flash_save(loadcell_get_scale(), loadcell_get_offset(),
                           s_target_for_save, torque_angle_get_target())) {
                s_step = CALIB_STEP_VERIFY;
                update_display();
            } else {
                s_step = CALIB_STEP_DONE;
                display_show_error("Flash save failed");
            }
            break;
        }

        default:
            break;
    }
}
