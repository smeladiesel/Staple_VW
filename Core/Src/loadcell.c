#include "loadcell.h"
#include "main.h"
#include "config.h"

#define AVG_SAMPLES  8

// ===== Стан модуля =====
static float   s_scale        = 1.0f;
static int32_t s_offset       = 0;
static float   s_force_kg     = 0.0f;
static float   s_force_fast_kg = 0.0f;
static bool    s_fast_valid   = false;

static int32_t s_avg_buf[AVG_SAMPLES];
static uint8_t s_avg_idx      = 0;
static int32_t s_avg_sum      = 0;
static uint8_t s_avg_count    = 0;

static uint32_t s_last_tick   = 0;
static bool     s_has_error   = false;

// ===== Приватні inline функції GPIO =====

static inline void hx_clk_hi(void)
{
    HX_CLK_GPIO_Port->BSRR = HX_CLK_Pin;
}

static inline void hx_clk_lo(void)
{
    HX_CLK_GPIO_Port->BSRR = (uint32_t)HX_CLK_Pin << 16U;
}

static inline bool hx_dat_lo(void)
{
    return (HX_DAT_GPIO_Port->IDR & HX_DAT_Pin) == 0;
}

// ~60ns при 168MHz — достатньо для t3 = 100ns з урахуванням часу запису GPIO
static inline void hx_delay(void)
{
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
}

// ===== Зчитування 24 біт з HX711 =====

static int32_t hx711_read(void)
{
    int32_t data = 0;

    for (int i = 0; i < 24; i++) {
        hx_clk_hi();
        hx_delay();
        data = (data << 1);
        if ((HX_DAT_GPIO_Port->IDR & HX_DAT_Pin) != 0) {
            data |= 1;
        }
        hx_clk_lo();
        hx_delay();
    }

    // 25-й імпульс → Channel A, Gain 128 для наступного зчитування
    hx_clk_hi();
    hx_delay();
    hx_clk_lo();
    hx_delay();

    // Знакове розширення 24→32 біт
    if (data & 0x800000) {
        data |= (int32_t)0xFF000000;
    }

    return data;
}

bool loadcell_capture_raw(int32_t *out_raw, uint16_t samples, uint32_t timeout_ms)
{
    if (!out_raw || samples == 0U) return false;

    int64_t sum = 0;
    uint16_t got = 0;
    uint32_t deadline = HAL_GetTick() + timeout_ms;

    while (got < samples) {
        if (hx_dat_lo()) {
            sum += hx711_read();
            got++;
            continue;
        }
        if ((int32_t)(HAL_GetTick() - deadline) >= 0) {
            return false;
        }
    }

    *out_raw = (int32_t)(sum / (int64_t)samples);
    return true;
}

// ===== Публічні функції =====

void loadcell_init(void)
{
    hx_clk_lo();
    s_scale      = 1.0f;
    s_offset     = 0;
    s_avg_idx    = 0;
    s_avg_sum    = 0;
    s_avg_count  = 0;
    s_has_error  = false;
    s_last_tick  = HAL_GetTick();
    s_force_fast_kg = 0.0f;
    s_fast_valid = false;
}

bool loadcell_is_ready(void)
{
    return hx_dat_lo();
}

void loadcell_update(void)
{
    if (!hx_dat_lo()) {
        if ((HAL_GetTick() - s_last_tick) > HX711_TIMEOUT_MS) {
            s_has_error = true;
        }
        return;
    }

    s_has_error = false;
    s_last_tick = HAL_GetTick();

    int32_t raw = hx711_read();

    // Ковзне середнє
    s_avg_sum -= s_avg_buf[s_avg_idx];
    s_avg_buf[s_avg_idx] = raw;
    s_avg_sum += raw;
    s_avg_idx = (s_avg_idx + 1) % AVG_SAMPLES;
    if (s_avg_count < AVG_SAMPLES) s_avg_count++;

    int32_t avg = s_avg_sum / (int32_t)s_avg_count;

    if (s_scale != 0.0f) {
        float force_kg = (float)(avg - s_offset) / s_scale;
        float raw_force_kg = (float)(raw - s_offset) / s_scale;
        if (LOADCELL_INVERT_SIGN) {
            force_kg = -force_kg;
            raw_force_kg = -raw_force_kg;
        }

        float abs_force = (force_kg < 0.0f) ? -force_kg : force_kg;
        float abs_raw_force = (raw_force_kg < 0.0f) ? -raw_force_kg : raw_force_kg;
        float glitch_limit = FORCE_MAX_KG * 2.0f;
        if (abs_force > glitch_limit || abs_raw_force > glitch_limit) {
            return;
        }

        s_force_kg = force_kg;
        if (!s_fast_valid) {
            s_force_fast_kg = raw_force_kg;
            s_fast_valid = true;
        } else {
            s_force_fast_kg += (raw_force_kg - s_force_fast_kg) * AUTO_FAST_FILTER_ALPHA;
        }
    } else {
        s_force_kg = 0.0f;
        s_force_fast_kg = 0.0f;
        s_fast_valid = false;
    }
}

float loadcell_get_kg(void)
{
    return s_force_kg;
}

float loadcell_get_kN(void)
{
    return s_force_kg / 101.97f;
}

float loadcell_get_fast_kg(void)
{
    return s_force_fast_kg;
}

float loadcell_get_fast_kN(void)
{
    return s_force_fast_kg / 101.97f;
}

bool loadcell_has_error(void)
{
    return s_has_error;
}

void loadcell_tare(void)
{
    if (s_avg_count > 0) {
        s_offset = s_avg_sum / (int32_t)s_avg_count;
    }
}

void loadcell_set_scale(float scale)
{
    s_scale = scale;
}

float loadcell_get_scale(void)
{
    return s_scale;
}

int32_t loadcell_get_offset(void)
{
    return s_offset;
}

void loadcell_set_offset(int32_t offset)
{
    s_offset = offset;
}

int32_t loadcell_read_raw(void)
{
    if (s_avg_count > 0) {
        return s_avg_sum / (int32_t)s_avg_count;
    }
    return 0;
}
