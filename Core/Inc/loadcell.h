#ifndef __LOADCELL_H
#define __LOADCELL_H

#include <stdbool.h>
#include <stdint.h>

void    loadcell_init(void);
void    loadcell_update(void);        // викликати з main loop
float   loadcell_get_kg(void);        // повертає зусилля в кг
float   loadcell_get_kN(void);        // повертає зусилля в кН (= kg / 101.97)
float   loadcell_get_fast_kg(void);   // швидший фільтр для AUTO режиму
float   loadcell_get_fast_kN(void);   // швидший фільтр в кН
bool    loadcell_is_ready(void);      // HX711 готовий до зчитування
bool    loadcell_has_error(void);     // timeout помилка (>500ms без даних)
void    loadcell_tare(void);          // скинути нуль
void    loadcell_set_scale(float s);  // встановити калібрувальний коефіцієнт
float   loadcell_get_scale(void);
int32_t loadcell_get_offset(void);
void    loadcell_set_offset(int32_t offset);
int32_t loadcell_read_raw(void);      // усереднене сире значення для калібровки
bool    loadcell_capture_raw(int32_t *out_raw, uint16_t samples, uint32_t timeout_ms);

#endif /* __LOADCELL_H */
