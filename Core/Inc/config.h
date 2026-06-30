#ifndef __CONFIG_H
#define __CONFIG_H

/* ===== Механічні константи ===== */
// Двигун: 86HBP113AL4 (NEMA34, 8.5 Нм, 200 кроків/оберт)
// Редуктор: PX86 планетарний 1:10
// Гвинт: T20×4 (крок 4 мм/оберт)
// Драйвер: DM556, мікрокрок 1/8 (1600 кроків/оберт)
#define SCREW_PITCH_MM      4.0f    // крок гвинта T20x4
#define GEAR_RATIO          10.0f   // PX86 планетарний редуктор 1:10
#define MICROSTEP           8       // мікрокрок DM556 (SW5=ON SW6=OFF SW7=ON SW8=ON)
#define MOTOR_STEPS_REV     200     // кроків/оберт (1.8°)

// Кроків на 1 мм = (200 * 8 * 10) / 4 = 4000
#define STEPS_PER_MM        4000.0f

/* ===== Таймінг TIM7 / STEP =====
 * TIM7 працює з тікoм 100 us.
 * STEP формується коротким імпульсом тривалістю STEP_PULSE_TICKS тікiв.
 */
#define TIM7_TICK_US        16U
#define STEP_PULSE_TICKS    1U

/* ===== Швидкості (кроків/с) =====
 * При TIM7 = 100 us швидкість відповідає заданій набагато точніше.
 * Теоретичний максимум генератора STEP при TIM7=100 us і STEP_PULSE_TICKS=1:
 * min_period = 2 ticks => 5000 steps/s.
 *   SPEED_FAST:     5000 steps/s → 1.25 mm/s ≈ 75 mm/min
 *   SPEED_PRESS:     400 steps/s → 0.10 mm/s ≈ 6  mm/min
 *   SPEED_ENC:       200 steps/s → 0.05 mm/s ≈ 3  mm/min
 *   SPEED_APPROACH: 5000 steps/s → 1.25 mm/s ≈ 75 mm/min
 *   SPEED_APPROACH_SOFT: 1200 steps/s → 0.30 mm/s ≈ 18 mm/min
 */
#define SPEED_FAST          7800    // вільний рух / retract (практичний максимум без зриву)
#define SPEED_PRESS         400     // робочий рух після контакту
#define SPEED_ENC           200     // енкодер, тонке підналаштування
#define SPEED_APPROACH      7800    // швидкий підхід до деталі до появи навантаження
#define SPEED_APPROACH_SOFT 4000    // м'який підхід перед контактом
#define APPROACH_SOFT_KG    5.0f    // після цього порогу скидаємо швидкість до soft approach
#define APPROACH_CONTACT_KG 10.0f   // поріг контакту: після нього переходимо на SPEED_PRESS
#define POT_ADC_MIN_ACTIVE  64U     // запас від країв АЦП, щоб крайні положення були стабільні
#define POT_ADC_MAX_ACTIVE  4031U
#define POT_DOWN_MIN_SPEED  SPEED_ENC
#define POT_DOWN_MAX_SPEED  SPEED_FAST
#define ANGLE_ADC_VREF      3.3f    // опорна напруга АЦП (В)
#define ANGLE_SENSOR_V_MAX  5.0f    // напруга датчика при 360° (В)
#define ANGLE_DIVIDER_RATIO 0.5f    // дільник 10k/10k: 5V -> 2.5V на вході МК
#define ANGLE_DEFAULT_DEG   90.0f   // типовий цільовий кут
#define ANGLE_STEP_DEG      5.0f    // крок зміни у Settings
#define ANGLE_MAX_DEG       360.0f  // абсолютний датчик — макс. 1 оберт
#define HEAVY_START_SPEED   80      // окремий профіль старту для важкої механіки
#define HEAVY_RAMP_MS       20      // раз на N мс зменшуємо/збільшуємо period у heavy profile
#define HEAVY_RAMP_STEP     1       // крок зміни period у heavy profile
#define ACCEL_STEPS         2000    // кроків/с² (не використовується напряму, залишено для документації)

#define TRAVEL_START_SPEED   1500
#define TRAVEL_RAMP_MS       5
#define TRAVEL_RAMP_STEP     2
#define SPEED_PROFILE_SWITCH 1000

/* ===== Зусилля ===== */
// Датчик: LCF-6-V 2T (2000 кг = ~19.6 кН)
// Робочий діапазон: 7–16 кН (PDE TDI 300/400)
#define KN_TO_KG            101.97f              // 1 кН = 101.97 кг
#define FORCE_MAX_KG        2040.0f              // 20 кН — ліміт датчика
#define FORCE_DEFAULT_KN    8.75f               // T300 Nozzle midpoint
#define FORCE_DEFAULT_KG    (FORCE_DEFAULT_KN * KN_TO_KG)  // ≈892 кг
#define FORCE_STEP_KN       0.1f                // крок енкодера в кН
#define FORCE_STEP_KG       (FORCE_STEP_KN * KN_TO_KG)     // ≈10.2 кг
#define SLOWDOWN_THRESHOLD  0.80f               // 80% → резерв під додаткове уповільнення
#define OVERLOAD_FACTOR     1.10f               // 110% → аварійна зупинка

/* ===== AUTO режим по зусиллю ===== */
#define AUTO_FORCE_TOLERANCE_KG  3.0f    // вікно готовності навколо target
#define AUTO_CRUISE_ENTRY_KG     50.0f   // вище цієї похибки можна ще рухатись безперервно
#define AUTO_BURST_LARGE_ERR_KG  20.0f   // велика похибка → великий burst
#define AUTO_BURST_MED_ERR_KG    8.0f    // середня похибка → середній burst
#define AUTO_BURST_LARGE_STEPS   50U
#define AUTO_BURST_MED_STEPS     20U
#define AUTO_BURST_SMALL_STEPS   5U
#define AUTO_SETTLE_MS           400U    // пауза на стабілізацію після burst
#define AUTO_HOLD_MS             700U    // коротка пауза підтвердження в допуску
#define AUTO_DONE_MS             1500U   // показати DONE перед поверненням в IDLE
#define AUTO_FAST_FILTER_ALPHA   0.50f   // швидкий фільтр сили для AUTO алгоритму

/* ===== EEPROM (емуляція у Flash) ===== */
#define EEPROM_CALIB_FACTOR   0x00  // float, 4 байти — калібрувальний коефіцієнт HX711
#define EEPROM_CALIB_OFFSET   0x04  // int32, 4 байти — нульове зміщення (tare)
#define EEPROM_TARGET_FORCE   0x08  // float, 4 байти — останнє задане зусилля
#define EEPROM_MAGIC          0x0C  // uint8 = версія формату
#define EEPROM_ANGLE_TARGET   0x10  // float, 4 байти — цільовий кут
#define EEPROM_MAGIC_VALUE    0xAD

/* ===== HX711 налаштування ===== */
#define HX711_GAIN_128      1       // Channel A, gain 128 (за замовчуванням)
#define HX711_TIMEOUT_MS    500     // таймаут очікування готовності
#define LOADCELL_INVERT_SIGN 1U

/* ===== LCD2004 I2C адреса ===== */
#define LCD_I2C_ADDR        (0x27 << 1)  // спробувати 0x3F << 1 якщо не працює

/* ===== Антидребезг ===== */
#define DEBOUNCE_MS         20      // мс для кнопок/джойстика

/* ===== Тонка підстройка (ENC hold mode) ===== */
#define FINE_TIMEOUT_MS     300     // мс без тіків енкодера → зупин мотора

/* ===== Калібровка HX711 ===== */
#define CALIB_KNOWN_MIN_KG      100.0f
#define CALIB_KNOWN_MAX_KG      1000.0f
#define CALIB_KNOWN_DEFAULT_KG  100.0f
#define CALIB_KNOWN_STEP_KG     1.0f

#endif /* __CONFIG_H */
