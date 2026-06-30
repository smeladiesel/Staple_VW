#include "motor.h"
#include "main.h"
#include "config.h"

// ===== Внутрішні змінні =====

static volatile MotorState  s_state        = MOTOR_IDLE;
static volatile int32_t     s_step_pos     = 0;      // поточна позиція (кроки)

static volatile uint16_t    s_step_period  = 0;      // тіків між кроками (0 = зупинено)
static volatile uint16_t    s_step_timer   = 0;      // лічильник до наступного переднього фронту
static volatile uint16_t    s_burst_steps  = 0;      // залишок кроків у burst mode (0 = без обмеження)
static volatile uint16_t    s_pulse_timer  = 0;      // тривалість STEP HIGH у тіках
static volatile bool        s_step_state   = false;

// Прискорення
static volatile uint16_t    s_period_target = 0;     // цільовий period (мінімальний)
static volatile uint16_t    s_period_current = 0;    // поточний period
static volatile uint16_t    s_accel_timer  = 0;      // мс до наступного кроку прискорення

typedef struct {
    uint16_t start_speed;   // стартова швидкість профілю в steps/s
    uint16_t ramp_ms;       // інтервал зміни period у ms
    uint16_t ramp_step;     // на скільки тіків змінюємо period за один крок ramp
} MotorRampProfile;

static const MotorRampProfile s_heavy_profile = {
    .start_speed = HEAVY_START_SPEED,
    .ramp_ms     = HEAVY_RAMP_MS,
    .ramp_step   = HEAVY_RAMP_STEP
};

static const MotorRampProfile s_travel_profile = {
    .start_speed = TRAVEL_START_SPEED,
    .ramp_ms     = TRAVEL_RAMP_MS,
    .ramp_step   = TRAVEL_RAMP_STEP
};

// ===== Приватні функції =====

// Зупинка при спрацюванні концевика — ISR-safe, встановлює IDLE (рух у протилежний бік дозволений)
static void motor_limit_stop_isr(void)
{
    s_step_period = 0;
    s_state       = MOTOR_IDLE;
    STEP_GPIO_Port->BSRR = (uint32_t)STEP_Pin << 16U;
    s_step_state  = false;
    s_pulse_timer = 0;
}

static uint16_t ms_to_ticks(uint16_t ms)
{
    uint32_t ticks = ((uint32_t)ms * 1000U + (TIM7_TICK_US - 1U)) / TIM7_TICK_US;
    return (ticks == 0U) ? 1U : (uint16_t)ticks;
}

// Переводить швидкість (кроків/с) у period (тіків між кроками).
static uint16_t speed_to_period(uint16_t speed_steps_per_sec)
{
    if (speed_steps_per_sec == 0) return 0;
    uint32_t ticks_per_sec = 1000000U / TIM7_TICK_US;
    uint32_t p = ticks_per_sec / speed_steps_per_sec;
    uint32_t min_period = STEP_PULSE_TICKS + 1U;
    if (p < min_period) p = min_period;
    if (p < 1U) p = 1U;
    return (uint16_t)p;
}

// Встановити напрямок
static void set_direction(bool up)
{
    if (up) {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
    }
}

static uint16_t motor_start_period(uint16_t target_period, const MotorRampProfile *profile)
{
    uint16_t start_period = speed_to_period(profile->start_speed);
    return (start_period < target_period) ? target_period : start_period;
}

static uint16_t motor_next_accel_timer(const MotorRampProfile *profile)
{
    return ms_to_ticks((profile->ramp_ms == 0U) ? 1U : profile->ramp_ms);
}

static void motor_apply_profile_step(const MotorRampProfile *profile)
{
    uint16_t step = (profile->ramp_step == 0U) ? 1U : profile->ramp_step;

    if (s_period_current > s_period_target) {
        uint16_t delta = s_period_current - s_period_target;
        s_period_current -= (delta > step) ? step : delta;
        s_step_period = s_period_current;
    } else if (s_period_current < s_period_target) {
        uint16_t delta = s_period_target - s_period_current;
        s_period_current += (delta > step) ? step : delta;
        s_step_period = s_period_current;
    }
}

static void motor_command(MotorState next_state, bool up, uint16_t speed, const MotorRampProfile *profile)
{
    if (s_state == MOTOR_ERROR) return;
    if (up ? motor_is_limit_top() : motor_is_limit_bot()) return;

    uint16_t target = speed_to_period(speed);
    if (target == 0) return;

    set_direction(up);

    __disable_irq();
    if (s_state == next_state) {
        s_period_target = target;
        s_burst_steps   = 0;

        // Якщо оператор зменшив швидкість, не скидаємо ramp, а м'яко переходимо на новий target.
        if (s_period_current == 0) {
            s_period_current = motor_start_period(target, profile);
            s_step_period    = s_period_current;
            s_step_timer     = (s_step_period > 0U) ? (s_step_period - 1U) : 0U;
        }
        if (s_accel_timer == 0) {
            s_accel_timer = motor_next_accel_timer(profile);
        }
    } else {
        s_period_target  = target;
        s_period_current = motor_start_period(target, profile);
        s_step_period    = s_period_current;
        s_step_timer     = (s_step_period > 0U) ? (s_step_period - 1U) : 0U;
        s_accel_timer    = motor_next_accel_timer(profile);
        s_burst_steps    = 0;
        s_state          = next_state;
        STEP_GPIO_Port->BSRR = (uint32_t)STEP_Pin << 16U;
        s_step_state     = false;
        s_pulse_timer    = 0;
    }
    __enable_irq();
}

static void motor_start_burst(MotorState next_state, bool up, uint16_t speed, uint16_t steps, const MotorRampProfile *profile)
{
    if (steps == 0U) return;
    if (s_state == MOTOR_ERROR) return;
    if (up ? motor_is_limit_top() : motor_is_limit_bot()) return;

    uint16_t target = speed_to_period(speed);
    if (target == 0) return;

    set_direction(up);

    __disable_irq();
    s_period_target  = target;
    s_period_current = motor_start_period(target, profile);
    s_step_period    = s_period_current;
    s_step_timer     = (s_step_period > 0U) ? (s_step_period - 1U) : 0U;
    s_accel_timer    = motor_next_accel_timer(profile);
    s_burst_steps    = steps;
    s_state          = next_state;
    STEP_GPIO_Port->BSRR = (uint32_t)STEP_Pin << 16U;
    s_step_state     = false;
    s_pulse_timer    = 0;
    __enable_irq();
}

// ===== Публічні функції =====

void motor_init(void)
{
    // Активувати драйвер (ENABLE = LOW)
    HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);

    s_state          = MOTOR_IDLE;
    s_step_pos       = 0;
    s_step_period    = 0;
    s_step_timer     = 0;
    s_step_state     = false;
    s_period_target  = 0;
    s_period_current = 0;
    s_pulse_timer    = 0;
    s_burst_steps    = 0;
}

void motor_clear_error(void)
{
    __disable_irq();
    if (s_state == MOTOR_ERROR) {
        s_state = MOTOR_IDLE;
    }
    __enable_irq();
}

void motor_move_up(uint16_t speed)
{
    const MotorRampProfile *profile = (speed >= SPEED_PROFILE_SWITCH) ? &s_travel_profile : &s_heavy_profile;
    motor_command(MOTOR_MOVING_UP, true, speed, profile);
}

void motor_move_down(uint16_t speed)
{
    const MotorRampProfile *profile = (speed >= SPEED_PROFILE_SWITCH) ? &s_travel_profile : &s_heavy_profile;
    motor_command(MOTOR_MOVING_DOWN, false, speed, profile);
}

void motor_burst_up(uint16_t speed, uint16_t steps)
{
    const MotorRampProfile *profile = (speed >= SPEED_PROFILE_SWITCH) ? &s_travel_profile : &s_heavy_profile;
    motor_start_burst(MOTOR_MOVING_UP, true, speed, steps, profile);
}

void motor_burst_down(uint16_t speed, uint16_t steps)
{
    const MotorRampProfile *profile = (speed >= SPEED_PROFILE_SWITCH) ? &s_travel_profile : &s_heavy_profile;
    motor_start_burst(MOTOR_MOVING_DOWN, false, speed, steps, profile);
}

void motor_nudge_up(uint16_t speed)
{
    if (s_state == MOTOR_ERROR) return;
    if (motor_is_limit_top()) return;

    set_direction(true);
    uint16_t period = speed_to_period(speed);

    __disable_irq();
    s_period_target  = period;
    s_period_current = period;
    s_step_period    = period;
    s_burst_steps    = 0;
    // Не скидаємо s_step_timer якщо вже рухаємось вгору — уникаємо стрибка
    if (s_state != MOTOR_MOVING_UP) s_step_timer = (period > 0U) ? (period - 1U) : 0U;
    s_accel_timer    = 0;
    s_state          = MOTOR_MOVING_UP;
    __enable_irq();
}

void motor_nudge_down(uint16_t speed)
{
    if (s_state == MOTOR_ERROR) return;
    if (motor_is_limit_bot()) return;

    set_direction(false);
    uint16_t period = speed_to_period(speed);

    __disable_irq();
    s_period_target  = period;
    s_period_current = period;
    s_step_period    = period;
    s_burst_steps    = 0;
    if (s_state != MOTOR_MOVING_DOWN) s_step_timer = (period > 0U) ? (period - 1U) : 0U;
    s_accel_timer    = 0;
    s_state          = MOTOR_MOVING_DOWN;
    __enable_irq();
}

void motor_stop(void)
{
    __disable_irq();
    s_step_period = 0;
    s_state       = MOTOR_IDLE;
    // Залишаємо STEP LOW
    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
    s_step_state = false;
    s_burst_steps = 0;
    s_pulse_timer = 0;
    __enable_irq();
}

void motor_emergency_stop(void)
{
    // Викликається з ISR — без disable_irq
    s_step_period = 0;
    s_state       = MOTOR_ERROR;
    STEP_GPIO_Port->BSRR = (uint32_t)STEP_Pin << 16U;  // STEP = LOW (атомарно)
    s_step_state = false;
    s_burst_steps = 0;
    s_pulse_timer = 0;
}

// Викликати з TIM7 IRQ (кожен TIM7_TICK_US)
void motor_tim_tick(void)
{
    if (s_step_period == 0) return;

    if (s_pulse_timer > 0U) {
        s_pulse_timer--;
        if (s_pulse_timer == 0U) {
            STEP_GPIO_Port->BSRR = (uint32_t)STEP_Pin << 16U;
            s_step_state = false;
        }
    }

    // Перевірка концевиків у ISR — зупиняє з IDLE (рух у протилежний бік дозволений)
    if (s_state == MOTOR_MOVING_UP && motor_is_limit_top()) {
        motor_limit_stop_isr();
        return;
    }
    if (s_state == MOTOR_MOVING_DOWN && motor_is_limit_bot()) {
        motor_limit_stop_isr();
        return;
    }

    // Плавно підтягуємо current period до target без перезапуску руху.
    if (s_accel_timer > 0) {
        s_accel_timer--;
    } else {
        s_accel_timer = motor_next_accel_timer(&s_heavy_profile);
        motor_apply_profile_step(&s_heavy_profile);
    }

    if (s_step_timer > 0U) {
        s_step_timer--;
    } else if (!s_step_state) {
        s_step_timer = (s_step_period > 0U) ? (s_step_period - 1U) : 0U;
        STEP_GPIO_Port->BSRR = STEP_Pin;
        s_step_state = true;
        s_pulse_timer = STEP_PULSE_TICKS;

        if (s_state == MOTOR_MOVING_UP) {
            s_step_pos++;
        } else if (s_state == MOTOR_MOVING_DOWN) {
            s_step_pos--;
        }

        if (s_burst_steps > 0U) {
            s_burst_steps--;
            if (s_burst_steps == 0U) {
                s_step_period = 0;
                s_state = MOTOR_IDLE;
            }
        }
    }
}

// Викликати з main loop: перевіряє переходи стану
void motor_update(void)
{
    // Якщо в стані ERROR — нічого не робимо, чекаємо команди зовні
    if (s_state == MOTOR_ERROR) return;

    // Перевірка концевиків для зупинки (додатковий захист у main loop)
    if (s_state == MOTOR_MOVING_UP && motor_is_limit_top()) {
        motor_stop();
    }
    if (s_state == MOTOR_MOVING_DOWN && motor_is_limit_bot()) {
        motor_stop();
    }
}

bool motor_is_limit_top(void)
{
    // NC + pull-up: normal = LOW (closed→GND), triggered = HIGH (open→pull-up)
    return HAL_GPIO_ReadPin(LIMIT_TOP_GPIO_Port, LIMIT_TOP_Pin) == GPIO_PIN_SET;
}

bool motor_is_limit_bot(void)
{
    return HAL_GPIO_ReadPin(LIMIT_BOT_GPIO_Port, LIMIT_BOT_Pin) == GPIO_PIN_SET;
}

bool motor_is_running(void)
{
    return (s_state == MOTOR_MOVING_UP || s_state == MOTOR_MOVING_DOWN);
}

bool motor_is_burst_active(void)
{
    return s_burst_steps > 0U;
}

float motor_get_position_mm(void)
{
    int32_t pos;
    __disable_irq();
    pos = s_step_pos;
    __enable_irq();
    return (float)pos / STEPS_PER_MM;
}

void motor_reset_position(void)
{
    __disable_irq();
    s_step_pos = 0;
    __enable_irq();
}

MotorState motor_get_state(void)
{
    return s_state;
}
