#include "input.h"
#include "main.h"
#include "config.h"

// ===== Антидребезг =====

typedef struct {
    uint8_t  counter;    // лічильник мс утримання
    bool     state;      // поточний підтверджений стан (true = натиснуто)
} DebounceBtn_t;

static DebounceBtn_t s_joy_up;
static DebounceBtn_t s_joy_down;
static DebounceBtn_t s_enc_sw;
static uint8_t       s_debounce_subtick = 0;

// Енкодер
static volatile int8_t s_enc_delta  = 0;   // накопичені кроки
static volatile bool   s_stop_flag  = false;
static GPIO_PinState   s_stop_normal_level = GPIO_PIN_SET;
static volatile GPIO_PinState s_stop_last_level = GPIO_PIN_SET;
static volatile bool   s_enc_sw_pressed_flag  = false;
static volatile bool   s_enc_sw_released_flag = false;


// ===== Приватні функції =====

static void debounce_tick(DebounceBtn_t *btn, bool raw_pressed)
{
    if (raw_pressed) {
        if (btn->counter < DEBOUNCE_MS) {
            btn->counter++;
        }
    } else {
        btn->counter = 0;
    }
    btn->state = (btn->counter >= DEBOUNCE_MS);
}

// ===== Публічні функції =====

void input_init(void)
{
    s_joy_up  = (DebounceBtn_t){0};
    s_joy_down = (DebounceBtn_t){0};
    s_enc_sw  = (DebounceBtn_t){0};
    s_enc_delta = 0;
    s_stop_flag = false;
    s_stop_normal_level = HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin);
    s_stop_last_level = s_stop_normal_level;
    s_enc_sw_pressed_flag = false;
    s_enc_sw_released_flag = false;
    s_debounce_subtick = 0;
}

void input_debounce_tick(void)
{
    // Encoder CLK polling at TIM7 rate (100µs) with 50-tick (5ms) debounce.
    // Static locals are ISR-only: no concurrent access with main loop writers.
    static bool     clk_prev = true;   // pull-up idle = HIGH
    static uint16_t deb_ctr  = 0;      // countdown; 0 = ready to accept next edge

    bool clk_now = (HAL_GPIO_ReadPin(ENC_CLK_GPIO_Port, ENC_CLK_Pin) == GPIO_PIN_SET);
    if (deb_ctr > 0U) deb_ctr--;
    if (!clk_now && clk_prev && (deb_ctr == 0U)) {
        bool dt = (HAL_GPIO_ReadPin(ENC_DT_GPIO_Port, ENC_DT_Pin) == GPIO_PIN_SET);
        // KY-040: CLK falls first (DT still HIGH) → CW; DT already LOW → CCW
        if (dt) s_enc_delta++;
        else    s_enc_delta--;
        deb_ctr = (uint16_t)(5000U / TIM7_TICK_US);  // 5ms debounce window
    }
    clk_prev = clk_now;

    s_debounce_subtick++;
    if (s_debounce_subtick < (1000U / TIM7_TICK_US)) {
        return;
    }
    s_debounce_subtick = 0;

    bool enc_prev = s_enc_sw.state;

    // LOW = натиснуто (pull-up + active-low)
    debounce_tick(&s_joy_up,   HAL_GPIO_ReadPin(JOY_UP_GPIO_Port,   JOY_UP_Pin)   == GPIO_PIN_RESET);
    debounce_tick(&s_joy_down, HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port, JOY_DOWN_Pin) == GPIO_PIN_RESET);
    debounce_tick(&s_enc_sw,   HAL_GPIO_ReadPin(ENC_SW_GPIO_Port,   ENC_SW_Pin)   == GPIO_PIN_RESET);

    if (!enc_prev && s_enc_sw.state) {
        s_enc_sw_pressed_flag = true;
    } else if (enc_prev && !s_enc_sw.state) {
        s_enc_sw_released_flag = true;
    }
}

// Викликається з EXTI3 ISR на falling edge ENC_CLK
void input_enc_isr(void)
{
    // KY-040: якщо DT=HIGH при CLK=FALLING → за годинниковою стрілкою (+1)
    //         якщо DT=LOW  при CLK=FALLING → проти годинникової стрілки (-1)
    if (HAL_GPIO_ReadPin(ENC_DT_GPIO_Port, ENC_DT_Pin) == GPIO_PIN_SET) {
        s_enc_delta++;
    } else {
        s_enc_delta--;
    }
}

void input_update(void)
{
    (void)0;
}

bool input_joy_up(void)
{
    return s_joy_up.state;
}

bool input_joy_down(void)
{
    return s_joy_down.state;
}

bool input_enc_sw_pressed(void)
{
    bool pressed;
    __disable_irq();
    pressed = s_enc_sw_pressed_flag;
    s_enc_sw_pressed_flag = false;
    __enable_irq();
    return pressed;
}

bool input_enc_sw_held(void)
{
    return s_enc_sw.state;
}

bool input_enc_sw_released(void)
{
    bool released;
    __disable_irq();
    released = s_enc_sw_released_flag;
    s_enc_sw_released_flag = false;
    __enable_irq();
    return released;
}

int8_t input_enc_get_delta(void)
{
    int8_t delta;
    __disable_irq();
    delta = s_enc_delta;
    s_enc_delta = 0;
    __enable_irq();
    return delta;
}

bool input_stop_pressed(void)
{
    return s_stop_flag;
}

void input_stop_clear(void)
{
    __disable_irq();
    s_stop_flag = false;
    __enable_irq();
}

bool input_stop_set(void)
{
    GPIO_PinState level = HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port, STOP_BTN_Pin);
    s_stop_last_level = level;
    s_stop_flag = true;
    return (level != s_stop_normal_level);
}

bool input_stop_is_active(void)
{
    GPIO_PinState level;
    __disable_irq();
    level = s_stop_last_level;
    __enable_irq();
    return (level != s_stop_normal_level);
}
