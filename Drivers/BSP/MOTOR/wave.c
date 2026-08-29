#include "wave.h"
#include "tx_api.h"

extern TIM_HandleTypeDef htim9;

#define TRIG_PORT GPIOF
#define TRIG_PIN  GPIO_PIN_4
#define ECHO_PORT GPIOF
#define ECHO_PIN  GPIO_PIN_7
#define ECHO_TIM_CHANNEL TIM_CHANNEL_1
/* 30 ms 可覆盖 HC-SR04 回波窗口，并限制控制线程的最长等待时间。 */
#define WAVE_TIMEOUT_TICKS 30U

TX_SEMAPHORE wave_sem;
static volatile uint8_t  capture_state = 0;
static volatile uint16_t capture_val1 = 0;
static volatile uint16_t capture_val2 = 0;
static volatile uint16_t high_time = 0;
static volatile uint8_t  wave_waiting = 0;
static uint8_t wave_initialized;

HAL_StatusTypeDef Wave_Init(void)
{
    if (tx_semaphore_create(&wave_sem, "wave_sem", 0) != TX_SUCCESS)
    {
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    if (HAL_TIM_Base_Start(&htim9) != HAL_OK)
    {
        (void)tx_semaphore_delete(&wave_sem);
        return HAL_ERROR;
    }
    wave_initialized = 1U;
    return HAL_OK;
}

/* 仅用于产生 HC-SR04 触发脉冲。 */
static void delay_us(uint32_t us)
{
    uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim9);

    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim9) - start) < us);
}

float Wave_Get_Distance(void)
{
    float distance = -1.0f;
    uint32_t interrupt_state;
    HAL_StatusTypeDef start_status;

    if (wave_initialized == 0U)
    {
        return -1.0f;
    }

    /* 清除上一次超时测量可能残留的完成信号。 */
    while (tx_semaphore_get(&wave_sem, TX_NO_WAIT) == TX_SUCCESS) { }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    capture_state = 0;
    high_time = 0;
    wave_waiting = 1;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim9, ECHO_TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_CLEAR_IT(&htim9, TIM_IT_CC1);

    /* 先启动捕获再触发，避免漏掉回波上升沿。 */
    start_status = HAL_TIM_IC_Start_IT(&htim9, ECHO_TIM_CHANNEL);
    if (start_status != HAL_OK)
    {
        wave_waiting = 0U;
    }
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
    if (start_status != HAL_OK)
    {
        return -1.0f;
    }

    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    if (tx_semaphore_get(&wave_sem, WAVE_TIMEOUT_TICKS) == TX_SUCCESS)
    {
        distance = (float)high_time * 0.017f;
    }
    else
    {
        interrupt_state = __get_PRIMASK();
        __disable_irq();
        if (wave_waiting != 0U)
        {
            wave_waiting = 0U;
            capture_state = 0U;
            (void)HAL_TIM_IC_Stop_IT(&htim9, ECHO_TIM_CHANNEL);
        }
        else
        {
            /* 回波下降沿恰好在超时边界到达。 */
            distance = (float)high_time * 0.017f;
        }
        if (interrupt_state == 0U)
        {
            __enable_irq();
        }
    }

    return distance;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM9)
    {
        /* 忽略已超时测量产生的迟到回调。 */
        if (wave_waiting == 0)
        {
            return;
        }
        if (capture_state == 0)
        {
            capture_val1 = HAL_TIM_ReadCapturedValue(htim, ECHO_TIM_CHANNEL);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, ECHO_TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
            capture_state = 1;
        }
        else if (capture_state == 1)
        {
            capture_val2 = HAL_TIM_ReadCapturedValue(htim, ECHO_TIM_CHANNEL);
            /* 无符号减法可正确处理一次 16 位计数器回绕。 */
            high_time = (uint16_t)(capture_val2 - capture_val1);
            HAL_TIM_IC_Stop_IT(htim, ECHO_TIM_CHANNEL);
            wave_waiting = 0;
            tx_semaphore_put(&wave_sem);
            capture_state = 0;
        }
    }
}
