//#include "wave.h"
//
//// 假设你在 CubeMX 中启用了 TIM9 来作为新的超声波时钟�?
//extern TIM_HandleTypeDef htim9;
//
//#define TRIG_PORT GPIOF
//#define TRIG_PIN  GPIO_PIN_4
//#define ECHO_PORT GPIOF
//#define ECHO_PIN  GPIO_PIN_7
//
//void Wave_Init(void)
//{
//    // 启动新的 16 位定时器
//    HAL_TIM_Base_Start(&htim9);
//    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
//}
//
///**
// * @brief  高精度微秒延时（兼容 16 位定时器溢出�?
// */
//static void delay_us(uint32_t us)
//{
//    uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim9);
//    // 使用 uint16_t 算差值，利用组合语言的补码特性自动处�?65535 �?0 的翻�?
//    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim9) - start) < us);
//}
//
//float Wave_Get_Distance(void)
//{
//    uint16_t start_time;
//    uint16_t echo_time;
//
//    // 1. 触发超声�?
//    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
//    delay_us(15);
//    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
//
//    // 2. 等待高电平（加入超时保护，防止卡死）
//    start_time = (uint16_t)__HAL_TIM_GET_COUNTER(&htim9);
//    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
//    {
//        // 16位定时器下，差值不能超�?65535。这里设�?10000us (10ms) 超时
//        if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim9) - start_time) > 10000)
//        {
//            return -1.0f;
//        }
//    }
//
//    // 3. 计算高电平持续时�?
//    start_time = (uint16_t)__HAL_TIM_GET_COUNTER(&htim9);
//    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
//    {
//        // 设定 25000us (25ms) 安全测距极限超时
//        if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim9) - start_time) > 25000)
//        {
//            return -1.0f;
//        }
//    }
//
//    // 核心改动：强转为 uint16_t 进行减法
//    // 举例：如�?start_time �?65530，结束时计数器翻转到�?10
//    // (uint16_t)(10 - 65530) 在数学上等同�?(10 - 65530 + 65536) = 16，溢出问题被硬件完美消除�?
//    echo_time = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim9) - start_time);
//
//    return (float)echo_time * 0.017f;
//}

#include "wave.h"
#include "tx_api.h"  // 引入 ThreadX 内核 API

extern TIM_HandleTypeDef htim9;

#define TRIG_PORT GPIOF
#define TRIG_PIN  GPIO_PIN_4
#define ECHO_PORT GPIOF
#define ECHO_PIN  GPIO_PIN_7   // 核心改动：Echo 引脚�?MX_TIM9_Init 中的 PF7 保持一�?
#define ECHO_TIM_CHANNEL TIM_CHANNEL_1
/* 30 ms covers the HC-SR04 echo window without blocking the control task for long. */
#define WAVE_TIMEOUT_TICKS 30U

// 声明 ThreadX 信号量，用于同步中断与测距线�?
TX_SEMAPHORE wave_sem;

// 捕获状态机内部变量
static volatile uint8_t  capture_state = 0;
static volatile uint16_t capture_val1 = 0;
static volatile uint16_t capture_val2 = 0;
static volatile uint16_t high_time = 0;
static volatile uint8_t  wave_waiting = 0;

/**
 * @brief  超声波模块初始化
 * @note   需�?ThreadX 启动后的应用初始化（�?tx_application_define）中调用
 */
void Wave_Init(void)
{
    // 创建 ThreadX 信号量，初始计数值为 0
    tx_semaphore_create(&wave_sem, "wave_sem", 0);

    // 确保 Trig 引脚常态为低电�?
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // 启动定时器基础计数器（不带中断，仅让它自增计数�?
    HAL_TIM_Base_Start(&htim9);
}

/**
 * @brief  高精度微秒级阻塞延时（仅用于维持 15us �?Trig 触发脉冲�?
 */
static void delay_us(uint32_t us)
{
    uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim9);
    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim9) - start) < us);
}

/**
 * @brief  非阻塞式获取超声波距离（支持 RTOS 线程挂起�?
 * @return 成功返回物理距离(cm)，失败或超时返回 -1.0f
 */
float Wave_Get_Distance(void)
{
    float distance = -1.0f;

    /* Drain a stale completion left by a timed-out measurement. */
    while (tx_semaphore_get(&wave_sem, TX_NO_WAIT) == TX_SUCCESS) { }

    // 1. 强刷捕获状态机，确保从上升沿开�?
    capture_state = 0;
    high_time = 0;
    wave_waiting = 1;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim9, ECHO_TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);

    // 清除可能残存的中断标志位，防止误触发
    __HAL_TIM_CLEAR_IT(&htim9, TIM_IT_CC1);

    /* Arm input capture before the trigger pulse to avoid missing the echo edge. */
    HAL_TIM_IC_Start_IT(&htim9, ECHO_TIM_CHANNEL);

    // 2. 物理触发：发�?15us 高电平脉冲触发超声波模块
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // 3. 开启硬件定时器的输入捕获中�?

    // 4. RTOS 核心调度：挂起当前调用本函数的线程，让出 CPU
    // 允许最大等�?25 �?OS Ticks（假设系�?Tick �?1000Hz，即 25ms 测距超时保护�?
    if (tx_semaphore_get(&wave_sem, WAVE_TIMEOUT_TICKS) == TX_SUCCESS)
    {
        // 成功获取到信号量，说明下降沿捕获中断已成功计算出 high_time
        distance = (float)high_time * 0.017f;
    }
    else
    {
        // 25ms 超时未收到回波信号（前方可能过远或模块无响应�?
        // 强行关闭捕获中断，防止状态机错乱
        wave_waiting = 0;
        capture_state = 0;
        HAL_TIM_IC_Stop_IT(&htim9, ECHO_TIM_CHANNEL);
        distance = -1.0f;
    }

    return distance;
}

/**
 * @brief  TIM 输入捕获回调函数（由 HAL 库中断服务函数自动调用）
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM9)
    {
        /* Ignore a late callback from a measurement that already timed out. */
        if (wave_waiting == 0)
        {
            return;
        }
        if (capture_state == 0) // 状�?：成功捕获到 Echo 的上升沿
        {
            // 记录当前计数器的硬件时间�?
            capture_val1 = HAL_TIM_ReadCapturedValue(htim, ECHO_TIM_CHANNEL);

            // 立即将硬件配置切换为：捕获下降沿
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, ECHO_TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
            capture_state = 1;
        }
        else if (capture_state == 1) // 状�?：成功捕获到 Echo 的下降沿
        {
            // 记录当前计数器的硬件时间�?
            capture_val2 = HAL_TIM_ReadCapturedValue(htim, ECHO_TIM_CHANNEL);

            // 硬件级无感处�?16 位定时器溢出翻转（利用补码减法特性）
            high_time = (uint16_t)(capture_val2 - capture_val1);

            // 测距完成，立刻关闭捕获中断，释放硬件资源
            HAL_TIM_IC_Stop_IT(htim, ECHO_TIM_CHANNEL);

            // 核心原语：向 ThreadX 释放信号量，瞬间唤醒处于 tx_semaphore_get �?待的控制线程
            wave_waiting = 0;
            tx_semaphore_put(&wave_sem);

            capture_state = 0;
        }
    }
}
