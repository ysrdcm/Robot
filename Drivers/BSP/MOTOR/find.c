#include "find.h"
#include "car.h"

#define O4_PORT GPIOC
#define O4_PIN  GPIO_PIN_10
#define O3_PORT GPIOD
#define O3_PIN  GPIO_PIN_12
#define O2_PORT GPIOD
#define O2_PIN  GPIO_PIN_4
#define O1_PORT GPIOD
#define O1_PIN  GPIO_PIN_14

/* 码型连续出现三次才确认，用于过滤单次采样毛刺。 */
#define TRACK_PATTERN_CONFIRM_SAMPLES 3U
#define TRACK_PATTERN_ALL_WHITE       0x00U
#define TRACK_PATTERN_ALL_BLACK       0x0FU
#define TRACK_LINE_LOST_HOLD_MS       2000U

typedef void (*Track_Action_t)(void);

static uint8_t Track_Read_Pattern(void)
{
    uint8_t o4 = (HAL_GPIO_ReadPin(O4_PORT, O4_PIN) == GPIO_PIN_SET) ?
                 1U : 0U;
    uint8_t o3 = (HAL_GPIO_ReadPin(O3_PORT, O3_PIN) == GPIO_PIN_SET) ?
                 1U : 0U;
    uint8_t o2 = (HAL_GPIO_ReadPin(O2_PORT, O2_PIN) == GPIO_PIN_SET) ?
                 1U : 0U;
    uint8_t o1 = (HAL_GPIO_ReadPin(O1_PORT, O1_PIN) == GPIO_PIN_SET) ?
                 1U : 0U;

    return (uint8_t)((o4 << 3) | (o3 << 2) | (o2 << 1) | o1);
}

static Track_Action_t Track_Get_Action(uint8_t pattern)
{
    switch (pattern)
    {
        case 0x06U: /* 0 1 1 0：居中 */
            return Car_Forward;

        case 0x02U: /* 0 0 1 0 */
            return Car_SlightTurnRight;

        case 0x04U: /* 0 1 0 0 */
            return Car_SlightTurnLeft;

        case 0x01U: /* 0 0 0 1：车身偏离黑线左侧 */
        case 0x03U: /* 0 0 1 1 */
        case 0x07U: /* 0 1 1 1 */
            return Car_TurnRight;

        case 0x08U: /* 1 0 0 0：车身偏离黑线右侧 */
        case 0x0CU: /* 1 1 0 0 */
        case 0x0EU: /* 1 1 1 0 */
            return Car_TurnLeft;

        case TRACK_PATTERN_ALL_BLACK:
            return Car_TurnLeft;

        case TRACK_PATTERN_ALL_WHITE:
            return Car_Stop;

        default:
            /* 未定义的非零码型按约定左转。 */
            return Car_TurnLeft;
    }
}

void Track_Process(void)
{
    static uint8_t candidate_pattern = TRACK_PATTERN_ALL_WHITE;
    static uint8_t candidate_count;
    static uint8_t line_lost_active;
    static uint32_t line_lost_start_tick;
    static Track_Action_t last_action = Car_Stop;
    uint8_t raw_pattern = Track_Read_Pattern();

    /* 全白时短暂保持上次确认动作，按真实毫秒计时，超时后停车。 */
    if (raw_pattern == TRACK_PATTERN_ALL_WHITE)
    {
        candidate_pattern = TRACK_PATTERN_ALL_WHITE;
        candidate_count = 0U;

        if (line_lost_active == 0U)
        {
            line_lost_active = 1U;
            line_lost_start_tick = HAL_GetTick();
        }

        if ((HAL_GetTick() - line_lost_start_tick) < TRACK_LINE_LOST_HOLD_MS)
        {
            last_action();
        }
        else
        {
            /* 停车后必须等新的非零码型连续确认三次才恢复动作。 */
            last_action = Car_Stop;
            Car_Stop();
        }
        return;
    }

    if (raw_pattern != candidate_pattern)
    {
        candidate_pattern = raw_pattern;
        candidate_count = 1U;
    }
    else if (candidate_count < TRACK_PATTERN_CONFIRM_SAMPLES)
    {
        candidate_count++;
    }

    if (candidate_count >= TRACK_PATTERN_CONFIRM_SAMPLES)
    {
        /* 只有连续确认三次的非零码型才能更新动作并清除丢线计时。 */
        last_action = Track_Get_Action(candidate_pattern);
        line_lost_active = 0U;
    }

    last_action();
}

uint8_t Check_Black_Line(void)
{
    return (Track_Read_Pattern() != TRACK_PATTERN_ALL_WHITE) ? 1U : 0U;
}
