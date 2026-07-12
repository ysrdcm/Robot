#include "find.h"
#include "car.h"

// ================= 引脚宏定义 (与你的物理接线完全对应) =================
#define O4_PORT GPIOC
#define O4_PIN  GPIO_PIN_10   // 最左

#define O3_PORT GPIOD
#define O3_PIN  GPIO_PIN_12   // 左内

#define O2_PORT GPIOD
#define O2_PIN  GPIO_PIN_4    // 右内

#define O1_PORT GPIOD
#define O1_PIN  GPIO_PIN_14   // 最右
// =======================================================================

/*
 * 红外探头逻辑说明（常见TCRT5000）：
 * - 压黑线（吸收红外光）：DO 输出 高电平 (1)
 * - 压白地（反射红外光）：DO 输出 低电平 (0)
 * 如果你的模块逻辑相反（压黑线是0），把下面所有的 1 和 0 互换即可。
 */

void Track_Process(void)
{
    // 增加一个静态变量，用于记录连续全白的次数
    static uint16_t white_miss_count = 0;

    // 1. 读取四个探头的电平状态
    uint8_t val_o4 = HAL_GPIO_ReadPin(O4_PORT, O4_PIN);
    uint8_t val_o3 = HAL_GPIO_ReadPin(O3_PORT, O3_PIN);
    uint8_t val_o2 = HAL_GPIO_ReadPin(O2_PORT, O2_PIN);
    uint8_t val_o1 = HAL_GPIO_ReadPin(O1_PORT, O1_PIN);

    // ================== 循迹逻辑分支 ==================

    // 状态 A：居中直行 (0 1 1 0)
    if (val_o4 == 0 && val_o3 == 1 && val_o2 == 1 && val_o1 == 0)
    {
        white_miss_count = 0; // 只要识别到黑线，全白计数器立即清零
        Car_Forward();
    }
    // 状态 B：小车偏you (0 0 1 0)
    else if (val_o4 == 0 && val_o3 == 0 && val_o2 == 1 && val_o1 == 0)
    {
        white_miss_count = 0;
        Car_SlightTurnLeft();
    }
    // 状态 C：小车偏zuo (0 1 0 0)
    else if (val_o4 == 0 && val_o3 == 1 && val_o2 == 0 && val_o1 == 0)
    {
        white_miss_count = 0;
        Car_SlightTurnRight();
    }
    // 状态 D：严重偏左 (需要大角度右转)
    else if ((val_o4 == 0 && val_o3 == 0 && val_o2 == 1 && val_o1 == 1) ||
             (val_o4 == 0 && val_o3 == 0 && val_o2 == 0 && val_o1 == 1) ||
             (val_o4 == 0 && val_o3 == 1 && val_o2 == 1 && val_o1 == 1))
    {
        white_miss_count = 0;
        Car_TurnLeft();
    }
    // 状态 E：严重偏右 (需要大角度左转)
    else if ((val_o4 == 1 && val_o3 == 1 && val_o2 == 0 && val_o1 == 0) ||
             (val_o4 == 1 && val_o3 == 0 && val_o2 == 0 && val_o1 == 0) ||
             (val_o4 == 1 && val_o3 == 1 && val_o2 == 1 && val_o1 == 0))
    {
        white_miss_count = 0;
        Car_TurnRight();
    }
    // 状态 F：十字路口或全黑 (1 1 1 1)
    else if (val_o4 == 1 && val_o3 == 1 && val_o2 == 1 && val_o1 == 1)
    {
        white_miss_count = 0;
        Car_Forward();
    }
    // 状态 G：全白脱轨 (0 0 0 0)
    else if (val_o4 == 0 && val_o3 == 0 && val_o2 == 0 && val_o1 == 0)
    {
        // 累加脱轨次数
        white_miss_count++;

        // 假设控制线程(ctrl_thread)的执行频率是 100Hz (休眠10ms)
        // 50 次循环即代表 0.5 秒。
        if (white_miss_count > 50)
        {
            // 超过 0.5 秒仍为全白，说明彻底脱轨跑偏，安全停车
            Car_Stop();
        }
        else
        {
            // 0.5 秒内的短暂全白（例如遇到反光、细小断缝或急弯甩尾途中）
            // 这里什么也不做 (不调用任何 Car_xxx 函数)。
            // 底盘将保持全白发生前一瞬间的姿态（比如正在左转或直行）依靠惯性继续动作。
        }
    }
}

// 只要有任何一个探头踩到黑线(1)，就返回 1 (代表找到路了)
uint8_t Check_Black_Line(void)
{
    uint8_t val_o4 = HAL_GPIO_ReadPin(O4_PORT, O4_PIN);
    uint8_t val_o3 = HAL_GPIO_ReadPin(O3_PORT, O3_PIN);
    uint8_t val_o2 = HAL_GPIO_ReadPin(O2_PORT, O2_PIN);
    uint8_t val_o1 = HAL_GPIO_ReadPin(O1_PORT, O1_PIN);

    if (val_o4 == 1 || val_o3 == 1 || val_o2 == 1 || val_o1 == 1) {
        return 1; // 找到黑线
    }
    return 0;     // 全白
}



