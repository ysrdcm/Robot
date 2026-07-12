/**
 ****************************************************************************************************
 * @file        app.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app.c文件
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 N647开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#include "app.h"
#include "app_config.h"
#include "app_utils.h"
#include "app_lcd.h"
#include "app_camera.h"
#include "app_bqueue.h"
#include "app_cpuload.h"
#include "app_postprocess.h"
#include "tx_api.h"
#include "cmw_camera.h"
#include "ll_aton_runtime.h"

// ==================================
#include "tracker.h"
#include "beep.h"
#include "car.h"
#include "wave.h"
#include "find.h"

extern TIM_HandleTypeDef htim15;  //nihao

static TX_MUTEX ai_data_lock;
static uint8_t  ai_person_detected = 0;
static float    ai_person_x = 0;
static float    ai_person_y = 0;
static float    ai_person_w = 0;
static float    ai_person_h = 0;

static uint16_t raw_detect_confirm_cnt = 0; // 连续检测到人的帧计数器
static uint16_t raw_loss_confirm_cnt = 0;   // 连续丢失目标的帧计数器
static float    smooth_w = 0;
static float    smooth_h = 0;

// 定义机器人的四大状态
typedef enum {
    STATE_PATROL,    // 巡视
    STATE_AVOID,     // 避障
    STATE_WARNING,   // 示警（蜂鸣器响）
    STATE_COMBAT     // 战斗（激光开火）
} RobotState_t;

// 新增控制线程
static TX_THREAD ctrl_thread;
static UCHAR ctrl_thread_stack[2048];
static VOID ctrl_thread_entry(ULONG id);
// =================================================

typedef struct {
    int32_t nb_detect;
    od_pp_outBuffer_t detects[AI_OBJDETECT_YOLOV2_PP_MAX_BOXES_LIMIT];
    uint32_t nn_period_ms;
    uint32_t inf_ms;
    uint32_t pp_ms;
    uint32_t disp_ms;
} app_display_info_t;

typedef struct {
    TX_SEMAPHORE update;
    TX_MUTEX lock;
    app_display_info_t info;
} app_display_t;

static TX_SEMAPHORE isp_semaphore;

static void app_camera_display_pipe_vsync_cb(void);
static void app_camera_display_pipe_frame_cb(void);
static void app_camera_nn_pipe_frame_cb(void);

static TX_THREAD nn_thread;
static UCHAR nn_thread_stack[4096];
static TX_THREAD pp_thread;
static UCHAR pp_thread_stack[4096];
static TX_THREAD dp_thread;
static UCHAR dp_thread_stack[4096];
static TX_THREAD isp_thread;
static UCHAR isp_thread_stack[4096];

static VOID nn_thread_entry(ULONG id);
static VOID pp_thread_entry(ULONG id);
static VOID dp_thread_entry(ULONG id);
static VOID isp_thread_entry(ULONG id);

static app_display_t display;

LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default);

static uint8_t nn_input_buffers[2][NN_WIDTH * NN_HEIGHT * NN_BPP] __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static app_bqueue_t nn_input_queue;
static uint8_t nn_output_buffers[2][ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32)] __attribute__((aligned(32)));
static app_bqueue_t nn_output_queue;
static const char *nn_classes_table[NN_CLASSES] = NN_CLASSES_TABLE;

static app_cpuload_t cpuload;

// =================================================
void Camera_Servo_Update(float target_x, uint8_t is_tracking)
{
    static float cam_pwm = 1500.0f;
    static int8_t auto_dir = 1;

    // 引入平滑目标点滤波
    static float filtered_target_x = 0.5f;

    if (is_tracking) {
        // 1. 双重过滤：只有当前检测点与已滤波点的绝对偏差大于 0.05 (5%画面) 时，才去混合新数据
        if (fabs(target_x - filtered_target_x) > 0.05f) {
            // 一阶低通滤波：新数据只占 10% 的权重，90% 保持原样，彻底抹平高频抖动
            filtered_target_x = filtered_target_x * 0.90f + target_x * 0.10f;
        }

        // 2. 映射到PWM脉宽
        float target_pwm = 2500 - filtered_target_x * 2000;

        // 3. 减小硬件逼近系数：从 0.15f 降到 0.06f，让舵机更温柔、更平滑地滑向目标点
        cam_pwm += 0.06f * (target_pwm - cam_pwm);
    } else {
        // 巡视模式：左右扫视
        cam_pwm += auto_dir * 2.0f;
        if (cam_pwm > 2300) { cam_pwm = 2300; auto_dir = -1; }
        if (cam_pwm < 700)  { cam_pwm = 700;  auto_dir = 1; }
        filtered_target_x = 0.5f; // 重置历史值
    }
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (uint32_t)cam_pwm);
}

static VOID ctrl_thread_entry(ULONG id)
{
    RobotState_t state = STATE_PATROL;

    uint32_t state_start_time = 0;
    float    lock_width = 0;
    float    lock_height = 0;

    while(1)
    {
        // 1. 获取传感器与AI数据
        float dist = Wave_Get_Distance();

        if (HAL_GetTick() < 2000) {
                    dist = -1.0f;
                }

        tx_mutex_get(&ai_data_lock, TX_WAIT_FOREVER);
        uint8_t p_det = ai_person_detected;
        float p_x = ai_person_x; float p_y = ai_person_y;
        float p_w = ai_person_w; float p_h = ai_person_h;
        tx_mutex_put(&ai_data_lock);

        // 2. 无论什么状态，只要有目标，云台和摄像头就要跟踪
        Camera_Servo_Update(p_x, p_det);
        Gimbal_Targeting_Update(p_x, p_y, p_det);

        // 3. 状态机流转
        switch(state)
        {
            case STATE_PATROL:
                Laser_Fire(0); // 确保激光关闭
                BEEP(0);       // 确保蜂鸣器关闭

                if (p_det) {
                    // 发现敌人 -> 切入示警模式
                    Car_Stop();
                    state = STATE_WARNING;
                    state_start_time = HAL_GetTick();
                    lock_width = p_w;   // 记录初始大小，用于判断是否靠近
                    lock_height = p_h;
                }
                else if (dist > 0 && dist < 25.0f) {
                    // 前方25cm有障碍物 -> 切入避障模式
                    Car_Stop();
                    state = STATE_AVOID;
                    state_start_time = HAL_GetTick();
                }
                else {
                	Track_Process();
                }
                break;

            case STATE_AVOID:
                // 战斗优先级最高，避障时发现人也立刻进入示警
                if (p_det) {
                    Car_Stop();
                    state = STATE_WARNING;
                    state_start_time = HAL_GetTick();
                    lock_width = p_w;
                    lock_height = p_h;
                } else {
                    // 简单的超声波避障序列
                    uint32_t elapsed = HAL_GetTick() - state_start_time;

                    if (elapsed < 400) {

                        Car_Backward(); // 后退
                    }
                    else if (elapsed < 900) {
                        Car_TurnLeft();
                    }
                    // 阶段 3：无限期弧线绕行，直到找到黑线为止！
                    else {
                    	// 【核心寻线判定】：如果绕行途中，底部任何一个探头踩到了黑线
                    	if (Check_Black_Line() == 1) {
                    		Car_Stop();             // 刹车稳住重心
                    		state = STATE_PATROL;   // 完美回归循迹巡逻模式！
                    	}
                    	else {
                    		// 还没找到线，继续兜圈子绕行
                    		// 【动态防撞】：绕圈子时，如果超声波发现右侧车头离障碍物太近了(小于15cm)
                    		if (dist > 0 && dist < 15.0f) {
                    			Car_TurnLeft(); // 向左躲避一下，把绕行半径扩大
                    		} else {
                    			// 安全距离下，让小车画一个“向右的圆弧” (一边前进一边向右拐)
                    			// 这样刚好能贴着障碍物绕一个半圆回到黑线上
                    			Car_SlightTurnRight();
                    		}
                    	}
                    }
                }
                break;

            case STATE_WARNING:
                if (!p_det) {
                    // 人跑了，解除警报
                    state = STATE_PATROL;
                } else {
                    uint32_t elapsed = HAL_GetTick() - state_start_time;

                    // 蜂鸣器滴、滴、滴响3次 (每次周期500ms：250ms响，250ms停)
                    // 3次需要 1500ms
                    if (elapsed < 1800) {// 阶段0(0-300ms):响, 阶段1(300-600ms):停, 阶段2:响...
                        uint32_t stage = elapsed / 300;
                        if (stage % 2 == 0) {
                            BEEP(1); // 偶数阶段响
                        } else {
                            BEEP(0); // 奇数阶段停
                        }

                        // 判别靠近的威胁评估逻辑（保持不变）
                        if ((p_w > lock_width * 1.15f) && (p_h > lock_height * 1.15f)) {
                            BEEP(0);
                            state = STATE_COMBAT;
                        }
                    } else {
                        // 1.8秒后（三次示警完全结束），目标还在，进入战斗
                        BEEP(0);
                        state = STATE_COMBAT;
                    }
                    // 👆==================================================👆

                }
                break;

            case STATE_COMBAT:
                if (!p_det) {
                    // 敌人消失在视野中，战斗结束，继续巡视
                    state = STATE_PATROL;
                } else {
                    // 持续开火！
                    Laser_Fire(1);
                }
                break;
        }

        // 线程休眠 20ms (大约50Hz的控制频率，既不占用大量CPU又足够顺滑)
        // ThreadX默认1个Tick=10ms，所以休眠2个Tick
        tx_thread_sleep(1);
    }
}
// =================================================

static void app_display_network_output(app_display_info_t *display_info);

void app_run(void)
{
    app_lcd_init();

    app_bqueue_init(&nn_input_queue, 2, (uint8_t *[2]){nn_input_buffers[0], nn_input_buffers[1]});
    app_bqueue_init(&nn_output_queue, 2, (uint8_t *[2]){nn_output_buffers[0], nn_output_buffers[1]});
    app_cpuload_init(&cpuload);
    app_camera_init(app_camera_display_pipe_vsync_cb, app_camera_display_pipe_frame_cb, NULL, app_camera_nn_pipe_frame_cb);

    tx_semaphore_create(&isp_semaphore, NULL, 0);
    tx_semaphore_create(&display.update, NULL, 0);
    tx_mutex_create(&display.lock, NULL, TX_INHERIT);

    // 👇============= 新增：初始化AI数据锁 =============👇
    tx_mutex_create(&ai_data_lock, "AI Lock", TX_INHERIT);
    // 👆===============================================👆

    app_camera_display_pipe_start(app_lcd_get_bg_buffer(), CMW_MODE_CONTINUOUS);

    tx_thread_create(&nn_thread, "NN Thread", nn_thread_entry, 0, nn_thread_stack, sizeof(nn_thread_stack), TX_MAX_PRIORITIES - 3, TX_MAX_PRIORITIES - 3, 10, TX_AUTO_START);
    tx_thread_create(&pp_thread, "PP Thread", pp_thread_entry, 0, pp_thread_stack, sizeof(pp_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    tx_thread_create(&dp_thread, "DP Thread", dp_thread_entry, 0, dp_thread_stack, sizeof(dp_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    tx_thread_create(&isp_thread, "ISP Thread", isp_thread_entry, 0, isp_thread_stack, sizeof(isp_thread_stack), TX_MAX_PRIORITIES - 4, TX_MAX_PRIORITIES - 4, 10, TX_AUTO_START);

    // 👇============= 新增：创建并启动控制线程 =============👇
    // 优先级设为 TX_MAX_PRIORITIES - 2 (和 PP Thread 平级，确保响应迅速)
    tx_thread_create(&ctrl_thread, "Ctrl Thread", ctrl_thread_entry, 0, ctrl_thread_stack, sizeof(ctrl_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    // 👆==================================================👆
}

static void app_camera_display_pipe_vsync_cb(void)
{
    tx_semaphore_put(&isp_semaphore);
}

static void app_camera_display_pipe_frame_cb(void)
{
    app_lcd_switch_bg_buffer();
    app_camera_display_pipe_set_address(app_lcd_get_bg_buffer());
}

static void app_camera_nn_pipe_frame_cb(void)
{
    uint8_t *buffer;

    buffer = app_bqueue_get_free(&nn_input_queue, 0);
    if (buffer != NULL)
    {
        app_camera_nn_pipe_set_address(buffer);
        app_bqueue_put_ready(&nn_input_queue);
    }
}

static VOID nn_thread_entry(ULONG id)
{
    uint32_t nn_out_len;
    uint32_t nn_in_len;
    uint8_t *nn_pipe_dst;
    uint8_t *capture_buffer;
    uint8_t *output_buffer;
    uint32_t nn_period[2];
    uint32_t nn_period_ms;
    uint32_t time_stamp;
    uint32_t inf_ms;

    nn_in_len = LL_Buffer_len(LL_ATON_Input_Buffers_Info_Default());
    nn_out_len = LL_Buffer_len(LL_ATON_Output_Buffers_Info_Default());

    nn_period[1] = HAL_GetTick();

    nn_pipe_dst = app_bqueue_get_free(&nn_input_queue, 0);

    app_camera_nn_pipe_start(nn_pipe_dst, CMW_MODE_CONTINUOUS);

    while (1)
    {
        nn_period[0] = nn_period[1];
        nn_period[1] = HAL_GetTick();
        nn_period_ms = nn_period[1] - nn_period[0];

        capture_buffer = app_bqueue_get_ready(&nn_input_queue);
        output_buffer = app_bqueue_get_free(&nn_output_queue, 1);

        time_stamp = HAL_GetTick();
        LL_ATON_Set_User_Input_Buffer_Default(0, capture_buffer, nn_in_len);
        SCB_InvalidateDCache_by_Addr(output_buffer, nn_out_len);
        LL_ATON_Set_User_Output_Buffer_Default(0, output_buffer, nn_out_len);
        LL_ATON_RT_Main(&NN_Instance_Default);
        inf_ms = HAL_GetTick() - time_stamp;

        app_bqueue_put_free(&nn_input_queue);
        app_bqueue_put_ready(&nn_output_queue);

        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        display.info.inf_ms = inf_ms;
        display.info.nn_period_ms = nn_period_ms;
        tx_mutex_put(&display.lock);
    }
}

static VOID pp_thread_entry(ULONG id)
{
    yolov2_pp_static_param_t pp_params;
    uint8_t *output_buffer;
    od_pp_out_t pp_output;
    uint32_t nn_pp[2];
    int32_t i;

    app_postprocess_init(&pp_params);

    while (1)
    {
        output_buffer = app_bqueue_get_ready(&nn_output_queue);
        pp_output.pOutBuff = NULL;

        nn_pp[0] = HAL_GetTick();
        app_postprocess_run((void *[]){(void *)output_buffer}, 1, &pp_output, &pp_params);
        nn_pp[1] = HAL_GetTick();

        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        display.info.nb_detect = pp_output.nb_detect;
        for (i = 0; i < pp_output.nb_detect; i++)
        {
            display.info.detects[i] = pp_output.pOutBuff[i];
        }
        display.info.pp_ms = nn_pp[1] - nn_pp[0];
        tx_mutex_put(&display.lock);


        // ---------- 👇 修改后的核心数据传递区 👇 ----------
        // 拿互斥锁，安全地将AI数据写入全局变量，供控制线程(ctrl_thread)决策使用
        tx_mutex_get(&ai_data_lock, TX_WAIT_FOREVER);

        // 提高基础置信度到 0.25f，并提取人体感知结果
        if (pp_output.nb_detect > 0 && pp_output.pOutBuff != NULL && pp_output.pOutBuff[0].conf > 0.25f)
        {
            // 获取检测到的第一个目标
            od_pp_outBuffer_t *person = &pp_output.pOutBuff[0];

            raw_loss_confirm_cnt = 0; // 既然看到了人，丢失计数器立刻清零

            // 1. 解决椅子瞬间误检：必须连续 4 帧都看到目标，才真正触发系统响应
            if (raw_detect_confirm_cnt < 4) {
                raw_detect_confirm_cnt++;
            }

            // 只有当连续4帧都确认是人时，才允许修改控制变量
            if (raw_detect_confirm_cnt >= 4)
            {
                ai_person_detected = 1;              // 标志位锁定为：检测到人
                ai_person_x = person->x_center;      // X坐标直接使用当前帧供云台追踪
                ai_person_y = person->y_center;      // Y坐标直接使用当前帧供云台追踪

                // 2. 解决检测框忽大忽小：使用低通滑动滤波器平滑尺寸
                if (smooth_w == 0) {
                    // 第一次锁定目标时，赋予初值
                    smooth_w = person->width;
                    smooth_h = person->height;
                } else {
                    // 后续检测：老尺寸占 85% 权重，新尺寸占 15% 权重
                    // 彻底抹平一瞬间框变大的噪点
                    smooth_w = smooth_w * 0.85f + person->width * 0.15f;
                    smooth_h = smooth_h * 0.85f + person->height * 0.15f;
                }

                // 将平滑后的稳如泰山的尺寸，交给全局变量
                ai_person_w = smooth_w;
                ai_person_h = smooth_h;
            }
        }
        else
        {
            // 这一帧什么都没看到，或者置信度太低（噪点）
            raw_detect_confirm_cnt = 0; // 连续检测计数器立刻被破坏、清零

            // 3. 目标丢失消抖：必须连续 8 帧都没看到人，才认为人彻底离开了
            if (ai_person_detected == 1) {
                raw_loss_confirm_cnt++;
                if (raw_loss_confirm_cnt >= 8) {
                    ai_person_detected = 0;
                    smooth_w = 0; // 目标彻底消失，平滑尺寸也要归零重置
                    smooth_h = 0;
                }
            }
        }

        tx_mutex_put(&ai_data_lock);
        // ---------- 👆 数据传递区结束 👆 ----------

        app_bqueue_put_free(&nn_output_queue);
        tx_semaphore_ceiling_put(&display.update, 1);
    }
}

static VOID dp_thread_entry(ULONG id)
{
    uint32_t disp_ms = 0;
    app_display_info_t display_info;
    uint32_t time_stamp;

    while (1)
    {
        tx_semaphore_get(&display.update, TX_WAIT_FOREVER);
        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        display_info = display.info;
        tx_mutex_put(&display.lock);
        display_info.disp_ms = disp_ms;

        time_stamp = HAL_GetTick();
        app_display_network_output(&display_info);
        disp_ms = HAL_GetTick() - time_stamp;
    }
}


static VOID isp_thread_entry(ULONG id)
{
    while (1)
    {
        tx_semaphore_get(&isp_semaphore, TX_WAIT_FOREVER);

        app_camera_isp_update();
    }
}

static uint8_t app_clamp_point(int32_t *x, int32_t *y)
{
    int32_t xi;
    int32_t yi;

    xi = *x;
    yi = *y;

    if (*x < 0)
    {
        *x = 0;
    }

    if (*y < 0)
    {
        *y = 0;
    }

    if (*x >= LCD_BG_WIDTH)
    {
        *x = LCD_BG_WIDTH - 1;
    }

    if (*y >= LCD_BG_HEIGHT)
    {
        *y = LCD_BG_HEIGHT - 1;
    }

    return (xi != *x) || (yi != *y);
}

static void app_display_detection(od_pp_outBuffer_t *detect)
{
    int32_t xc;
    int32_t yc;
    int32_t x0;
    int32_t y0;
    int32_t x1;
    int32_t y1;
    int32_t w;
    int32_t h;

    xc = (int32_t)(LCD_BG_WIDTH * detect->x_center);
    yc = (int32_t)(LCD_BG_HEIGHT * detect->y_center);
    w = (int32_t)(LCD_BG_WIDTH * detect->width);
    h = (int32_t)(LCD_BG_HEIGHT * detect->height);

    x0 = xc - (w + 1) / 2;
    y0 = yc - (h + 1) / 2;
    x1 = xc + (w + 1) / 2;
    y1 = yc + (h + 1) / 2;

    app_clamp_point(&x0, &y0);
    app_clamp_point(&x1, &y1);

    UTIL_LCD_DrawRect(x0, y0, x1 - x0, y1 - y0, UTIL_LCD_COLOR_GREEN);
    UTIL_LCDEx_PrintfAt(x0, y0, LEFT_MODE, nn_classes_table[detect->class_index]);
}

static void app_display_network_output(app_display_info_t *display_info)
{
    float cpuload_one_second;
    uint8_t line_nb = 0;
    int32_t i;

    app_lcd_draw_area_update();

    UTIL_LCD_FillRect(0, 0, LCD_FG_WIDTH, LCD_FG_HEIGHT, 0x00000000);

    app_cpuload_update(&cpuload);
    app_cpuload_get_info(&cpuload, NULL, &cpuload_one_second, NULL);

    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "CPU load");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%.1f%%", cpuload_one_second);
    line_nb += 2;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Inference");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%ums", display_info->inf_ms);
    line_nb += 2;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "FPS");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%.2f", 1000.0 / display_info->nn_period_ms);
    line_nb += 2;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Peoples");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%u", display_info->nb_detect);

    for (i = 0; i < display_info->nb_detect; i++)
    {
        app_display_detection(&display_info->detects[i]);
    }

    app_lcd_draw_area_commit();
}
