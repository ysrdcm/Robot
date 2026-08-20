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
#include <stdio.h>
#include <string.h>

// ==================================
#include "tracker.h"
#include "beep.h"
#include "car.h"
#include "wave.h"
#include "find.h"
#include "esp8266_log.h"
#include "rc100.h"

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

/* CODEX 2026-07-16: Robot-control thresholds added in this pass. */
#define PERSON_CLASS_INDEX                 0
#define ULTRASONIC_SAMPLE_PERIOD_MS        60U
/* CODEX 2026-07-24: RC-100B manual-control safety and servo limits. */
#define RC100_LINK_TIMEOUT_MS              500U
#define MANUAL_OBSTACLE_STOP_CM            20.0f
#define CAMERA_SERVO_PWM_MIN               700.0f
#define CAMERA_SERVO_PWM_MAX               2300.0f
#define CAMERA_SERVO_UPDATE_PERIOD_MS      20U
#define CAMERA_SERVO_MANUAL_STEP           10.0f
/* CODEX 2026-07-26: Stable visual-servo limits for automatic camera pan. */
#define CAMERA_SERVO_AUTO_DEADBAND          0.06f
/* CODEX 2026-07-27: Increase all camera-pan speeds slightly while retaining the fixed cadence. */
#define CAMERA_SERVO_AUTO_GAIN             24.0f
#define CAMERA_SERVO_AUTO_MAX_STEP          7.5f
#define CAMERA_SERVO_SCAN_STEP              6.0f
#define SERVO_DETECTION_HOLD_MS             300U
/* CODEX 2026-07-26: DCMIPP frame watchdog and bounded recovery retry. */
#define CAMERA_FRAME_TIMEOUT_MS            2000U
#define CAMERA_RECOVERY_RETRY_MS           1500U

/* CODEX 2026-07-20: Event snapshot settings. BMP avoids treating raw RGB as JPEG. */
#define PERSON_CONFIRM_FRAMES              4U
#define PERSON_LOSS_FRAMES                 8U
#define SNAPSHOT_WIDTH                     (NN_WIDTH / 2U)
#define SNAPSHOT_HEIGHT                    (NN_HEIGHT / 2U)
#define SNAPSHOT_ROW_BYTES                 (((SNAPSHOT_WIDTH * 3U) + 3U) & ~3U)
#define SNAPSHOT_HEADER_BYTES              54U
#define SNAPSHOT_BMP_SIZE                  (SNAPSHOT_HEADER_BYTES + (SNAPSHOT_ROW_BYTES * SNAPSHOT_HEIGHT))
/* CODEX 2026-07-24: Keep HTTP buffers bounded; the image is served separately. */
#define WIFI_HTTP_REQUEST_SIZE             1024U
#define WIFI_PAGE_BUFFER_SIZE              4096U
#define WIFI_IPD_HEADER_SIZE               96U

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
/* CODEX 2026-07-24: Expose remote mode to the LCD diagnostics layer. */
static volatile uint8_t rc100_manual_mode_display;

static void app_camera_display_pipe_vsync_cb(void);
static void app_camera_display_pipe_frame_cb(void);
static void app_camera_nn_pipe_frame_cb(void);
static volatile uint32_t camera_display_last_frame_tick;
static volatile uint32_t camera_nn_last_frame_tick;
static volatile uint32_t camera_recovery_count;

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

// =================================================
// 声明 WiFi 线程
static TX_THREAD wifi_thread;
/* CODEX 2026-07-20: HTTP formatting and AT parsing need more than the old 2 KB stack. */
static UCHAR wifi_thread_stack[4096];
static VOID wifi_thread_entry(ULONG id);

// 用于通知 WiFi 线程“抓拍完成，开始上传”的信号量

// 图片缓存区 (存放置信度最高的一帧)
/* CODEX 2026-07-20: Keep each thumbnail paired with its NN output queue slot. */
static uint8_t nn_snapshot_buffers[2][SNAPSHOT_BMP_SIZE]
    __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static uint8_t event_best_image[SNAPSHOT_BMP_SIZE]
    __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static uint8_t latest_event_image[SNAPSHOT_BMP_SIZE]
    __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static uint8_t wifi_tx_image[SNAPSHOT_BMP_SIZE]
    __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
/* CODEX 2026-07-24: Shared page buffer keeps the Wi-Fi thread stack small. */
static uint8_t wifi_page_buffer[WIFI_PAGE_BUFFER_SIZE]
    __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static TX_MUTEX snapshot_lock;
static float event_best_confidence = -1.0f;
static float latest_event_confidence = 0.0f;
static uint32_t latest_event_tick = 0;
static uint32_t latest_event_id = 0;
static uint8_t person_event_active = 0;
// =================================================

/* CODEX 2026-07-20: Build a small browser-readable BMP from the exact RGB888 NN frame. */
static void snapshot_write_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
}

static void snapshot_write_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
}

static void snapshot_build_bmp(uint8_t *bmp, const uint8_t *rgb888)
{
    uint32_t x;
    uint32_t y;

    memset(bmp, 0, SNAPSHOT_BMP_SIZE);
    bmp[0] = 'B';
    bmp[1] = 'M';
    snapshot_write_u32(&bmp[2], SNAPSHOT_BMP_SIZE);
    snapshot_write_u32(&bmp[10], SNAPSHOT_HEADER_BYTES);
    snapshot_write_u32(&bmp[14], 40U);
    snapshot_write_u32(&bmp[18], SNAPSHOT_WIDTH);
    snapshot_write_u32(&bmp[22], SNAPSHOT_HEIGHT);
    snapshot_write_u16(&bmp[26], 1U);
    snapshot_write_u16(&bmp[28], 24U);
    snapshot_write_u32(&bmp[34], SNAPSHOT_ROW_BYTES * SNAPSHOT_HEIGHT);

    for (y = 0; y < SNAPSHOT_HEIGHT; y++)
    {
        const uint8_t *src_row = &rgb888[(y * 2U) * NN_WIDTH * NN_BPP];
        uint8_t *dst_row = &bmp[SNAPSHOT_HEADER_BYTES +
                                ((SNAPSHOT_HEIGHT - 1U - y) * SNAPSHOT_ROW_BYTES)];

        for (x = 0; x < SNAPSHOT_WIDTH; x++)
        {
            const uint8_t *src = &src_row[(x * 2U) * NN_BPP];
            uint8_t *dst = &dst_row[x * 3U];

            dst[0] = src[2];
            dst[1] = src[1];
            dst[2] = src[0];
        }
    }
}

static uint8_t *snapshot_for_output_buffer(const uint8_t *output_buffer)
{
    if (output_buffer == nn_output_buffers[0])
    {
        return nn_snapshot_buffers[0];
    }
    if (output_buffer == nn_output_buffers[1])
    {
        return nn_snapshot_buffers[1];
    }
    return NULL;
}

static void snapshot_publish_event(void)
{
    tx_mutex_get(&snapshot_lock, TX_WAIT_FOREVER);
    memcpy(latest_event_image, event_best_image, SNAPSHOT_BMP_SIZE);
    latest_event_confidence = event_best_confidence;
    latest_event_tick = HAL_GetTick();
    latest_event_id++;
    tx_mutex_put(&snapshot_lock);
}

/* CODEX 2026-07-24: Parse one complete +IPD frame using its declared payload length. */
static uint8_t wifi_read_ipd_packet(uint8_t *link_id,
                                    char *request,
                                    uint32_t request_capacity,
                                    uint32_t *declared_length)
{
    static const char prefix[] = "+IPD,";
    char header[WIFI_IPD_HEADER_SIZE];
    uint32_t prefix_index = 0U;
    uint32_t header_length = 0U;
    uint32_t stored_length = 0U;
    unsigned int parsed_link = 0U;
    unsigned long parsed_length = 0U;
    uint8_t byte;

    if ((link_id == NULL) || (request == NULL) ||
        (request_capacity < 2U) || (declared_length == NULL))
    {
        return 1U;
    }

    for (;;)
    {
        if (ESP8266_Log_UART_ReadByte(&byte, 100U) != 0U)
        {
            tx_thread_sleep(1);
            continue;
        }

        if (byte == (uint8_t)prefix[prefix_index])
        {
            prefix_index++;
            if (prefix_index == (sizeof(prefix) - 1U))
            {
                break;
            }
        }
        else
        {
            prefix_index = (byte == (uint8_t)prefix[0]) ? 1U : 0U;
        }
    }

    while (header_length < (sizeof(header) - 1U))
    {
        if (ESP8266_Log_UART_ReadByte(&byte, 2000U) != 0U)
        {
            return 1U;
        }
        if (byte == (uint8_t)':')
        {
            break;
        }
        header[header_length++] = (char)byte;
    }
    if ((byte != (uint8_t)':') || (header_length == 0U))
    {
        return 1U;
    }
    header[header_length] = '\0';

    if ((sscanf(header, "%u,%lu", &parsed_link, &parsed_length) != 2) ||
        (parsed_link > 4U) || (parsed_length > 65535UL))
    {
        return 1U;
    }

    for (unsigned long index = 0UL; index < parsed_length; index++)
    {
        if (ESP8266_Log_UART_ReadByte(&byte, 2000U) != 0U)
        {
            return 1U;
        }
        if (stored_length < (request_capacity - 1U))
        {
            request[stored_length++] = (char)byte;
        }
    }

    request[stored_length] = '\0';
    *link_id = (uint8_t)parsed_link;
    *declared_length = (uint32_t)parsed_length;
    return 0U;
}

static uint32_t wifi_build_event_page(void)
{
    uint32_t event_id;
    uint32_t event_tick;
    uint32_t hours;
    uint32_t minutes;
    uint32_t seconds;
    uint32_t page_length;
    float confidence;
    uint8_t image_available;
    int written;

    tx_mutex_get(&snapshot_lock, TX_WAIT_FOREVER);
    event_id = latest_event_id;
    event_tick = latest_event_tick;
    confidence = latest_event_confidence;
    image_available = (latest_event_id > 0U);
    tx_mutex_put(&snapshot_lock);

    hours = event_tick / 3600000U;
    minutes = (event_tick / 60000U) % 60U;
    seconds = (event_tick / 1000U) % 60U;

    written = snprintf((char *)wifi_page_buffer, WIFI_PAGE_BUFFER_SIZE,
        "<!doctype html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<link rel=\"icon\" href=\"data:,\">"
        "<title>&#24033;&#38450;&#26426;&#22120;&#20154;&#20107;&#20214;&#26085;&#24535;</title>"
        "<style>"
        "*{box-sizing:border-box}body{margin:0;background:#101214;color:#f4f5f6;"
        "font-family:system-ui,-apple-system,\"Microsoft YaHei\",sans-serif}"
        "main{width:min(100%%,720px);margin:0 auto;padding:24px 18px 40px}"
        "header{display:flex;align-items:center;justify-content:space-between;gap:16px;"
        "border-bottom:1px solid #34383c;padding-bottom:16px}"
        "h1{font-size:28px;line-height:1.25;margin:0;font-weight:700}"
        "button{border:0;border-radius:6px;background:#2f81f7;color:white;padding:10px 15px;"
        "font-size:16px;font-weight:650;white-space:nowrap}"
        ".meta{display:grid;grid-template-columns:repeat(3,1fr);gap:1px;margin:20px 0;"
        "background:#34383c;border:1px solid #34383c;border-radius:6px;overflow:hidden}"
        ".item{background:#191c1f;padding:12px}.label{display:block;color:#9da5ad;"
        "font-size:13px;margin-bottom:5px}.value{font-size:18px;font-weight:650}"
        "figure{margin:0;border:1px solid #34383c;background:#08090a;border-radius:6px;"
        "overflow:hidden}img{display:block;width:100%%;max-height:70vh;aspect-ratio:1/1;"
        "object-fit:contain;image-rendering:auto}.empty{padding:64px 20px;text-align:center;"
        "color:#b8bec5;border:1px solid #34383c;border-radius:6px}.hint{color:#8f979f;"
        "font-size:14px;line-height:1.6;margin:12px 2px 0}"
        "@media(max-width:520px){main{padding:18px 14px 32px}h1{font-size:23px}"
        ".meta{grid-template-columns:1fr}.item{display:flex;justify-content:space-between;"
        "align-items:center}.label{margin:0}.value{font-size:16px}}"
        "</style></head><body><main><header>"
        "<h1>&#24033;&#38450;&#26426;&#22120;&#20154;&#20107;&#20214;&#26085;&#24535;</h1>"
        "<button type=\"button\" onclick=\"location.reload()\">"
        "&#21047;&#26032;&#26085;&#24535;</button></header>"
        "<section class=\"meta\">"
        "<div class=\"item\"><span class=\"label\">&#20107;&#20214;</span>"
        "<span class=\"value\">#%lu</span></div>"
        "<div class=\"item\"><span class=\"label\">&#32622;&#20449;&#24230;</span>"
        "<span class=\"value\">%.1f%%</span></div>"
        "<div class=\"item\"><span class=\"label\">&#36816;&#34892;&#26102;&#38388;</span>"
        "<span class=\"value\">%02lu:%02lu:%02lu</span></div></section>",
        (unsigned long)event_id, confidence * 100.0f,
        (unsigned long)hours, (unsigned long)minutes, (unsigned long)seconds);
    if ((written <= 0) || ((uint32_t)written >= WIFI_PAGE_BUFFER_SIZE))
    {
        return 0U;
    }
    page_length = (uint32_t)written;

    if (image_available == 0U)
    {
        written = snprintf((char *)&wifi_page_buffer[page_length],
                           WIFI_PAGE_BUFFER_SIZE - page_length,
                           "<div class=\"empty\">&#23578;&#26410;&#35760;&#24405;&#21040;"
                           "&#24050;&#32467;&#26463;&#30340;&#20154;&#21592;&#20107;&#20214;&#12290;"
                           "</div></main></body></html>");
        if ((written <= 0) || ((uint32_t)written >= (WIFI_PAGE_BUFFER_SIZE - page_length)))
        {
            return 0U;
        }
        return page_length + (uint32_t)written;
    }

    /* CODEX 2026-07-24: Avoid Base64's 33% overhead and let the page render first. */
    written = snprintf((char *)&wifi_page_buffer[page_length],
                       WIFI_PAGE_BUFFER_SIZE - page_length,
                       "<figure><img alt=\"&#20107;&#20214;&#22270;&#20687;\" "
                       "src=\"/latest.bmp?id=%lu\"></figure>"
                       "<p class=\"hint\">&#22270;&#20687;&#20026;&#35813;&#20107;&#20214;&#20013;"
                       "&#32622;&#20449;&#24230;&#26368;&#39640;&#30340;&#19968;&#24103;&#12290;</p>"
                       "</main></body></html>",
                       (unsigned long)event_id);
    if ((written <= 0) || ((uint32_t)written >= (WIFI_PAGE_BUFFER_SIZE - page_length)))
    {
        return 0U;
    }
    return page_length + (uint32_t)written;
}

/* CODEX 2026-07-24: Shared position supports auto tracking and manual slow jog. */
static float camera_servo_pwm = 1500.0f;
/* CODEX 2026-07-27: Update the SG90 at a fixed 20 ms cadence. */
static uint32_t camera_servo_last_update_tick;

static void Camera_Servo_Manual_Update(int8_t direction)
{
    uint32_t now = HAL_GetTick();

    /* CODEX 2026-07-27: Decouple servo speed from the variable control-loop rate. */
    if ((now - camera_servo_last_update_tick) <
        CAMERA_SERVO_UPDATE_PERIOD_MS)
    {
        return;
    }
    camera_servo_last_update_tick = now;

    if (direction > 0)
    {
        camera_servo_pwm += CAMERA_SERVO_MANUAL_STEP;
    }
    else if (direction < 0)
    {
        camera_servo_pwm -= CAMERA_SERVO_MANUAL_STEP;
    }

    if (camera_servo_pwm > CAMERA_SERVO_PWM_MAX)
    {
        camera_servo_pwm = CAMERA_SERVO_PWM_MAX;
    }
    if (camera_servo_pwm < CAMERA_SERVO_PWM_MIN)
    {
        camera_servo_pwm = CAMERA_SERVO_PWM_MIN;
    }
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (uint32_t)camera_servo_pwm);
}

void Camera_Servo_Update(float target_x, uint8_t is_tracking)
{
    static int8_t auto_dir = 1;

    // 引入平滑目标点滤波
    static float filtered_target_x = 0.5f;
    static uint32_t last_tracking_tick;
    uint32_t now = HAL_GetTick();

    if (is_tracking)
    {
        last_tracking_tick = now;
    }

    /* CODEX 2026-07-27: Keep scan and tracking motion on a steady time base. */
    if ((now - camera_servo_last_update_tick) <
        CAMERA_SERVO_UPDATE_PERIOD_MS)
    {
        return;
    }
    camera_servo_last_update_tick = now;

    if (is_tracking) {
        // 1. 双重过滤：只有当前检测点与已滤波点的绝对偏差大于 0.05 (5%画面) 时，才去混合新数据
        if (fabs(target_x - filtered_target_x) > 0.05f) {
            // 一阶低通滤波：新数据只占 10% 的权重，90% 保持原样，彻底抹平高频抖动
            filtered_target_x = filtered_target_x * 0.90f + target_x * 0.10f;
        }

        // 2. 映射到PWM脉宽
        float error = filtered_target_x - 0.5f;
        float step = 0.0f;

        // 3. 减小硬件逼近系数：从 0.15f 降到 0.06f，让舵机更温柔、更平滑地滑向目标点
        /*
         * CODEX 2026-07-26: Incremental control holds the current angle when
         * the person is centered instead of repeatedly commanding 1500 us.
         */
        if (fabsf(error) > CAMERA_SERVO_AUTO_DEADBAND)
        {
            step = -error * CAMERA_SERVO_AUTO_GAIN;
            if (step > CAMERA_SERVO_AUTO_MAX_STEP)
            {
                step = CAMERA_SERVO_AUTO_MAX_STEP;
            }
            else if (step < -CAMERA_SERVO_AUTO_MAX_STEP)
            {
                step = -CAMERA_SERVO_AUTO_MAX_STEP;
            }
            camera_servo_pwm += step;
        }
    } else if ((last_tracking_tick == 0U) ||
               ((now - last_tracking_tick) >
                SERVO_DETECTION_HOLD_MS)) {
        // 巡视模式：左右扫视
        camera_servo_pwm += auto_dir * CAMERA_SERVO_SCAN_STEP;
        if (camera_servo_pwm > CAMERA_SERVO_PWM_MAX) {
            camera_servo_pwm = CAMERA_SERVO_PWM_MAX;
            auto_dir = -1;
        }
        if (camera_servo_pwm < CAMERA_SERVO_PWM_MIN) {
            camera_servo_pwm = CAMERA_SERVO_PWM_MIN;
            auto_dir = 1;
        }
        filtered_target_x = 0.5f; // 重置历史值
    }
    /* CODEX 2026-07-26: Clamp both incremental tracking and scan motion. */
    if (camera_servo_pwm > CAMERA_SERVO_PWM_MAX)
    {
        camera_servo_pwm = CAMERA_SERVO_PWM_MAX;
    }
    else if (camera_servo_pwm < CAMERA_SERVO_PWM_MIN)
    {
        camera_servo_pwm = CAMERA_SERVO_PWM_MIN;
    }
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (uint32_t)camera_servo_pwm);
}

/* CODEX 2026-07-24: Apply RC-100B direction bits with conflict and obstacle checks. */
static void Manual_Drive_Update(uint16_t buttons, float distance_cm)
{
    uint16_t direction = buttons & (RC100_BUTTON_UP | RC100_BUTTON_DOWN |
                                    RC100_BUTTON_LEFT | RC100_BUTTON_RIGHT);
    uint8_t conflicting = (((direction & RC100_BUTTON_UP) != 0U) &&
                           ((direction & RC100_BUTTON_DOWN) != 0U)) ||
                          (((direction & RC100_BUTTON_LEFT) != 0U) &&
                           ((direction & RC100_BUTTON_RIGHT) != 0U));
    uint8_t obstacle = (distance_cm > 0.0f) &&
                       (distance_cm < MANUAL_OBSTACLE_STOP_CM);

    if (conflicting != 0U)
    {
        Car_Stop();
    }
    else if ((obstacle != 0U) && (direction != RC100_BUTTON_DOWN) &&
             (direction != 0U))
    {
        /* Front obstacle blocks forward/turning commands but still permits retreat. */
        Car_Stop();
    }
    else if (direction == (RC100_BUTTON_UP | RC100_BUTTON_LEFT))
    {
        Car_SlightTurnLeft();
    }
    else if (direction == (RC100_BUTTON_UP | RC100_BUTTON_RIGHT))
    {
        Car_SlightTurnRight();
    }
    else if (direction == RC100_BUTTON_UP)
    {
        Car_Forward();
    }
    else if (direction == RC100_BUTTON_DOWN)
    {
        Car_Backward();
    }
    else if (direction == RC100_BUTTON_LEFT)
    {
        Car_TurnLeft();
    }
    else if (direction == RC100_BUTTON_RIGHT)
    {
        Car_TurnRight();
    }
    else
    {
        Car_Stop();
    }
}

static VOID ctrl_thread_entry(ULONG id)
{
    RobotState_t state = STATE_PATROL;
    uint8_t manual_mode = 0U;
    uint8_t mode_button_armed = 1U;

    uint32_t state_start_time = 0;
    float    lock_width = 0;
    float    lock_height = 0;
    /* CODEX 2026-07-16: Cache ultrasonic readings so HC-SR04 is not retriggered every control tick. */
    uint32_t last_ultrasonic_tick = 0;
    float    dist = -1.0f;
    uint32_t last_camera_recovery_attempt = 0U;

    while(1)
    {
        // 1. 获取传感器与AI数据
        uint32_t now = HAL_GetTick();

        if (((app_camera_recovery_requested() != 0U) ||
             ((now - camera_display_last_frame_tick) >
              CAMERA_FRAME_TIMEOUT_MS) ||
             ((now - camera_nn_last_frame_tick) >
              CAMERA_FRAME_TIMEOUT_MS)) &&
            ((now - last_camera_recovery_attempt) >=
             CAMERA_RECOVERY_RETRY_MS))
        {
            /*
             * CODEX 2026-07-26: Recover a stalled DCMIPP pipeline in thread
             * context. Do not let stale AI coordinates keep steering outputs.
             */
            last_camera_recovery_attempt = now;
            Car_Stop();
            Laser_Fire(0U);
            BEEP(0);

            tx_mutex_get(&ai_data_lock, TX_WAIT_FOREVER);
            ai_person_detected = 0U;
            raw_detect_confirm_cnt = 0U;
            raw_loss_confirm_cnt = 0U;
            tx_mutex_put(&ai_data_lock);

            if (app_camera_recover() == HAL_OK)
            {
                camera_display_last_frame_tick = HAL_GetTick();
                camera_nn_last_frame_tick = camera_display_last_frame_tick;
                camera_recovery_count++;
            }
            tx_thread_sleep(1);
            continue;
        }

        /* CODEX 2026-07-16: HC-SR04 needs a quiet interval between trigger pulses. */
        if ((now - last_ultrasonic_tick) >= ULTRASONIC_SAMPLE_PERIOD_MS)
        {
            dist = Wave_Get_Distance();
            last_ultrasonic_tick = now;
        }

        if (HAL_GetTick() < 2000) {
                    dist = -1.0f;
                }

        tx_mutex_get(&ai_data_lock, TX_WAIT_FOREVER);

        uint8_t p_det = ai_person_detected;
        float p_x = ai_person_x; float p_y = ai_person_y;
        float p_w = ai_person_w; float p_h = ai_person_h;
        tx_mutex_put(&ai_data_lock);

        // 2. 无论什么状态，只要有目标，云台和摄像头就要跟踪
        /* CODEX 2026-07-24: Manual mode has priority; gimbal tracking remains automatic. */
        {
            rc100_state_t remote_state;
            uint16_t remote_buttons = 0U;
            uint16_t mode_buttons;
            uint8_t remote_fresh;

            RC100_GetState(&remote_state);
            remote_fresh = (remote_state.has_packet != 0U) &&
                           ((now - remote_state.last_packet_tick) <= RC100_LINK_TIMEOUT_MS);
            if (remote_fresh != 0U)
            {
                remote_buttons = remote_state.buttons;
            }
            mode_buttons = remote_buttons & (RC100_BUTTON_5 | RC100_BUTTON_6);
            if (mode_buttons == 0U)
            {
                /* CODEX 2026-07-24: A release packet rearms the next mode change. */
                mode_button_armed = 1U;
            }

            /*
             * CODEX 2026-07-26: Add camera yaw to the existing image tracking
             * correction so the gimbal targets in chassis coordinates.
             */
            Gimbal_Targeting_Update(p_x, p_y, p_det, camera_servo_pwm);

            if ((mode_button_armed != 0U) &&
                (manual_mode == 0U) &&
                ((remote_buttons & RC100_BUTTON_6) != 0U))
            {
                /*
                 * CODEX 2026-07-24: Treat mechanically combined 5+6 as one
                 * press and wait for release before another mode transition.
                 */
                mode_button_armed = 0U;
                manual_mode = 1U;
                rc100_manual_mode_display = 1U;
                Car_Stop();
                Laser_Fire(0U);
                BEEP(0);
                Camera_Servo_Manual_Update(0);
                tx_thread_sleep(1);
                continue;
            }

            if ((mode_button_armed != 0U) &&
                (manual_mode != 0U) &&
                ((remote_buttons & RC100_BUTTON_5) != 0U))
            {
                /* CODEX 2026-07-24: Stop outputs before resetting autonomous state. */
                mode_button_armed = 0U;
                Car_Stop();
                Laser_Fire(0U);
                BEEP(0);
                state = STATE_PATROL;
                state_start_time = now;
                manual_mode = 0U;
                rc100_manual_mode_display = 0U;
                tx_thread_sleep(1);
                continue;
            }

            if (manual_mode != 0U)
            {
                int8_t camera_direction = 0;

                if (((remote_buttons & RC100_BUTTON_2) != 0U) &&
                    ((remote_buttons & RC100_BUTTON_4) == 0U))
                {
                    camera_direction = 1;
                }
                else if (((remote_buttons & RC100_BUTTON_4) != 0U) &&
                         ((remote_buttons & RC100_BUTTON_2) == 0U))
                {
                    camera_direction = -1;
                }

                Camera_Servo_Manual_Update(camera_direction);
                BEEP(0);
                Laser_Fire(((remote_buttons & RC100_BUTTON_1) != 0U) ? 1U : 0U);
                Manual_Drive_Update(remote_buttons, dist);
                tx_thread_sleep(1);
                continue;
            }
        }

        Camera_Servo_Update(p_x, p_det);

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
                    uint32_t elapsed = HAL_GetTick() - state_start_time;

                    const uint32_t dt_back = 200;   // 1. 后退时间
                    const uint32_t dt_turn = 2120;   // 2. 原地转 90 度所需时间（左转右转复用）
                    const uint32_t dt_fw_w = 1800;   // 3. 侧向驶出距离的时间 (决定矩形的宽)
                    const uint32_t dt_fw_l = 4450;  // 4. 平行越过障碍物的时间 (决定矩形的长)

                    // 计算时间轴节点 (累加)
                    const uint32_t t1 = dt_back;
                    const uint32_t t2 = t1 + dt_turn;
                    const uint32_t t3 = t2 + dt_fw_w;
                    const uint32_t t4 = t3 + dt_turn;
                    const uint32_t t5 = t4 + dt_fw_l;
                    const uint32_t t6 = t5 + dt_turn;
                    const uint32_t t7 = t6 + dt_fw_w * 2;
                    const uint32_t t8 = t7 + dt_turn;
                    const uint32_t t9 = t8 + dt_fw_l ;
                    const uint32_t t10 = t9 + dt_turn;
                    const uint32_t t11 = t10 + dt_fw_w;


                    if (elapsed > t2 && Check_Black_Line() == 1) {
                    	Car_Stop();
                    	state = STATE_PATROL;
                    	break; // 触发打断，直接跳出本轮状态机
                    }

                    // 3. 全局打断机制 B：避障途中遭遇二次障碍物！
                    // 如果在后退之后的任何移动中，前方突然又出现不足 15cm 的障碍物
                    if (elapsed > t1 && dist > 0 && dist < 15.0f) {
                    	Car_Stop();
                    	// 核心：直接重置状态机时间戳，把这个新障碍物当成一次全新的避障任务（重新后退、左转）
                    	state_start_time = HAL_GetTick();
                    	break; // 触发打断
                    }

                    // 4. 完整的矩形盲跑序列
                    if (elapsed < t1) {
                    	Car_Backward();     // 阶段 1：后退，腾出转向空间
                    }
                    else if (elapsed < t2) {
                    	Car_TurnLeft();     // 阶段 2：左转 90 度，车头朝外
                    }
                    else if (elapsed < t3) {
                    	Car_Forward();      // 阶段 3：直行，拉开与障碍物的侧向距离
                    }
                    else if (elapsed < t4) {
                    	Car_TurnRight();    // 阶段 4：右转 90 度，车身再次与黑线平行
                    }
                    else if (elapsed < t5) {
                    	Car_Forward();      // 阶段 5：直行，平行越过障碍物
                    }
                    else if (elapsed < t6) {
                    	Car_TurnRight();    // 阶段 6：右转 90 度，车头垂直指向原本的黑线
                    }
                    else if (elapsed < t7) {
                    	Car_Forward();      //
                    }
                    else if (elapsed < t8) {
                    	Car_TurnRight();     //
                    }
                    else if (elapsed < t9) {
                    	Car_Forward();      //
                    }
                    else if (elapsed < t10) {
                    	Car_TurnRight();     //
                    }
                    else if (elapsed < t11) {
                    	Car_Forward();      //
                    }
                    else {
                    	Car_Stop(); // 可选：稍微停顿一下稳住底盘
                    	state = STATE_PATROL;
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

static VOID wifi_thread_entry(ULONG id)
{
    uint8_t init_status;
    char request[WIFI_HTTP_REQUEST_SIZE];

    /* CODEX 2026-07-24: Start UART4 interrupt reception before resetting ESP-01S. */
    ESP8266_Log_UART_Start();

    /* CODEX 2026-07-24: Use PQ4 to recover even when the AT firmware is unresponsive. */
    do
    {
        HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_4, GPIO_PIN_RESET);
        tx_thread_sleep(20);
        HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_4, GPIO_PIN_SET);
        /* CODEX 2026-07-24: Allow the AT firmware to finish booting before synchronization. */
        tx_thread_sleep(1000);
        ESP8266_Log_UART_Flush();

        init_status = ESP8266_Log_Server_Init();
        if (init_status != 0U)
        {
            printf("ESP-01S init failed at step %u\r\n", init_status);
        }
        tx_thread_sleep(1000);
    }
    while (init_status != 0U);
    printf("ESP-01S AP ready: Robot_Cam\r\n");

    while (1)
    {
        uint8_t link_id;
        uint32_t request_length;

        if (wifi_read_ipd_packet(&link_id, request, sizeof(request), &request_length) != 0U)
        {
            printf("ESP HTTP malformed +IPD packet\r\n");
            continue;
        }

        /* CODEX 2026-07-24: Route only after the complete declared HTTP payload is stored. */
        if (strstr(request, "GET /latest.bmp") != NULL)
        {
            static const char no_event[] =
                "No completed person event yet.\r\n";
            uint8_t image_available;

            printf("ESP HTTP GET /latest.bmp (%lu bytes)\r\n",
                   (unsigned long)request_length);

            tx_mutex_get(&snapshot_lock, TX_WAIT_FOREVER);
            image_available = (latest_event_id > 0U);
            if (image_available != 0U)
            {
                memcpy(wifi_tx_image, latest_event_image, SNAPSHOT_BMP_SIZE);
            }
            tx_mutex_put(&snapshot_lock);

            if (image_available != 0U)
            {
                (void)ESP8266_Log_Send_Response(link_id, "image/bmp",
                                                wifi_tx_image, SNAPSHOT_BMP_SIZE);
            }
            else
            {
                (void)ESP8266_Log_Send_Response(link_id, "text/plain; charset=utf-8",
                                                (const uint8_t *)no_event,
                                                (uint32_t)strlen(no_event));
            }
        }
        else if (strstr(request, "GET /favicon.ico ") != NULL)
        {
            /* CODEX 2026-07-24: A data favicon is used, but answer legacy browser requests. */
            printf("ESP HTTP GET /favicon.ico\r\n");
            (void)ESP8266_Log_Send_Response(link_id, "image/x-icon", NULL, 0U);
        }
        else if ((strstr(request, "GET / ") != NULL) ||
                 (strstr(request, "GET http://192.168.4.1/ ") != NULL) ||
                 (strstr(request, "GET /generate_204 ") != NULL) ||
                 (strstr(request, "GET /hotspot-detect.html ") != NULL) ||
                 (strstr(request, "GET /connecttest.txt ") != NULL) ||
                 (strstr(request, "GET /ncsi.txt ") != NULL))
        {
            uint32_t page_length;

            printf("ESP HTTP GET / (%lu bytes)\r\n", (unsigned long)request_length);
            page_length = wifi_build_event_page();
            if (page_length != 0U)
            {
                (void)ESP8266_Log_Send_Response(link_id, "text/html; charset=utf-8",
                                                wifi_page_buffer, page_length);
            }
            else
            {
                static const char page_error[] = "Page generation failed.\r\n";
                (void)ESP8266_Log_Send_Response(link_id, "text/plain; charset=utf-8",
                                                (const uint8_t *)page_error,
                                                (uint32_t)strlen(page_error));
            }
        }
        else
        {
            static const char not_found[] = "Not found.\r\n";

            printf("ESP HTTP unknown request: %.96s\r\n", request);
            (void)ESP8266_Log_Send_Response(link_id, "text/plain; charset=utf-8",
                                            (const uint8_t *)not_found,
                                            (uint32_t)strlen(not_found));
        }
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
    /* CODEX 2026-07-20: Protect the published event while ESP-01S is serving it. */
    tx_mutex_create(&snapshot_lock, "Snapshot Lock", TX_INHERIT);

    // 👇============= 新增：初始化AI数据锁 =============👇
    tx_mutex_create(&ai_data_lock, "AI Lock", TX_INHERIT);
    /* CODEX 2026-07-24: Start interrupt-driven RC-100B reception on USART1. */
    RC100_Init();
    // 👆===============================================👆

    camera_display_last_frame_tick = HAL_GetTick();
    camera_nn_last_frame_tick = camera_display_last_frame_tick;
    app_camera_display_pipe_start(app_lcd_get_bg_buffer(), CMW_MODE_CONTINUOUS);

    tx_thread_create(&nn_thread, "NN Thread", nn_thread_entry, 0, nn_thread_stack, sizeof(nn_thread_stack), TX_MAX_PRIORITIES - 3, TX_MAX_PRIORITIES - 3, 10, TX_AUTO_START);
    tx_thread_create(&pp_thread, "PP Thread", pp_thread_entry, 0, pp_thread_stack, sizeof(pp_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    tx_thread_create(&dp_thread, "DP Thread", dp_thread_entry, 0, dp_thread_stack, sizeof(dp_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    tx_thread_create(&isp_thread, "ISP Thread", isp_thread_entry, 0, isp_thread_stack, sizeof(isp_thread_stack), TX_MAX_PRIORITIES - 4, TX_MAX_PRIORITIES - 4, 10, TX_AUTO_START);

    // 👇============= 新增：创建并启动控制线程 =============👇
    // 优先级设为 TX_MAX_PRIORITIES - 2 (和 PP Thread 平级，确保响应迅速)
    tx_thread_create(&ctrl_thread, "Ctrl Thread", ctrl_thread_entry, 0, ctrl_thread_stack, sizeof(ctrl_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    tx_thread_create(&wifi_thread, "WiFi Thread", wifi_thread_entry, 0, wifi_thread_stack, sizeof(wifi_thread_stack), TX_MAX_PRIORITIES - 1, TX_MAX_PRIORITIES - 1, 10, TX_AUTO_START);
    // 👆==================================================👆
}

static void app_camera_display_pipe_vsync_cb(void)
{
    tx_semaphore_put(&isp_semaphore);
}

static void app_camera_display_pipe_frame_cb(void)
{
    camera_display_last_frame_tick = HAL_GetTick();
    app_lcd_switch_bg_buffer();
    app_camera_display_pipe_set_address(app_lcd_get_bg_buffer());
}

static void app_camera_nn_pipe_frame_cb(void)
{
    uint8_t *buffer;

    camera_nn_last_frame_tick = HAL_GetTick();
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

        /* CODEX 2026-07-20: Pair this inference result with its exact camera frame. */
        {
            uint8_t *snapshot = snapshot_for_output_buffer(output_buffer);
            if (snapshot != NULL)
            {
                snapshot_build_bmp(snapshot, capture_buffer);
            }
        }

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
        /* CODEX 2026-07-16: Pick an actual person target instead of assuming pOutBuff[0] is a person. */
        od_pp_outBuffer_t *person = NULL;
        float best_person_conf = 0.0f;
        for (i = 0; (pp_output.pOutBuff != NULL) && (i < pp_output.nb_detect); i++)
        {
            od_pp_outBuffer_t *candidate = &pp_output.pOutBuff[i];
            if ((candidate->class_index == PERSON_CLASS_INDEX) &&
                (candidate->conf >= AI_OBJDETECT_YOLOV2_PP_CONF_THRESHOLD) &&
                (candidate->conf > best_person_conf))
            {
                person = candidate;
                best_person_conf = candidate->conf;
            }
        }

        if (person != NULL)
        {
            // 获取检测到的第一个目标

            /* CODEX 2026-07-20: Retain only the highest-confidence frame in this event. */
            if ((raw_detect_confirm_cnt == 0U) && (person_event_active == 0U))
            {
                event_best_confidence = -1.0f;
            }
            if (best_person_conf > event_best_confidence)
            {
                uint8_t *current_snapshot = snapshot_for_output_buffer(output_buffer);
                if (current_snapshot != NULL)
                {
                    memcpy(event_best_image, current_snapshot, SNAPSHOT_BMP_SIZE);
                    event_best_confidence = best_person_conf;
                }
            }

            raw_loss_confirm_cnt = 0; // 既然看到了人，丢失计数器立刻清零

            // 1. 解决椅子瞬间误检：必须连续 4 帧都看到目标，才真正触发系统响应
            if (raw_detect_confirm_cnt < PERSON_CONFIRM_FRAMES) {
                raw_detect_confirm_cnt++;
            }

            // 只有当连续4帧都确认是人时，才允许修改控制变量
            if (raw_detect_confirm_cnt >= PERSON_CONFIRM_FRAMES)
            {
                person_event_active = 1U;
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
            if (person_event_active == 1U) {
                raw_loss_confirm_cnt++;
                if (raw_loss_confirm_cnt >= PERSON_LOSS_FRAMES) {
                    if (event_best_confidence >= AI_OBJDETECT_YOLOV2_PP_CONF_THRESHOLD)
                    {
                        snapshot_publish_event();
                    }
                    person_event_active = 0U;
                    event_best_confidence = -1.0f;
                    ai_person_detected = 0;
                    smooth_w = 0; // 目标彻底消失，平滑尺寸也要归零重置
                    smooth_h = 0;
                }
            }
            else
            {
                raw_loss_confirm_cnt = 0;
                event_best_confidence = -1.0f;
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

    /*
     * CODEX 2026-07-27: Reject malformed, non-person and degenerate boxes.
     * A zero-sized rectangle underflows inside UTIL_LCD_DrawRect() and can
     * briefly draw a long green line outside the intended detection box.
     */
    if ((detect == NULL) ||
        (detect->class_index != PERSON_CLASS_INDEX) ||
        (detect->conf < AI_OBJDETECT_YOLOV2_PP_CONF_THRESHOLD) ||
        !(detect->x_center >= 0.0f && detect->x_center <= 1.0f) ||
        !(detect->y_center >= 0.0f && detect->y_center <= 1.0f) ||
        !(detect->width > 0.0f && detect->width <= 1.0f) ||
        !(detect->height > 0.0f && detect->height <= 1.0f))
    {
        return;
    }

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

    if (((x1 - x0) < 2) || ((y1 - y0) < 2))
    {
        return;
    }

    UTIL_LCD_DrawRect(x0, y0, x1 - x0, y1 - y0, UTIL_LCD_COLOR_GREEN);
    UTIL_LCDEx_PrintfAt(x0, y0, LEFT_MODE, nn_classes_table[detect->class_index]);
}

static void app_display_network_output(app_display_info_t *display_info)
{
    float cpuload_one_second;
    rc100_state_t remote_state;
    uint32_t remote_color;
    const char *remote_status;
    uint8_t line_nb = 0;
    int32_t i;

    app_lcd_draw_area_update();

    UTIL_LCD_FillRect(0, 0, LCD_FG_WIDTH, LCD_FG_HEIGHT, 0x00000000);

    app_cpuload_update(&cpuload);
    app_cpuload_get_info(&cpuload, NULL, &cpuload_one_second, NULL);

    /*
     * CODEX 2026-07-24: On-screen RC-100 diagnostics replace unavailable
     * USART1 logging. Counts let field tests distinguish wiring from framing.
     */
    RC100_GetState(&remote_state);
    if (remote_state.byte_count == 0U)
    {
        remote_status = "RC NO DATA";
        remote_color = UTIL_LCD_COLOR_RED;
    }
    else if (remote_state.packet_count == 0U)
    {
        remote_status = "RC BAD FRAME";
        remote_color = UTIL_LCD_COLOR_YELLOW;
    }
    else if ((HAL_GetTick() - remote_state.last_packet_tick) >
             RC100_LINK_TIMEOUT_MS)
    {
        remote_status = "RC TIMEOUT";
        remote_color = UTIL_LCD_COLOR_RED;
    }
    else if (rc100_manual_mode_display != 0U)
    {
        remote_status = "RC MANUAL";
        remote_color = UTIL_LCD_COLOR_CYAN;
    }
    else
    {
        remote_status = "RC AUTO";
        remote_color = UTIL_LCD_COLOR_GREEN;
    }

    UTIL_LCD_SetTextColor(remote_color);
    UTIL_LCDEx_PrintfAt(8, LINE(0), LEFT_MODE,
                       "%s  B:%04X", remote_status, remote_state.buttons);
    UTIL_LCDEx_PrintfAt(8, LINE(1), LEFT_MODE,
                       "RX:%lu OK:%lu BAD:%lu E:%lu F:%lu",
                       (unsigned long)remote_state.byte_count,
                       (unsigned long)remote_state.packet_count,
                       (unsigned long)remote_state.invalid_frame_count,
                       (unsigned long)remote_state.uart_error_count,
                       (unsigned long)remote_state.receive_start_failure_count);
    /* CODEX 2026-07-26: Show successful camera-pipeline recoveries. */
    UTIL_LCDEx_PrintfAt(8, LINE(2), LEFT_MODE,
                       "CAM REC:%lu",
                       (unsigned long)camera_recovery_count);
    UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);

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
