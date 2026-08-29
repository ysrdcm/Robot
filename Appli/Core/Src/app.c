/**
 * @file app.c
 * @brief 巡防控制、AI 推理、画面显示与事件日志任务。
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
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "tracker.h"
#include "beep.h"
#include "car.h"
#include "wave.h"
#include "find.h"
#include "esp8266_log.h"
#include "rc100.h"

extern TIM_HandleTypeDef htim15;

static TX_MUTEX ai_data_lock;
static uint8_t  ai_person_detected = 0;
static float    ai_person_x = 0;
static float    ai_person_y = 0;
static float    ai_person_w = 0;
static float    ai_person_h = 0;

static uint16_t raw_detect_confirm_cnt = 0;
static uint16_t raw_loss_confirm_cnt = 0;
static float    smooth_w = 0;
static float    smooth_h = 0;

/* 控制与传感器时序参数。 */
#define PERSON_CLASS_INDEX                 0
#define ULTRASONIC_SAMPLE_PERIOD_MS        60U
#define RC100_LINK_TIMEOUT_MS              500U
#define MANUAL_OBSTACLE_STOP_CM            20.0f
#define CAMERA_SERVO_PWM_MIN               700.0f
#define CAMERA_SERVO_PWM_MAX               2300.0f
#define CAMERA_SERVO_UPDATE_PERIOD_MS      20U
#define CAMERA_SERVO_MANUAL_STEP           10.0f
#define CAMERA_SERVO_AUTO_DEADBAND          0.06f
#define CAMERA_SERVO_AUTO_GAIN             24.0f
#define CAMERA_SERVO_AUTO_MAX_STEP          7.5f
#define CAMERA_SERVO_SCAN_STEP              6.0f
#define SERVO_DETECTION_HOLD_MS             300U
/* DCMIPP 看门狗只在控制线程中执行恢复，避免在中断中调用阻塞式 HAL 接口。 */
#define CAMERA_FRAME_TIMEOUT_MS            2000U
#define CAMERA_RECOVERY_RETRY_MS           1500U

/* 推理帧为 RGB888，事件缩略图直接编码为无需额外编解码器的 BMP。 */
#define PERSON_CONFIRM_FRAMES              4U
#define PERSON_LOSS_FRAMES                 8U
#define SNAPSHOT_WIDTH                     (NN_WIDTH / 2U)
#define SNAPSHOT_HEIGHT                    (NN_HEIGHT / 2U)
#define SNAPSHOT_ROW_BYTES                 (((SNAPSHOT_WIDTH * 3U) + 3U) & ~3U)
#define SNAPSHOT_HEADER_BYTES              54U
#define SNAPSHOT_BMP_SIZE                  (SNAPSHOT_HEADER_BYTES + (SNAPSHOT_ROW_BYTES * SNAPSHOT_HEIGHT))
/* 网页和图片分开传输，避免 HTTP 工作缓冲区随图片增大。 */
#define WIFI_HTTP_REQUEST_SIZE             1024U
#define WIFI_PAGE_BUFFER_SIZE              4096U
#define WIFI_IPD_HEADER_SIZE               96U

typedef enum {
    STATE_PATROL,
    STATE_AVOID,
    STATE_WARNING,
    STATE_COMBAT
} RobotState_t;

/* 机器人控制线程。 */
static TX_THREAD ctrl_thread;
static UCHAR ctrl_thread_stack[2048];
static VOID ctrl_thread_entry(ULONG id);

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

static uint8_t app_is_valid_person_detection(const od_pp_outBuffer_t *detect)
{
    return (detect != NULL) &&
           (detect->class_index == PERSON_CLASS_INDEX) &&
           (detect->conf >= AI_OBJDETECT_YOLOV2_PP_CONF_THRESHOLD) &&
           (detect->x_center >= 0.0f) && (detect->x_center <= 1.0f) &&
           (detect->y_center >= 0.0f) && (detect->y_center <= 1.0f) &&
           (detect->width > 0.0f) && (detect->width <= 1.0f) &&
           (detect->height > 0.0f) && (detect->height <= 1.0f);
}

LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default);

static uint8_t nn_input_buffers[2][NN_WIDTH * NN_HEIGHT * NN_BPP] __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static app_bqueue_t nn_input_queue;
static uint8_t nn_output_buffers[2][ALIGN_VALUE(NN_BUFFER_OUT_SIZE, 32)] __attribute__((aligned(32)));
static app_bqueue_t nn_output_queue;
static const char *nn_classes_table[NN_CLASSES] = NN_CLASSES_TABLE;

static app_cpuload_t cpuload;

static TX_THREAD wifi_thread;
/* HTTP 格式化和 AT 解析需要比普通控制任务更大的栈。 */
static UCHAR wifi_thread_stack[4096];
static VOID wifi_thread_entry(ULONG id);

/* 每张缩略图与对应的推理输出队列槽一一配对。 */
static uint8_t nn_snapshot_buffers[2][SNAPSHOT_BMP_SIZE]
    __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static uint8_t event_best_image[SNAPSHOT_BMP_SIZE]
    __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static uint8_t latest_event_image[SNAPSHOT_BMP_SIZE]
    __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static uint8_t wifi_tx_image[SNAPSHOT_BMP_SIZE]
    __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
/* 大块 HTTP 和图像缓冲区放在外部 RAM，避免占用线程栈。 */
static uint8_t wifi_page_buffer[WIFI_PAGE_BUFFER_SIZE]
    __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static TX_MUTEX snapshot_lock;
static float event_best_confidence = -1.0f;
static float latest_event_confidence = 0.0f;
static uint32_t latest_event_tick = 0;
static uint32_t latest_event_id = 0;
static uint8_t person_event_active = 0;

/* 摄像头恢复后不得继续使用故障前的检测坐标或事件候选帧。 */
static void app_reset_person_detection(void)
{
    ai_person_detected = 0U;
    ai_person_x = 0.5f;
    ai_person_y = 0.5f;
    ai_person_w = 0.0f;
    ai_person_h = 0.0f;
    raw_detect_confirm_cnt = 0U;
    raw_loss_confirm_cnt = 0U;
    smooth_w = 0.0f;
    smooth_h = 0.0f;
    person_event_active = 0U;
    event_best_confidence = -1.0f;
}

/* 从实际送入 NPU 的 RGB888 帧生成浏览器可显示的缩略图。 */
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

/* 严格按照 +IPD 声明的长度读取一个完整请求。 */
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

    /* 图片独立请求可避免 Base64 膨胀，并让网页先完成排版。 */
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

static float camera_servo_pwm = 1500.0f;
static uint32_t camera_servo_last_update_tick;

static void Camera_Servo_Manual_Update(int8_t direction)
{
    uint32_t now = HAL_GetTick();

    /* 固定更新周期使舵机速度不受控制线程负载影响。 */
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

    static float filtered_target_x = 0.5f;
    static uint32_t last_tracking_tick;
    uint32_t now = HAL_GetTick();

    if (is_tracking)
    {
        last_tracking_tick = now;
    }

    /* 扫描与跟踪共用固定更新周期。 */
    if ((now - camera_servo_last_update_tick) <
        CAMERA_SERVO_UPDATE_PERIOD_MS)
    {
        return;
    }
    camera_servo_last_update_tick = now;

    if (is_tracking) {
        /* 忽略小幅检测抖动，再对有效位移进行低通滤波。 */
        if (fabsf(target_x - filtered_target_x) > 0.05f) {
            filtered_target_x = filtered_target_x * 0.90f + target_x * 0.10f;
        }

        float error = filtered_target_x - 0.5f;
        float step = 0.0f;

        /* 增量控制在人居中时保持当前角度，而不是强制回到 1500 us。 */
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
        /* 短暂丢检期间保持角度，超时后才恢复扫描。 */
        camera_servo_pwm += auto_dir * CAMERA_SERVO_SCAN_STEP;
        if (camera_servo_pwm > CAMERA_SERVO_PWM_MAX) {
            camera_servo_pwm = CAMERA_SERVO_PWM_MAX;
            auto_dir = -1;
        }
        if (camera_servo_pwm < CAMERA_SERVO_PWM_MIN) {
            camera_servo_pwm = CAMERA_SERVO_PWM_MIN;
            auto_dir = 1;
        }
        filtered_target_x = 0.5f;
    }
    /* 跟踪和扫描均限制在 SG90 的机械行程内。 */
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

/* 处理互斥方向键，并执行前向障碍紧急停车。 */
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
        /* 前方有障碍时禁止前进和转向，但仍允许后退。 */
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
    /* HC-SR04 两次触发之间需要静默间隔，因此复用最近一次有效距离。 */
    uint32_t last_ultrasonic_tick = 0;
    float    dist = -1.0f;
    uint32_t last_camera_recovery_attempt = 0U;

    (void)id;

    while(1)
    {
        uint32_t now = HAL_GetTick();

        if (((app_camera_recovery_requested() != 0U) ||
             ((now - camera_display_last_frame_tick) >
              CAMERA_FRAME_TIMEOUT_MS) ||
             ((now - camera_nn_last_frame_tick) >
              CAMERA_FRAME_TIMEOUT_MS)) &&
            ((now - last_camera_recovery_attempt) >=
             CAMERA_RECOVERY_RETRY_MS))
        {
            /* 在线程中恢复 DCMIPP，并先清除检测状态，防止旧坐标继续驱动执行器。 */
            last_camera_recovery_attempt = now;
            Car_Stop();
            Laser_Fire(0U);
            BEEP(0);

            tx_mutex_get(&ai_data_lock, TX_WAIT_FOREVER);
            app_reset_person_detection();
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

        /* 人工驾驶优先，二维云台仍使用 AI 坐标跟踪。 */
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
                /* 必须先收到按键释放帧，才允许下一次模式切换。 */
                mode_button_armed = 1U;
            }

            /* 将摄像头偏航叠加到图像误差，使云台目标转换到车体坐标系。 */
            Gimbal_Targeting_Update(p_x, p_y, p_det, camera_servo_pwm);

            if ((mode_button_armed != 0U) &&
                (manual_mode == 0U) &&
                ((remote_buttons & RC100_BUTTON_6) != 0U))
            {
                /* 遥控器上机械联动的 5+6 视为一次按键，释放前不重复切换。 */
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
                /* 恢复自动巡防前先关闭人工模式下的全部输出。 */
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

        switch(state)
        {
            case STATE_PATROL:
                Laser_Fire(0);
                BEEP(0);

                if (p_det) {
                    Car_Stop();
                    state = STATE_WARNING;
                    state_start_time = HAL_GetTick();
                    /* 保存初始目标框尺寸，用于判断人员是否靠近。 */
                    lock_width = p_w;
                    lock_height = p_h;
                }
                else if (dist > 0 && dist < 25.0f) {
                    Car_Stop();
                    state = STATE_AVOID;
                    state_start_time = HAL_GetTick();
                }
                else {
                	Track_Process();
                }
                break;

            case STATE_AVOID:
                /* 检测到人时立即中断定时避障流程。 */
                if (p_det) {
                    Car_Stop();
                    state = STATE_WARNING;
                    state_start_time = HAL_GetTick();
                    lock_width = p_w;
                    lock_height = p_h;
                } else {
                    uint32_t elapsed = HAL_GetTick() - state_start_time;

                    const uint32_t dt_back = 200;
                    const uint32_t dt_turn = 2120;
                    const uint32_t dt_fw_w = 1800;
                    const uint32_t dt_fw_l = 4450;

                    /* 各时间点是矩形绕障动作的累计边界。 */
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
                        break;
                    }

                    /* 绕障途中再次遇到近距离障碍时重新开始。 */
                    if (elapsed > t1 && dist > 0 && dist < 15.0f) {
                        Car_Stop();
                        state_start_time = HAL_GetTick();
                        break;
                    }

                    if (elapsed < t1) {
                        Car_Backward();
                    }
                    else if (elapsed < t2) {
                        Car_TurnLeft();
                    }
                    else if (elapsed < t3) {
                        Car_Forward();
                    }
                    else if (elapsed < t4) {
                        Car_TurnRight();
                    }
                    else if (elapsed < t5) {
                        Car_Forward();
                    }
                    else if (elapsed < t6) {
                        Car_TurnRight();
                    }
                    else if (elapsed < t7) {
                        Car_Forward();
                    }
                    else if (elapsed < t8) {
                        Car_TurnRight();
                    }
                    else if (elapsed < t9) {
                        Car_Forward();
                    }
                    else if (elapsed < t10) {
                        Car_TurnRight();
                    }
                    else if (elapsed < t11) {
                        Car_Forward();
                    }
                    else {
                        Car_Stop();
                        state = STATE_PATROL;
                    }
                }
                break;

            case STATE_WARNING:
                if (!p_det) {
                    BEEP(0);
                    Laser_Fire(0);
                    state = STATE_PATROL;
                } else {
                    uint32_t elapsed = HAL_GetTick() - state_start_time;

                    /* 以 300 ms 为周期鸣叫三次。 */
                    if (elapsed < 1800) {
                        uint32_t stage = elapsed / 300;
                        if (stage % 2 == 0) {
                            BEEP(1);
                        } else {
                            BEEP(0);
                        }

                        /* 目标框宽高均增大 15% 时判定人员正在靠近。 */
                        if ((p_w > lock_width * 1.15f) && (p_h > lock_height * 1.15f)) {
                            BEEP(0);
                            state = STATE_COMBAT;
                        }
                    } else {
                        BEEP(0);
                        state = STATE_COMBAT;
                    }
                }
                break;

            case STATE_COMBAT:
                if (!p_det) {
                    Laser_Fire(0);
                    state = STATE_PATROL;
                } else {
                    Laser_Fire(1);
                }
                break;
        }

        tx_thread_sleep(1);
    }
}

static VOID wifi_thread_entry(ULONG id)
{
    uint8_t init_status;
    char request[WIFI_HTTP_REQUEST_SIZE];

    (void)id;

    /* 复位前启动接收，以免遗漏 ESP8266 启动信息。 */
    ESP8266_Log_UART_Start();

    /* AT 固件无响应时仍可通过 PQ4 硬件复位恢复。 */
    do
    {
        HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_4, GPIO_PIN_RESET);
        tx_thread_sleep(20);
        HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_4, GPIO_PIN_SET);
        /* 等待 AT 固件启动完成后再同步。 */
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

        /* 完整接收声明长度的 HTTP 数据后再进行路由。 */
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
            /* 单独响应部分浏览器仍会发出的 favicon 请求。 */
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
static void app_display_network_output(app_display_info_t *display_info);

static void app_require_tx_success(UINT status)
{
    if (status != TX_SUCCESS)
    {
        Error_Handler();
    }
}

void app_run(void)
{
    app_lcd_init();

    app_require_tx_success(app_bqueue_init(
        &nn_input_queue, 2,
        (uint8_t *[2]){nn_input_buffers[0], nn_input_buffers[1]}));
    app_require_tx_success(app_bqueue_init(
        &nn_output_queue, 2,
        (uint8_t *[2]){nn_output_buffers[0], nn_output_buffers[1]}));
    app_cpuload_init(&cpuload);
    if (app_camera_init(app_camera_display_pipe_vsync_cb,
                        app_camera_display_pipe_frame_cb,
                        NULL,
                        app_camera_nn_pipe_frame_cb) != HAL_OK)
    {
        Error_Handler();
    }

    app_require_tx_success(tx_semaphore_create(&isp_semaphore, NULL, 0));
    app_require_tx_success(tx_semaphore_create(&display.update, NULL, 0));
    app_require_tx_success(tx_mutex_create(&display.lock, NULL, TX_INHERIT));
    /* Wi-Fi 发送期间用互斥量保护事件数据。 */
    app_require_tx_success(tx_mutex_create(
        &snapshot_lock, "Snapshot Lock", TX_INHERIT));

    app_require_tx_success(tx_mutex_create(
        &ai_data_lock, "AI Lock", TX_INHERIT));
    RC100_Init();

    camera_display_last_frame_tick = HAL_GetTick();
    camera_nn_last_frame_tick = camera_display_last_frame_tick;
    app_camera_display_pipe_start(app_lcd_get_bg_buffer(), CMW_MODE_CONTINUOUS);

    app_require_tx_success(tx_thread_create(&nn_thread, "NN Thread", nn_thread_entry, 0, nn_thread_stack, sizeof(nn_thread_stack), TX_MAX_PRIORITIES - 3, TX_MAX_PRIORITIES - 3, 10, TX_AUTO_START));
    app_require_tx_success(tx_thread_create(&pp_thread, "PP Thread", pp_thread_entry, 0, pp_thread_stack, sizeof(pp_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START));
    app_require_tx_success(tx_thread_create(&dp_thread, "DP Thread", dp_thread_entry, 0, dp_thread_stack, sizeof(dp_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START));
    app_require_tx_success(tx_thread_create(&isp_thread, "ISP Thread", isp_thread_entry, 0, isp_thread_stack, sizeof(isp_thread_stack), TX_MAX_PRIORITIES - 4, TX_MAX_PRIORITIES - 4, 10, TX_AUTO_START));

    /* 控制线程与后处理线程同优先级，兼顾执行器响应和检测吞吐。 */
    app_require_tx_success(tx_thread_create(&ctrl_thread, "Ctrl Thread", ctrl_thread_entry, 0, ctrl_thread_stack, sizeof(ctrl_thread_stack), TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START));
    app_require_tx_success(tx_thread_create(&wifi_thread, "WiFi Thread", wifi_thread_entry, 0, wifi_thread_stack, sizeof(wifi_thread_stack), TX_MAX_PRIORITIES - 1, TX_MAX_PRIORITIES - 1, 10, TX_AUTO_START));
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

    (void)id;

    nn_in_len = LL_Buffer_len(LL_ATON_Input_Buffers_Info_Default());
    nn_out_len = LL_Buffer_len(LL_ATON_Output_Buffers_Info_Default());

    nn_period[1] = HAL_GetTick();

    nn_pipe_dst = app_bqueue_get_free(&nn_input_queue, 0);
    if (nn_pipe_dst == NULL)
    {
        Error_Handler();
    }

    app_camera_nn_pipe_start(nn_pipe_dst, CMW_MODE_CONTINUOUS);

    while (1)
    {
        nn_period[0] = nn_period[1];
        nn_period[1] = HAL_GetTick();
        nn_period_ms = nn_period[1] - nn_period[0];

        capture_buffer = app_bqueue_get_ready(&nn_input_queue);
        output_buffer = app_bqueue_get_free(&nn_output_queue, 1);
        if ((capture_buffer == NULL) || (output_buffer == NULL))
        {
            Error_Handler();
        }

        time_stamp = HAL_GetTick();
        LL_ATON_Set_User_Input_Buffer_Default(0, capture_buffer, nn_in_len);
        SCB_InvalidateDCache_by_Addr(output_buffer, nn_out_len);
        LL_ATON_Set_User_Output_Buffer_Default(0, output_buffer, nn_out_len);
        LL_ATON_RT_Main(&NN_Instance_Default);
        inf_ms = HAL_GetTick() - time_stamp;

        /* 使用送入 NPU 的同一帧生成事件图片。 */
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
    int32_t detect_count;
    int32_t person_count;
    int32_t i;

    (void)id;

    app_postprocess_init(&pp_params);

    while (1)
    {
        output_buffer = app_bqueue_get_ready(&nn_output_queue);
        if (output_buffer == NULL)
        {
            Error_Handler();
        }
        pp_output.pOutBuff = NULL;

        nn_pp[0] = HAL_GetTick();
        app_postprocess_run((void *[]){(void *)output_buffer}, 1, &pp_output, &pp_params);
        nn_pp[1] = HAL_GetTick();

        detect_count = pp_output.nb_detect;
        if ((pp_output.pOutBuff == NULL) || (detect_count < 0))
        {
            detect_count = 0;
        }
        else if (detect_count > (int32_t)AI_OBJDETECT_YOLOV2_PP_MAX_BOXES_LIMIT)
        {
            detect_count = (int32_t)AI_OBJDETECT_YOLOV2_PP_MAX_BOXES_LIMIT;
        }

        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        person_count = 0;
        for (i = 0; i < detect_count; i++)
        {
            if (app_is_valid_person_detection(&pp_output.pOutBuff[i]) != 0U)
            {
                display.info.detects[person_count++] = pp_output.pOutBuff[i];
            }
        }
        display.info.nb_detect = person_count;
        display.info.pp_ms = nn_pp[1] - nn_pp[0];
        tx_mutex_put(&display.lock);


        /* 将检测状态作为一个整体发布给控制线程。 */
        tx_mutex_get(&ai_data_lock, TX_WAIT_FOREVER);

        /* 从全部检测框中选择置信度最高的人。 */
        od_pp_outBuffer_t *person = NULL;
        float best_person_conf = 0.0f;
        for (i = 0; i < detect_count; i++)
        {
            od_pp_outBuffer_t *candidate = &pp_output.pOutBuff[i];
            if ((app_is_valid_person_detection(candidate) != 0U) &&
                (candidate->conf > best_person_conf))
            {
                person = candidate;
                best_person_conf = candidate->conf;
            }
        }

        if (person != NULL)
        {
            /* 当前事件只保留置信度最高的一帧。 */
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

            raw_loss_confirm_cnt = 0;

            /* 连续多帧检出后才向控制状态机确认有人。 */
            if (raw_detect_confirm_cnt < PERSON_CONFIRM_FRAMES) {
                raw_detect_confirm_cnt++;
            }

            if (raw_detect_confirm_cnt >= PERSON_CONFIRM_FRAMES)
            {
                person_event_active = 1U;
                ai_person_detected = 1;
                ai_person_x = person->x_center;
                ai_person_y = person->y_center;

                /* 判断靠近前先平滑目标框尺寸。 */
                if (smooth_w == 0) {
                    smooth_w = person->width;
                    smooth_h = person->height;
                } else {
                    smooth_w = smooth_w * 0.85f + person->width * 0.15f;
                    smooth_h = smooth_h * 0.85f + person->height * 0.15f;
                }

                ai_person_w = smooth_w;
                ai_person_h = smooth_h;
            }
        }
        else
        {
            raw_detect_confirm_cnt = 0;

            /* 连续多帧丢检后才结束一次事件。 */
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
                    smooth_w = 0;
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

        app_bqueue_put_free(&nn_output_queue);
        tx_semaphore_ceiling_put(&display.update, 1);
    }
}

static VOID dp_thread_entry(ULONG id)
{
    uint32_t disp_ms = 0;
    app_display_info_t display_info;
    uint32_t time_stamp;

    (void)id;

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
    (void)id;

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
     * 过滤格式异常、类别不符和尺寸退化的检测框。零尺寸矩形会使
     * UTIL_LCD_DrawRect() 内部发生下溢，并在画面上产生异常长线。
     */
    if (app_is_valid_person_detection(detect) == 0U)
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

    /* USART1 被蓝牙占用时用屏幕计数区分接线故障和数据帧错误。 */
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
    if (display_info->nn_period_ms != 0U)
    {
        UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "%.2f",
                           1000.0 / display_info->nn_period_ms);
    }
    else
    {
        UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "--");
    }
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
