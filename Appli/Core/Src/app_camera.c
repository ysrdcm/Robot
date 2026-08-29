/**
 * @file app_camera.c
 * @brief OV5640 与 DCMIPP 显示、推理管线管理。
 */

#include "app_camera.h"
#include "app_config.h"
#include "app_utils.h"
#include "app.h"
#include "stm32n6xx_hal.h"
#include "cmw_camera.h"
#include "ov5640.h"

extern DCMIPP_HandleTypeDef hcamera_dcmipp;
#define hdcmipp hcamera_dcmipp

static HAL_StatusTypeDef app_camera_display_pipe_init(uint32_t sensor_width, uint32_t sensor_height);
static HAL_StatusTypeDef app_camera_nn_pipe_init(uint32_t sensor_width, uint32_t sensor_height);
static HAL_StatusTypeDef app_camera_dcmipp_init(void);
static void app_camera_init_crop_config(CMW_Manual_roi_area_t *roi, uint32_t sensor_width, uint32_t sensor_height);

#define APP_CAMERA_SENSOR_INIT_RETRIES 3U

static void (*app_camera_display_pipe_vsync_user_cb)(void) = NULL;
static void (*app_camera_display_pipe_frame_user_cb)(void) = NULL;
static void (*app_camera_nn_pipe_vsync_user_cb)(void) = NULL;
static void (*app_camera_nn_pipe_frame_user_cb)(void) = NULL;
/* 保存当前缓冲区和采集模式，供控制线程恢复管线。 */
static volatile uint8_t app_camera_recover_request;
static uint8_t * volatile app_camera_display_destination;
static uint8_t * volatile app_camera_nn_destination;
static uint32_t app_camera_display_capture_mode;
static uint32_t app_camera_nn_capture_mode;

HAL_StatusTypeDef app_camera_init(void (*display_pipe_vsync_cb)(void), void (*display_pipe_frame_cb)(void), void (*nn_pipe_vsync_cb)(void), void (*nn_pipe_frame_cb)(void))
{
    uint32_t attempt;

    if (app_camera_dcmipp_init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (attempt = 0U; attempt < APP_CAMERA_SENSOR_INIT_RETRIES; attempt++)
    {
        if (ov5640_init() == 0U)
        {
            break;
        }
        HAL_Delay(500U);
    }
    if (attempt == APP_CAMERA_SENSOR_INIT_RETRIES)
    {
        return HAL_ERROR;
    }

    ov5640_rgb565_mode();
    ov5640_focus_init();
    ov5640_light_mode(0);
    ov5640_color_saturation(3);
    ov5640_brightness(4);
    ov5640_contrast(3);
    ov5640_sharpness(33);
    ov5640_focus_constant();
    if (ov5640_outsize_set(4, 0, LCD_BG_WIDTH, LCD_BG_HEIGHT) != 0U)
    {
        return HAL_ERROR;
    }

    if ((app_camera_display_pipe_init(LCD_BG_WIDTH, LCD_BG_HEIGHT) != HAL_OK) ||
        (app_camera_nn_pipe_init(LCD_BG_WIDTH, LCD_BG_HEIGHT) != HAL_OK))
    {
        return HAL_ERROR;
    }

    if (display_pipe_vsync_cb != NULL)
    {
        app_camera_display_pipe_vsync_user_cb = display_pipe_vsync_cb;
    }

    if (display_pipe_frame_cb != NULL)
    {
        app_camera_display_pipe_frame_user_cb = display_pipe_frame_cb;
    }

    if (nn_pipe_vsync_cb != NULL)
    {
        app_camera_nn_pipe_vsync_user_cb = nn_pipe_vsync_cb;
    }

    if (nn_pipe_frame_cb != NULL)
    {
        app_camera_nn_pipe_frame_user_cb = nn_pipe_frame_cb;
    }

    return HAL_OK;
}

void app_camera_display_pipe_start(uint8_t *display_pipe_destination, uint32_t capture_mode)
{
    app_camera_display_destination = display_pipe_destination;
    app_camera_display_capture_mode = capture_mode;
    if (HAL_DCMIPP_PIPE_Start(&hdcmipp, DCMIPP_PIPE1,
                              (uint32_t)display_pipe_destination,
                              capture_mode) != HAL_OK)
    {
        app_camera_recover_request = 1U;
    }
}

void app_camera_nn_pipe_start(uint8_t *nn_pipe_destination, uint32_t capture_mode)
{
    app_camera_nn_destination = nn_pipe_destination;
    app_camera_nn_capture_mode = capture_mode;
    if (HAL_DCMIPP_PIPE_Start(&hdcmipp, DCMIPP_PIPE2,
                              (uint32_t)nn_pipe_destination,
                              capture_mode) != HAL_OK)
    {
        app_camera_recover_request = 1U;
    }
}

void app_camera_display_pipe_set_address(uint8_t *display_pipe_destination)
{
    app_camera_display_destination = display_pipe_destination;
    if (HAL_DCMIPP_PIPE_SetMemoryAddress(&hdcmipp, DCMIPP_PIPE1,
                                         DCMIPP_MEMORY_ADDRESS_0,
                                         (uint32_t)display_pipe_destination) != HAL_OK)
    {
        app_camera_recover_request = 1U;
    }
}

void app_camera_nn_pipe_set_address(uint8_t *nn_pipe_destination)
{
    app_camera_nn_destination = nn_pipe_destination;
    if (HAL_DCMIPP_PIPE_SetMemoryAddress(&hdcmipp, DCMIPP_PIPE2,
                                         DCMIPP_MEMORY_ADDRESS_0,
                                         (uint32_t)nn_pipe_destination) != HAL_OK)
    {
        app_camera_recover_request = 1U;
    }
}

uint8_t app_camera_recovery_requested(void)
{
    return app_camera_recover_request;
}

HAL_StatusTypeDef app_camera_recover(void)
{
    HAL_StatusTypeDef display_status;
    HAL_StatusTypeDef nn_status;
    uint8_t *display_destination;
    uint8_t *nn_destination;
    uint32_t display_capture_mode;
    uint32_t nn_capture_mode;

    HAL_NVIC_DisableIRQ(DCMIPP_IRQn);
    __DMB();
    display_destination = app_camera_display_destination;
    nn_destination = app_camera_nn_destination;
    display_capture_mode = app_camera_display_capture_mode;
    nn_capture_mode = app_camera_nn_capture_mode;

    if ((display_destination == NULL) || (nn_destination == NULL))
    {
        HAL_NVIC_EnableIRQ(DCMIPP_IRQn);
        return HAL_ERROR;
    }

    /* HAL 在过载后会把管线置为 ERROR，必须在线程中停止并重新启动。 */
    display_status = HAL_DCMIPP_PIPE_Stop(&hdcmipp, DCMIPP_PIPE1);
    nn_status = HAL_DCMIPP_PIPE_Stop(&hdcmipp, DCMIPP_PIPE2);

    if ((display_status != HAL_OK) || (nn_status != HAL_OK))
    {
        /*
         * 管线停止超时后，单纯重试无法清除 HAL 的错误状态。
         * 复位 DCMIPP 外设并恢复配置，摄像头寄存器无需重新初始化。
         */
        __HAL_RCC_DCMIPP_FORCE_RESET();
        __DSB();
        __HAL_RCC_DCMIPP_RELEASE_RESET();
        hdcmipp.State = HAL_DCMIPP_STATE_RESET;

        display_status = app_camera_dcmipp_init();
        HAL_NVIC_DisableIRQ(DCMIPP_IRQn);
        if (display_status == HAL_OK)
        {
            display_status = app_camera_display_pipe_init(
                LCD_BG_WIDTH, LCD_BG_HEIGHT);
            nn_status = app_camera_nn_pipe_init(
                LCD_BG_WIDTH, LCD_BG_HEIGHT);
        }
        else
        {
            nn_status = HAL_ERROR;
        }
    }

    if ((display_status == HAL_OK) && (nn_status == HAL_OK))
    {
        hdcmipp.ErrorCode = HAL_DCMIPP_ERROR_NONE;
        hdcmipp.State = HAL_DCMIPP_STATE_READY;
        __HAL_DCMIPP_CLEAR_FLAG(
            &hdcmipp,
            DCMIPP_FLAG_AXI_TRANSFER_ERROR |
            DCMIPP_FLAG_PARALLEL_SYNC_ERROR |
            DCMIPP_FLAG_PIPE1_FRAME |
            DCMIPP_FLAG_PIPE1_VSYNC |
            DCMIPP_FLAG_PIPE1_OVR |
            DCMIPP_FLAG_PIPE2_FRAME |
            DCMIPP_FLAG_PIPE2_VSYNC |
            DCMIPP_FLAG_PIPE2_OVR);

        display_status = HAL_DCMIPP_PIPE_Start(
            &hdcmipp, DCMIPP_PIPE1,
            (uint32_t)display_destination,
            display_capture_mode);
        nn_status = HAL_DCMIPP_PIPE_Start(
            &hdcmipp, DCMIPP_PIPE2,
            (uint32_t)nn_destination,
            nn_capture_mode);
        /* HAL 处理全局错误时会关闭这些中断源。 */
        __HAL_DCMIPP_ENABLE_IT(
            &hdcmipp,
            DCMIPP_IT_AXI_TRANSFER_ERROR |
            DCMIPP_IT_PARALLEL_SYNC_ERROR);
    }

    if ((display_status == HAL_OK) && (nn_status == HAL_OK))
    {
        app_camera_recover_request = 0U;
    }
    __DMB();
    HAL_NVIC_EnableIRQ(DCMIPP_IRQn);

    return ((display_status == HAL_OK) && (nn_status == HAL_OK)) ?
           HAL_OK : HAL_ERROR;
}

void app_camera_isp_update(void)
{
    /* OV5640 使用片内 ISP，无需逐帧更新 STM32 ISP 参数。 */
}

static HAL_StatusTypeDef app_camera_dcmipp_init(void)
{
    DCMIPP_ParallelConfTypeDef parallel_config = {0};
    extern HAL_StatusTypeDef MX_DCMIPP_ClockConfig(
        DCMIPP_HandleTypeDef *hdcmipp);

    if (MX_DCMIPP_ClockConfig(&hdcmipp) != HAL_OK)
    {
        return HAL_ERROR;
    }

    hdcmipp.Instance = DCMIPP;
    if (HAL_DCMIPP_Init(&hdcmipp) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* OV5640 通过 STM32N6 的 8 位并行接口输出 RGB565。 */
    parallel_config.PCKPolarity = DCMIPP_PCKPOLARITY_RISING;
    parallel_config.HSPolarity = DCMIPP_HSPOLARITY_LOW;
    parallel_config.VSPolarity = DCMIPP_VSPOLARITY_LOW;
    parallel_config.ExtendedDataMode = DCMIPP_INTERFACE_8BITS;
    parallel_config.Format = DCMIPP_FORMAT_RGB565;
    parallel_config.SwapBits = DCMIPP_SWAPBITS_DISABLE;
    parallel_config.SwapCycles = DCMIPP_SWAPCYCLES_ENABLE;
    parallel_config.SynchroMode = DCMIPP_SYNCHRO_HARDWARE;

    return HAL_DCMIPP_PARALLEL_SetConfig(&hdcmipp, &parallel_config);
}

static HAL_StatusTypeDef app_camera_display_pipe_init(uint32_t sensor_width, uint32_t sensor_height)
{
    CMW_DCMIPP_Conf_t cmw_dcmipp_conf = {0};
    uint32_t hw_pitch;

    cmw_dcmipp_conf.output_width = LCD_BG_WIDTH;
    cmw_dcmipp_conf.output_height = LCD_BG_HEIGHT;
    cmw_dcmipp_conf.output_format = DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1;
    cmw_dcmipp_conf.output_bpp = 2;
    cmw_dcmipp_conf.enable_swap = 0;
    cmw_dcmipp_conf.enable_gamma_conversion = 0;
    cmw_dcmipp_conf.mode = CMW_Aspect_ratio_manual_roi;
    app_camera_init_crop_config(&cmw_dcmipp_conf.manual_conf, sensor_width, sensor_height);
    return (CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &cmw_dcmipp_conf,
                                     &hw_pitch) == CMW_ERROR_NONE) ?
           HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef app_camera_nn_pipe_init(uint32_t sensor_width, uint32_t sensor_height)
{
    CMW_DCMIPP_Conf_t cmw_dcmipp_conf = {0};
    uint32_t hw_pitch;

    cmw_dcmipp_conf.output_width = NN_WIDTH;
    cmw_dcmipp_conf.output_height = NN_HEIGHT;
    cmw_dcmipp_conf.output_format = NN_FORMAT;
    cmw_dcmipp_conf.output_bpp = NN_BPP;
    cmw_dcmipp_conf.enable_swap = 1;
    cmw_dcmipp_conf.enable_gamma_conversion = 0;
    cmw_dcmipp_conf.mode = CMW_Aspect_ratio_manual_roi;
    app_camera_init_crop_config(&cmw_dcmipp_conf.manual_conf, sensor_width, sensor_height);
    return (CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE2, &cmw_dcmipp_conf,
                                     &hw_pitch) == CMW_ERROR_NONE) ?
           HAL_OK : HAL_ERROR;
}

static void app_camera_init_crop_config(CMW_Manual_roi_area_t *roi, uint32_t sensor_width, uint32_t sensor_height)
{
    float ratiox;
    float ratioy;
    float ratio;

    ratiox = (float)sensor_width / LCD_BG_WIDTH;
    ratioy = (float)sensor_height / LCD_BG_HEIGHT;
    ratio = MIN(ratiox, ratioy);

    roi->width = (uint32_t)MIN(LCD_BG_WIDTH * ratio, sensor_width);
    roi->height = (uint32_t)MIN(LCD_BG_HEIGHT * ratio, sensor_height);
    roi->offset_x = (sensor_width - roi->width + 1) / 2;
    roi->offset_y = (sensor_height - roi->height + 1) / 2;
}

HAL_StatusTypeDef MX_DCMIPP_ClockConfig(DCMIPP_HandleTypeDef *hdcmipp)
{
    RCC_PeriphCLKInitTypeDef rcc_periph_clk_init_struct = {0};

    (void)hdcmipp;

    rcc_periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_DCMIPP | RCC_PERIPHCLK_CSI;
    rcc_periph_clk_init_struct.DcmippClockSelection = RCC_DCMIPPCLKSOURCE_IC17;
    rcc_periph_clk_init_struct.ICSelection[RCC_IC17].ClockSelection = RCC_ICCLKSOURCE_PLL2;
    rcc_periph_clk_init_struct.ICSelection[RCC_IC17].ClockDivider = 3;
    rcc_periph_clk_init_struct.ICSelection[RCC_IC18].ClockSelection = RCC_ICCLKSOURCE_PLL1;
    rcc_periph_clk_init_struct.ICSelection[RCC_IC18].ClockDivider = 40;
    if (HAL_RCCEx_PeriphCLKConfig(&rcc_periph_clk_init_struct) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void HAL_DCMIPP_PIPE_VsyncEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
    if (hdcmipp->Instance == DCMIPP)
    {
        if (Pipe == DCMIPP_PIPE1)
        {
            if (app_camera_display_pipe_vsync_user_cb != NULL)
            {
                app_camera_display_pipe_vsync_user_cb();
            }
        }
        else if (Pipe == DCMIPP_PIPE2)
        {
            if (app_camera_nn_pipe_vsync_user_cb != NULL)
            {
                app_camera_nn_pipe_vsync_user_cb();
            }
        }
    }
}

void HAL_DCMIPP_PIPE_FrameEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
    if (hdcmipp->Instance == DCMIPP)
    {
        if (Pipe == DCMIPP_PIPE1)
        {
            if (app_camera_display_pipe_frame_user_cb != NULL)
            {
                app_camera_display_pipe_frame_user_cb();
            }
        }
        else if (Pipe == DCMIPP_PIPE2)
        {
            if (app_camera_nn_pipe_frame_user_cb != NULL)
            {
                app_camera_nn_pipe_frame_user_cb();
            }
        }
    }
}

void HAL_DCMIPP_PIPE_ErrorCallback(DCMIPP_HandleTypeDef *hdcmipp,
                                   uint32_t Pipe)
{
    (void)Pipe;
    if (hdcmipp->Instance == DCMIPP)
    {
        /* 阻塞式恢复交给控制线程执行。 */
        app_camera_recover_request = 1U;
    }
}

void HAL_DCMIPP_ErrorCallback(DCMIPP_HandleTypeDef *hdcmipp)
{
    if (hdcmipp->Instance == DCMIPP)
    {
        /* 覆盖并行同步错误和 AXI 传输错误。 */
        app_camera_recover_request = 1U;
    }
}
