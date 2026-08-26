/**
 ****************************************************************************************************
 * @file        app_camera.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app_camera.c文件
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
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

static void app_camera_display_pipe_init(uint32_t sensor_width, uint32_t sensor_height);
static void app_camera_nn_pipe_init(uint32_t sensor_width, uint32_t sensor_height);
static void app_camera_init_crop_config(CMW_Manual_roi_area_t *roi, uint32_t sensor_width, uint32_t sensor_height);

static void (*app_camera_display_pipe_vsync_user_cb)(void) = NULL;
static void (*app_camera_display_pipe_frame_user_cb)(void) = NULL;
static void (*app_camera_nn_pipe_vsync_user_cb)(void) = NULL;
static void (*app_camera_nn_pipe_frame_user_cb)(void) = NULL;
/* Active buffers and modes are retained for thread-context recovery. */
static volatile uint8_t app_camera_recover_request;
static uint8_t *app_camera_display_destination;
static uint8_t *app_camera_nn_destination;
static uint32_t app_camera_display_capture_mode;
static uint32_t app_camera_nn_capture_mode;

void app_camera_init(void (*display_pipe_vsync_cb)(void), void (*display_pipe_frame_cb)(void), void (*nn_pipe_vsync_cb)(void), void (*nn_pipe_frame_cb)(void))
{
    DCMIPP_ParallelConfTypeDef pParallelConfig = {0};

    extern HAL_StatusTypeDef MX_DCMIPP_ClockConfig(DCMIPP_HandleTypeDef *hdcmipp);
    MX_DCMIPP_ClockConfig(&hdcmipp);

    hdcmipp.Instance = DCMIPP;
    HAL_DCMIPP_Init(&hdcmipp);

    /* OV5640 uses the STM32N6 eight-bit parallel DCMIPP interface. */
    pParallelConfig.PCKPolarity = DCMIPP_PCKPOLARITY_RISING ;
    pParallelConfig.HSPolarity = DCMIPP_HSPOLARITY_LOW ;
    pParallelConfig.VSPolarity = DCMIPP_VSPOLARITY_LOW ;
    pParallelConfig.ExtendedDataMode = DCMIPP_INTERFACE_8BITS;
    pParallelConfig.Format = DCMIPP_FORMAT_RGB565;
    pParallelConfig.SwapBits = DCMIPP_SWAPBITS_DISABLE;
    pParallelConfig.SwapCycles = DCMIPP_SWAPCYCLES_ENABLE;
    pParallelConfig.SynchroMode = DCMIPP_SYNCHRO_HARDWARE;

    HAL_DCMIPP_PARALLEL_SetConfig(&hdcmipp, &pParallelConfig);

    while (ov5640_init()) {
        HAL_Delay(500);
    }

    ov5640_rgb565_mode();
    ov5640_focus_init();
    ov5640_light_mode(0);
    ov5640_color_saturation(3);
    ov5640_brightness(4);
    ov5640_contrast(3);
    ov5640_sharpness(33);
    ov5640_focus_constant();
    ov5640_outsize_set(4, 0, LCD_BG_WIDTH, LCD_BG_HEIGHT);

    app_camera_display_pipe_init(LCD_BG_WIDTH, LCD_BG_HEIGHT);
    app_camera_nn_pipe_init(LCD_BG_WIDTH, LCD_BG_HEIGHT);

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

    /*
     * Stop and restart outside the ISR. HAL marks an overrun
     * pipe ERROR and disables its interrupt, so it cannot self-recover.
     */
    display_status = HAL_DCMIPP_PIPE_Stop(&hdcmipp, DCMIPP_PIPE1);
    nn_status = HAL_DCMIPP_PIPE_Stop(&hdcmipp, DCMIPP_PIPE2);

    if ((display_status == HAL_OK) && (nn_status == HAL_OK))
    {
        hdcmipp.ErrorCode = HAL_DCMIPP_ERROR_NONE;
        hdcmipp.State = HAL_DCMIPP_STATE_READY;
        __HAL_DCMIPP_CLEAR_FLAG(
            &hdcmipp,
            DCMIPP_FLAG_AXI_TRANSFER_ERROR |
            DCMIPP_FLAG_PARALLEL_SYNC_ERROR |
            DCMIPP_FLAG_PIPE1_OVR |
            DCMIPP_FLAG_PIPE2_OVR);

        display_status = HAL_DCMIPP_PIPE_Start(
            &hdcmipp, DCMIPP_PIPE1,
            (uint32_t)display_destination,
            display_capture_mode);
        nn_status = HAL_DCMIPP_PIPE_Start(
            &hdcmipp, DCMIPP_PIPE2,
            (uint32_t)nn_destination,
            nn_capture_mode);
        /* HAL disables these interrupt sources after a global error. */
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
    // 置空即可，OV5640 自带硬件 ISP
}

static void app_camera_display_pipe_init(uint32_t sensor_width, uint32_t sensor_height)
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
    CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &cmw_dcmipp_conf, &hw_pitch);
}

static void app_camera_nn_pipe_init(uint32_t sensor_width, uint32_t sensor_height)
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
    CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE2, &cmw_dcmipp_conf, &hw_pitch);
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
        /* Blocking recovery is deferred to the control thread. */
        app_camera_recover_request = 1U;
    }
}

void HAL_DCMIPP_ErrorCallback(DCMIPP_HandleTypeDef *hdcmipp)
{
    if (hdcmipp->Instance == DCMIPP)
    {
        /* Covers parallel-sync and AXI transfer errors. */
        app_camera_recover_request = 1U;
    }
}
