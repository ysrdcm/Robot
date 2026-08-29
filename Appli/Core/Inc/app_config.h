#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "stm32n6xx_hal.h"
#include "postprocess_conf.h"

#define LCD_BG_WIDTH                            800
#define LCD_BG_HEIGHT                           480
#define LCD_FG_WIDTH                            LCD_BG_WIDTH
#define LCD_FG_HEIGHT                           LCD_BG_HEIGHT

#define DISPLAY_DELAY                           1
#define DISPLAY_BUFFER_NB                       (DISPLAY_DELAY + 2)

#define CAMERA_MIRROR_FLIP                      CMW_MIRRORFLIP_MIRROR

#define CPU_LOAD_HISTORY_DEPTH                  8

#define BQUEUE_MAX_BUFFERS                      2

#define NN_WIDTH                                224
#define NN_HEIGHT                               224
#define NN_FORMAT                               DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1
#define NN_BPP                                  3
#define NN_BUFFER_OUT_SIZE                      5880
#define NN_CLASSES                              2
#define NN_CLASSES_TABLE                        {"person", "not_person"}

#define POSTPROCESS_TYPE                        POSTPROCESS_OD_YOLO_V2_UF
#define AI_OBJDETECT_YOLOV2_PP_CONF_THRESHOLD   0.80f
#define AI_OBJDETECT_YOLOV2_PP_IOU_THRESHOLD    0.3f
#define AI_OBJDETECT_YOLOV2_PP_MAX_BOXES_LIMIT  10

#endif
