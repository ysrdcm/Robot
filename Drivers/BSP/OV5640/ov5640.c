/**
 ****************************************************************************************************
 * @file        ov5640.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       OV5640 驱动代码
 * @license     Copyright (c) 2023-2035, 广州市星翼电子科技有限公司
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

#include "ov5640.h"
#include "sccb.h"
#include "ov5640cfg.h"
#include "ov5640af.h"

/**
 * @brief   读OV5640寄存器
 * @param   reg: 寄存器地址
 * @retval  寄存器值
 */
uint8_t ov5640_read_reg(uint16_t reg)
{
    uint8_t data = 0;
    
    sccb_start();
    sccb_send_byte((OV5640_ADDR << 1) | 0x00);
    sccb_send_byte(reg >> 8);
    sccb_send_byte(reg);
    sccb_stop();
    
    sccb_start();
    sccb_send_byte((OV5640_ADDR << 1) | 0x01);
    data = sccb_read_byte();
    sccb_nack();
    sccb_stop();
    
    return data;
}

/**
 * @brief   写OV5640寄存器
 * @param   reg: 寄存器地址
 * @param   data: 寄存器值
 * @retval  无
 */
void ov5640_write_reg(uint16_t reg, uint8_t data)
{
    sccb_start();
    sccb_send_byte((OV5640_ADDR << 1) | 0x00);
    sccb_send_byte(reg >> 8);
    sccb_send_byte(reg);
    sccb_send_byte(data);
    sccb_stop();
}

/**
 * @brief   初始化OV5640
 * @param   无
 * @retval  初始化结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t ov5640_init(void)
{
    uint16_t id;
    uint16_t i;
    
    /* 硬件复位 */
    OV5640_RST(0);
    HAL_Delay(20);
    OV5640_PWDN(0);
    HAL_Delay(5);
    OV5640_RST(1);
    HAL_Delay(20);
    
    /* 初始化SCCB */
    sccb_init();
    
    /* 校验ID */
    id = ov5640_read_reg(0x300A) << 8;
    id |= ov5640_read_reg(0x300B);
    if (id != OV5640_ID)
    {
        return 1;
    }
    
    /* 软件复位 */
    ov5640_write_reg(0x3103, 0x11);
    ov5640_write_reg(0x3008, 0x82);
    HAL_Delay(10);
    
    /* 初始化OV5640寄存器 */
    for (i=0; i<(sizeof(ov5640_init_reg_tbl) / sizeof(ov5640_init_reg_tbl[0])); i++)
    {
        ov5640_write_reg(ov5640_init_reg_tbl[i][0], ov5640_init_reg_tbl[i][1]);
    }
    
    /* 检查闪光灯 */
    ov5640_flash_ctrl(1);
    HAL_Delay(50);
    ov5640_flash_ctrl(0);
    
    return 0;
}

/**
 * @brief   控制OV5640闪光灯
 * @param   sw: 闪光灯状态
 * @param   0: 关闭
 * @param   1: 开启
 * @retval  无
 */
void ov5640_flash_ctrl(uint8_t sw)
{
    ov5640_write_reg(0x3016, 0x02);
    ov5640_write_reg(0x301C, 0x02);

    if (sw != 0)
    {
        ov5640_write_reg(0x3019, 0x02);
    }
    else
    {
        ov5640_write_reg(0x3019, 0x00);
    }
}

/**
 * @brief   配置OV5640 RGB565输出
 * @param   无
 * @retval  无
 */
void ov5640_rgb565_mode(void)
{
    uint16_t i;
    
    for (i=0; i<(sizeof(ov5640_rgb565_reg_tbl) / sizeof(ov5640_rgb565_reg_tbl[0])); i++)
    {
        ov5640_write_reg(ov5640_rgb565_reg_tbl[i][0], ov5640_rgb565_reg_tbl[i][1]);
    }
}

/**
 * @brief   配置OV5640 JPEG输出
 * @param   无
 * @retval  无
 */
void ov5640_jpeg_mode(void)
{
    uint16_t i;
    
    for (i=0; i<(sizeof(ov5640_jpeg_reg_tbl) / sizeof(ov5640_jpeg_reg_tbl[0])); i++)
    {
        ov5640_write_reg(ov5640_jpeg_reg_tbl[i][0], ov5640_jpeg_reg_tbl[i][1]);
    }
}

/* OV5640灯光配置数组 */
static const uint8_t ov5640_lightmode_tbl[5][7] =
{
    {0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00}, /* Auto */
    {0x06, 0x1C, 0x04, 0x00, 0x04, 0xF3, 0x01}, /* Sunny */
    {0x05, 0x48, 0x04, 0x00, 0x07, 0xCF, 0x01}, /* Office */
    {0x06, 0x48, 0x04, 0x00, 0x04, 0xD3, 0x01}, /* Cloudy */
    {0x04, 0x10, 0x04, 0x00, 0x08, 0x40, 0x01}, /* Home */
};

/**
 * @brief   设置OV5640灯光模式
 * @param   mode: 灯光模式
 * @arg     0: Auto
 * @arg     1: Sunny
 * @arg     2: Office
 * @arg     3: Cloudy
 * @arg     4: Home
 * @retval  无
 */
void ov5640_light_mode(uint8_t mode)
{
    uint8_t i;
    
    ov5640_write_reg(0x3212, 0x03);
    for (i=0; i<(sizeof(ov5640_lightmode_tbl) / sizeof(ov5640_lightmode_tbl[0])); i++)
    {
        ov5640_write_reg(0x3400 + i, ov5640_lightmode_tbl[mode][i]);
    }
    ov5640_write_reg(0x3212, 0x13);
    ov5640_write_reg(0x3212, 0xA3);
}

/* OV5640曝光配置数组 */
static const uint8_t ov5640_exposure_tbl[7][6] =
{
    {0x10, 0x08, 0x10, 0x08, 0x20, 0x10},   /* -3 */
    {0x20, 0x18, 0x41, 0x20, 0x18, 0x10},   /* -2 */
    {0x30, 0x28, 0x61, 0x30, 0x28, 0x10},   /* -1 */
    {0x38, 0x30, 0x61, 0x38, 0x30, 0x10},   /* 0 */
    {0x40, 0x38, 0x71, 0x40, 0x38, 0x10},   /* +1 */
    {0x50, 0x48, 0x90, 0x50, 0x48, 0x20},   /* +2 */
    {0x60, 0x58, 0xa0, 0x60, 0x58, 0x20},   /* +3 */
};

/**
 * @brief   设置OV5640曝光等级
 * @param   level: 曝光等级
 * @arg     0~6: -3~+3
 * @retval  无
 */
void ov5640_exposure(uint8_t level)
{
    ov5640_write_reg(0x3212, 0x03);
    ov5640_write_reg(0x3A0F, ov5640_exposure_tbl[level][0]);
    ov5640_write_reg(0x3A10, ov5640_exposure_tbl[level][1]);
    ov5640_write_reg(0x3A1B, ov5640_exposure_tbl[level][2]);
    ov5640_write_reg(0x3A1E, ov5640_exposure_tbl[level][3]);
    ov5640_write_reg(0x3A11, ov5640_exposure_tbl[level][4]);
    ov5640_write_reg(0x3A1F, ov5640_exposure_tbl[level][5]);
    ov5640_write_reg(0x3212, 0x13);
    ov5640_write_reg(0x3212, 0xA3);
}

/**
 * @brief   设置OV5640亮度
 * @param   level: 亮度值
 * @arg     0~8: -4~+4
 * @retval  无
 */
void ov5640_brightness(uint8_t level)
{
    uint8_t value;
    
    if (level < 4)
    {
        value = 4 - level;
    }
    else
    {
        value = level - 4;
    }
    
    ov5640_write_reg(0x3212, 0x03);
    ov5640_write_reg(0x5587, value << 4);
    if (level < 4)
    {
        ov5640_write_reg(0x5588, 0x09);
    }
    else
    {
        ov5640_write_reg(0x5588, 0x01);
    }
    ov5640_write_reg(0x3212, 0x13);
    ov5640_write_reg(0x3212, 0xA3);
}

/* OV5640色彩饱和度配置数组 */
static const uint8_t ov5640_saturation_tbl[7][6] =
{
    {0x0C, 0x30, 0x3D, 0x3E, 0x3D, 0x01},   /* -3 */
    {0x10, 0x3D, 0x4D, 0x4E, 0x4D, 0x01},   /* -2 */
    {0x15, 0x52, 0x66, 0x68, 0x66, 0x02},   /* -1 */
    {0x1A, 0x66, 0x80, 0x82, 0x80, 0x02},   /* +0 */
    {0x1F, 0x7A, 0x9A, 0x9C, 0x9A, 0x02},   /* +1 */
    {0x24, 0x8F, 0xB3, 0xB6, 0xB3, 0x03},   /* +2 */
    {0x2B, 0xAB, 0xD6, 0xDA, 0xD6, 0x04},   /* +3 */
};

/**
 * @brief   设置OV5640色彩饱和度
 * @param   level: 色彩饱和度等级
 * @arg     0~6: -3~+3
 * @retval  无
 */
void ov5640_color_saturation(uint8_t level)
{
    uint8_t i;
    
    ov5640_write_reg(0x3212, 0x03);
    ov5640_write_reg(0x5381, 0x1C);
    ov5640_write_reg(0x5382, 0x5A);
    ov5640_write_reg(0x5383, 0x06);
    for (i=0; i<sizeof(ov5640_saturation_tbl[0]); i++)
    {
        ov5640_write_reg(0x5384 + i, ov5640_saturation_tbl[level][i]);
    }
    ov5640_write_reg(0x538B, 0x98);
    ov5640_write_reg(0x538A, 0x01);
    ov5640_write_reg(0x3212, 0x13);
    ov5640_write_reg(0x3212, 0xA3);
}

/**
 * @brief   设置OV5640对比度
 * @param   level: 对比度等级
 * @arg     0~6: -3~+3
 * @retval  无
 */
void ov5640_contrast(uint8_t level)
{
    uint8_t reg5585;
    uint8_t reg5586;
    
    switch (level)
    {
        case 0:
        {
            reg5585 = 0x14;
            reg5586 = 0x14;
            break;
        }
        case 1:
        {
            reg5585 = 0x18;
            reg5586 = 0x18;
            break;
        }
        case 2:
        {
            reg5585 = 0x1C;
            reg5586 = 0x1C;
            break;
        }
        case 3:
        default:
        {
            reg5585 = 0x00;
            reg5586 = 0x20;
            break;
        }
        case 4:
        {
            reg5585 = 0x10;
            reg5586 = 0x24;
            break;
        }
        case 5:
        {
            reg5585 = 0x18;
            reg5586 = 0x28;
            break;
        }
        case 6:
        {
            reg5585 = 0x1C;
            reg5586 = 0x2C;
            break;
        }
    }
    
    ov5640_write_reg(0x3212, 0x03);
    ov5640_write_reg(0x5585, reg5585);
    ov5640_write_reg(0x5586, reg5586);
    ov5640_write_reg(0x3212, 0x13);
    ov5640_write_reg(0x3212, 0xA3);
}

/**
 * @brief   设置OV5640锐度
 * @param   level: 锐度等级
 * @arg     0~32: 锐度
 * @arg     33: 自动锐度
 * @retval  无
 */
void ov5640_sharpness(uint8_t level)
{
    if (level < 33)
    {
        ov5640_write_reg(0x5308, 0x65);
        ov5640_write_reg(0x5302, level);
    }
    else
    {
        ov5640_write_reg(0x5308, 0x25);
        ov5640_write_reg(0x5300, 0x08);
        ov5640_write_reg(0x5301, 0x30);
        ov5640_write_reg(0x5302, 0x10);
        ov5640_write_reg(0x5303, 0x00);
        ov5640_write_reg(0x5309, 0x08);
        ov5640_write_reg(0x530A, 0x30);
        ov5640_write_reg(0x530B, 0x04);
        ov5640_write_reg(0x530C, 0x06);
    }
}

/* OV5640特效配置数组 */
static const uint8_t ov5640_effects_tbl[7][3] =
{
    {0x06, 0x40, 0x10}, /* 正常 */
    {0x1E, 0xA0, 0x40}, /* 冷色 */
    {0x1E, 0x80, 0xC0}, /* 暖色 */
    {0x1E, 0x80, 0x80}, /* 黑白 */
    {0x1E, 0x40, 0xA0}, /* 泛黄 */
    {0x40, 0x40, 0x10}, /* 反色 */
    {0x1E, 0x60, 0x60}, /* 偏绿 */
};

/**
 * @brief   设置OV5640特效
 * @param   effects: 特效
 * @arg     0: 正常
 * @arg     1: 冷色
 * @arg     2: 暖色
 * @arg     3: 黑白
 * @arg     4: 泛黄
 * @arg     5: 反色
 * @arg     6: 偏绿
 * @retval  无
 */
void ov5640_special_effects(uint8_t effects)
{
    ov5640_write_reg(0x3212, 0x03);
    ov5640_write_reg(0x5580, ov5640_effects_tbl[effects][0]);
    ov5640_write_reg(0x5583, ov5640_effects_tbl[effects][1]);
    ov5640_write_reg(0x5584, ov5640_effects_tbl[effects][2]);
    ov5640_write_reg(0x5003, 0x08);
    ov5640_write_reg(0x3212, 0x13);
    ov5640_write_reg(0x3212, 0xA3);
}

/**
 * @brief   OV5640测试
 * @param   mode: 测试模式
 * @arg     0: 关闭测试
 * @arg     1: 彩条测试
 * @arg     2: 色块测试
 * @retval  无
 */
void ov5640_test_pattern(uint8_t mode)
{
    if (mode == 0)
    {
        ov5640_write_reg(0x503D, 0X00);
    }
    else if (mode == 1)
    {
        ov5640_write_reg(0x503D, 0X80);
    }
    else if (mode == 2)
    {
        ov5640_write_reg(0x503D, 0X82);
    }
}

/**
 * @brief   设置OV5640输出图像大小
 * @param   offx: 输出图像在ov5640_image_window_set设定窗口的X方向偏移
 * @param   offy: 输出图像在ov5640_image_window_set设定窗口的Y方向偏移
 * @param   width: 输出图像宽度
 * @param   height: 输出图像高度
 * @retval  设置结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t ov5640_outsize_set(uint16_t offx, uint16_t offy, uint16_t width, uint16_t height)
{
    ov5640_write_reg(0x3212, 0x03);
    
    /* 输出尺寸（带缩放） */
    ov5640_write_reg(0x3808, width >> 8);
    ov5640_write_reg(0x3809, width);
    ov5640_write_reg(0x380A, height >> 8);
    ov5640_write_reg(0x380B, height);
    
    /* 输出尺寸在ISP上面的取图范围
     * 范围: xsize-2*offx,ysize-2*offy
     */
    ov5640_write_reg(0x3810, offx >> 8);
    ov5640_write_reg(0x3811, offx);
    ov5640_write_reg(0x3812, offy >> 8);
    ov5640_write_reg(0x3813, offy);
    
    ov5640_write_reg(0x3212, 0x13);
    ov5640_write_reg(0x3212, 0xA3);
    
    return 0;
}

/**
 * @brief   设置OV5640图像开窗大小
 * @param   offx: 图像在传感器的X方向偏移
 * @param   offy: 图像在传感器的Y方向偏移
 * @param   width: 图像宽度
 * @param   height: 图像高度
 * @retval  设置结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t ov5640_image_window_set(uint16_t offx, uint16_t offy, uint16_t width, uint16_t height)
{
    uint16_t xst;
    uint16_t yst;
    uint16_t xend;
    uint16_t yend;
    
    xst = offx;
    yst = offy;
    xend = offx + width - 1;
    yend = offy + height - 1;
    
    ov5640_write_reg(0x3212, 0x03);
    ov5640_write_reg(0x3800, xst >> 8);
    ov5640_write_reg(0x3801, xst);
    ov5640_write_reg(0x3802, yst >> 8);
    ov5640_write_reg(0x3803, yst);
    ov5640_write_reg(0x3804, xend >> 8);
    ov5640_write_reg(0x3805, xend);
    ov5640_write_reg(0x3806, yend >> 8);
    ov5640_write_reg(0x3807, yend);
    ov5640_write_reg(0x3212, 0x13);
    ov5640_write_reg(0x3212, 0xA3);
    
    return 0;
}

/**
 * @brief   初始化OV5640自动对焦
 * @param   无
 * @retval  初始化结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t ov5640_focus_init(void)
{
    uint16_t i;
    uint8_t state;
    
    ov5640_write_reg(0x3000, 0x20);
    
    for (i=0; i<(sizeof(ov5640_af_config) / sizeof(ov5640_af_config[0])); i++)
    {
        ov5640_write_reg(0x8000 + i, ov5640_af_config[i]);
    }
    
    ov5640_write_reg(0x3022, 0x00);
    ov5640_write_reg(0x3023, 0x00);
    ov5640_write_reg(0x3024, 0x00);
    ov5640_write_reg(0x3025, 0x00);
    ov5640_write_reg(0x3026, 0x00);
    ov5640_write_reg(0x3027, 0x00);
    ov5640_write_reg(0x3028, 0x00);
    ov5640_write_reg(0x3029, 0x7F);
    ov5640_write_reg(0x3000, 0x00);
    
    i = 0;
    do {
        state = ov5640_read_reg(0x3029);
        HAL_Delay(5);
        if (++i > 1000)
        {
            return 1;
        }
    } while (state != 0x70);
    
    return 0;
}

/**
 * @brief   OV5640单次自动对焦
 * @param   无
 * @retval  对焦结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t ov5640_focus_single(void)
{
    uint8_t state;
    uint16_t i;
    
    ov5640_write_reg(0x3022, 0x03);
    
    i = 0;
    do {
        state = ov5640_read_reg(0x3029);
        HAL_Delay(5);
        if (++i > 1000)
        {
            return 1;
        }
    } while (state != 0x10);
    
    return 0;
}

/**
 * @brief   OV5640连续自动对焦
 * @param   无
 * @retval  对焦结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t ov5640_focus_constant(void)
{
    uint8_t state;
    uint16_t i;
    
    ov5640_write_reg(0x3023, 0x01);
    ov5640_write_reg(0x3022, 0x08);
    
    i = 0;
    do {
        state = ov5640_read_reg(0x3023);
        HAL_Delay(5);
        if (++i > 1000)
        {
            return 1;
        }
    } while (state != 0x00);
    
    ov5640_write_reg(0x3023, 0x01);
    ov5640_write_reg(0x3022, 0x04);
    
    i = 0;
    do {
        state = ov5640_read_reg(0x3023);
        HAL_Delay(5);
        if (++i > 1000)
        {
            return 1;
        }
    } while (state != 0x00);
    
    return 0;
}
