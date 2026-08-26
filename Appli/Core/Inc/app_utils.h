/**
 ****************************************************************************************************
 * @file        app_utils.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-13
 * @brief       app_utils.h文件
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 */

#ifndef __APP_UTILS_H
#define __APP_UTILS_H

#ifndef MIN
#define MIN(a, b)               ((a) < (b) ? (a) : (b))
#endif

#define ALIGN_VALUE(v, a)       (((v) + (a) - 1) & ~((a) - 1))

#endif
