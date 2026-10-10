/**
 ******************************************************************************
 * @file    gimbal_points.h
 * @brief   Q1/Q2 标定点 — 电机角度空间存储 O / 左上 / 右上 / 右下 / 左下
 *
 * Q1/Q2 用电机角度直接闭环, 因此每个屏幕目标点用一对 (pan_deg, tilt_deg)
 * 表示. 标定 = 把激光手动移动到某点, 记录当时 Pan/Tilt 编码器角度.
 *
 * 点序 (顺时针): O(原点, 复位目标) → 左上 → 右上 → 右下 → 左下
 ******************************************************************************
 */
#ifndef __GIMBAL_POINTS_H
#define __GIMBAL_POINTS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  点标识                                                                     */
/* ========================================================================= */
typedef enum
{
    GP_ORIGIN = 0,   /* O  原点 (复位目标) */
    GP_LEFT_UP,      /* 左上 */
    GP_RIGHT_UP,     /* 右上 */
    GP_RIGHT_DOWN,   /* 右下 */
    GP_LEFT_DOWN,    /* 左下 */
    GP_COUNT
} GimbalPointId;

/* ========================================================================= */
/*  点数据                                                                     */
/* ========================================================================= */
typedef struct
{
    float pan_deg;
    float tilt_deg;
    bool  valid;
} GimbalPoint;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 (全部置零/无效) */
void GimbalPoints_Init(void);

/** @brief 记录某点当前的 Pan/Tilt 角度 */
bool GimbalPoints_Record(GimbalPointId id, float pan_deg, float tilt_deg);

/** @brief 读取某点 (未标定为 invalid) */
bool GimbalPoints_Get(GimbalPointId id, GimbalPoint *out);

/** @brief 该点是否已标定 */
bool GimbalPoints_IsValid(GimbalPointId id);

/** @brief 只读访问整个数组 (len = GP_COUNT) */
const GimbalPoint *GimbalPoints_All(void);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_POINTS_H */
