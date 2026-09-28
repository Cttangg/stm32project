/**
 ******************************************************************************
 * @file    coordinate_transform.h
 * @brief   坐标转换 — 像素偏移 → Pan/Tilt 角度增量 (线性模型)
 *
 * 第一阶段 (requirements.md §16):
 *   ex = target_x - center_x
 *   ey = target_y - center_y
 *   pan_delta  = ex * pixel_to_angle_x
 *   tilt_delta = ey * pixel_to_angle_y
 *
 * 未来可升级为相机内参/射线模型 (requirements.md §17), 接口不变.
 * 本模块不能访问电机驱动器, 不能发送运动命令.
 ******************************************************************************
 */
#ifndef __COORDINATE_TRANSFORM_H
#define __COORDINATE_TRANSFORM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  输出                                                                      */
/* ========================================================================= */
typedef struct
{
    float pan_delta;    /* Pan 角度增量 deg */
    float tilt_delta;   /* Tilt 角度增量 deg */
} TrackingOffset;

/* ========================================================================= */
/*  配置                                                                      */
/* ========================================================================= */
typedef struct
{
    float center_x;         /* 图像中心 X px */
    float center_y;         /* 图像中心 Y px */
    float pixel_to_angle_x; /* X 像素→Pan 角度 系数 deg/px */
    float pixel_to_angle_y; /* Y 像素→Tilt 角度 系数 deg/px */
} CoordTransformConfig;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 */
void CoordTransform_Init(const CoordTransformConfig *cfg);

/** @brief 像素误差 → 角度增量 */
TrackingOffset CoordTransform_ComputeOffset(float target_x, float target_y);

#ifdef __cplusplus
}
#endif

#endif /* __COORDINATE_TRANSFORM_H */