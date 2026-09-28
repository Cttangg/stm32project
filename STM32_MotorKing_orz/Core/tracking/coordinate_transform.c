/**
 ******************************************************************************
 * @file    coordinate_transform.c
 * @brief   坐标转换实现 — 线性像素→角度模型
 ******************************************************************************
 */
#include "coordinate_transform.h"

static CoordTransformConfig s_cfg;
static bool s_inited;

void CoordTransform_Init(const CoordTransformConfig *cfg)
{
    s_cfg    = *cfg;
    s_inited = true;
}

TrackingOffset CoordTransform_ComputeOffset(float target_x, float target_y)
{
    TrackingOffset off = {0.0f, 0.0f};
    float ex, ey;

    if (!s_inited) return off;

    ex = target_x - s_cfg.center_x;
    ey = target_y - s_cfg.center_y;

    off.pan_delta  = ex * s_cfg.pixel_to_angle_x;
    off.tilt_delta = ey * s_cfg.pixel_to_angle_y;

    return off;
}