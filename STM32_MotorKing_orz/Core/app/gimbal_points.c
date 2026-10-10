/**
 ******************************************************************************
 * @file    gimbal_points.c
 * @brief   Q1/Q2 标定点存储实现 (纯数据; 按键标定流程见 calib.c)
 ******************************************************************************
 */
#include "gimbal_points.h"

/* ========================================================================= */
/*  内部状态                                                                   */
/* ========================================================================= */
static GimbalPoint s_points[GP_COUNT];

void GimbalPoints_Init(void)
{
    for (int i = 0; i < GP_COUNT; i++) {
        s_points[i].pan_deg  = 0.0f;
        s_points[i].tilt_deg = 0.0f;
        s_points[i].valid    = false;
    }
}

bool GimbalPoints_Record(GimbalPointId id, float pan_deg, float tilt_deg)
{
    if (id >= GP_COUNT) return false;
    s_points[id].pan_deg  = pan_deg;
    s_points[id].tilt_deg = tilt_deg;
    s_points[id].valid    = true;
    return true;
}

bool GimbalPoints_Get(GimbalPointId id, GimbalPoint *out)
{
    if (id >= GP_COUNT || out == NULL) return false;
    *out = s_points[id];
    return s_points[id].valid;
}

bool GimbalPoints_IsValid(GimbalPointId id)
{
    if (id >= GP_COUNT) return false;
    return s_points[id].valid;
}

const GimbalPoint *GimbalPoints_All(void)
{
    return s_points;
}
