/**
 ******************************************************************************
 * @file    tmc2209.c
 * @brief   TMC2209 步进电机驱动板 — GPIO 控制实现
 *
 * 职责: Enable/Disable/Direction/Microstep + GPIO 引脚绑定.
 * STEP 脉冲由 motor_stepper.c 的硬件定时器生成, 本文件不做任何阻塞等待.
 ******************************************************************************
 */
#include "tmc2209.h"

/* ========================================================================= */
/*  GPIO 原语                                                                */
/* ========================================================================= */
static void pin_set(const TMC2209_PinDef *p) {
    HAL_GPIO_WritePin(p->port, p->pin, GPIO_PIN_SET);
}
static void pin_reset(const TMC2209_PinDef *p) {
    HAL_GPIO_WritePin(p->port, p->pin, GPIO_PIN_RESET);
}

/* ========================================================================= */
/*  公开 API                                                                  */
/* ========================================================================= */

void TMC2209_Init(TMC2209_HandleTypeDef *h, const TMC2209_PinConfig *cfg) {
    h->pins       = *cfg;
    h->enabled    = 0;
    h->dir        = TMC2209_DIR_CCW;
    h->step_count = 0;
    h->microstep  = TMC2209_MICROSTEP_32;   /* 默认 1/32 细分 */

    /* 初始电平: 禁止电机, STEP/DIR 低, MS1/MS2 = High/Low (= 1/32 细分) */
    pin_set(&h->pins.en);
    pin_reset(&h->pins.step);
    pin_reset(&h->pins.dir);
    pin_set(&h->pins.ms1);
    pin_reset(&h->pins.ms2);
}

void TMC2209_Enable(TMC2209_HandleTypeDef *h) {
    pin_reset(&h->pins.en);   /* EN 低有效 */
    h->enabled = 1;
}

void TMC2209_Disable(TMC2209_HandleTypeDef *h) {
    pin_set(&h->pins.en);     /* EN 高 = 禁止 */
    h->enabled = 0;
}

void TMC2209_SetDirection(TMC2209_HandleTypeDef *h, TMC2209_Dir dir) {
    if (dir == TMC2209_DIR_CW)
        pin_set(&h->pins.dir);
    else
        pin_reset(&h->pins.dir);
    h->dir = (uint8_t)dir;
}

void TMC2209_SetMicrostep(TMC2209_HandleTypeDef *h, TMC2209_Microstep ms) {
    switch (ms) {
        case TMC2209_MICROSTEP_8:   /* MS1=L MS2=L */
            pin_reset(&h->pins.ms1);
            pin_reset(&h->pins.ms2);
            break;
        case TMC2209_MICROSTEP_16:  /* MS1=H MS2=H */
            pin_set(&h->pins.ms1);
            pin_set(&h->pins.ms2);
            break;
        case TMC2209_MICROSTEP_32:  /* MS1=H MS2=L */
            pin_set(&h->pins.ms1);
            pin_reset(&h->pins.ms2);
            break;
        case TMC2209_MICROSTEP_64:  /* MS1=L MS2=H */
            pin_reset(&h->pins.ms1);
            pin_set(&h->pins.ms2);
            break;
        default:
            return;
    }
    h->microstep = ms;
}
