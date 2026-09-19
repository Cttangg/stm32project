/**
 ******************************************************************************
 * @file    app.h
 * @brief   Application 层入口 (双轴 XY 角度闭环, 丝杆 2mm/r)
 *
 * 当前职责:
 *   - 创建/持有 X、Y 两个 AxisController, 各自绑定 MT6701 角度反馈
 *   - 在主循环被周期调度
 *   - 串口命令接口 (pos/cm, cw/ccw, stop, ...)
 *
 * 安全约束: 无指令不自走; 位置不允许负值 (钳到 0);
 *           初始化预留自动校零 (当前以上电位置为 0 点)。
 ******************************************************************************
 */
#ifndef __APP_H
#define __APP_H

#include "motor_stepper.h"
#include "mt6701.h"
#include "uart.h"
#include "axis_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化应用层双轴
 * @param x_stepper X 轴步进执行器 (电机1, TIM14)
 * @param x_encoder X 轴编码器 (I2C1, 可为 NULL)
 * @param y_stepper Y 轴步进执行器 (电机2, TIM13)
 * @param y_encoder Y 轴编码器 (I2C2, 可为 NULL)
 * @param debug_uart 调试串口 (USART1)
 */
void App_Init(MotorStepper *x_stepper, MT6701_HandleTypeDef *x_encoder,
              MotorStepper *y_stepper, MT6701_HandleTypeDef *y_encoder,
              UART_Device *debug_uart);

/** @brief 周期调度 (主循环调用, 非阻塞) */
void App_Update(void);

/* ── 命令接口 (供串口解析层调用) ── */

/** @brief 移动到 (x_cm, y_cm) 位置, 单位厘米 (负值非法) */
AxisResult_t App_MovePosCM(float x_cm, float y_cm);

/** @brief 自动校零 (当前: 以上电位置为 0 点) */
AxisResult_t App_Home(void);

/** @brief 闭环: X 轴相对转动圈数 (+ 逆时针, - 顺时针) */
AxisResult_t App_MoveTurns(float turns);

/** @brief 闭环: Y 轴(电机2)相对转动圈数 (+ 逆时针, - 顺时针) */
AxisResult_t App_MoveTurnsY(float turns);

/** @brief 停止两轴 */
AxisResult_t App_Stop(void);

/** @brief 使能两轴 */
AxisResult_t App_Enable(void);

/** @brief 停止并禁止两轴 */
void App_Disable(void);

/** @brief 查询 X 轴状态快照 */
AxisStatus_t App_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
