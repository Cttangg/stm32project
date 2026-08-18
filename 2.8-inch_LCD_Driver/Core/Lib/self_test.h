/**
 ******************************************************************************
 * @file    self_test.h
 * @brief   驱动自检接口 — LCD 色条/读点校验, 触摸有效点检测, 串口回环
 *
 * 使用:
 *   初始化完成后调用 LIB_SelfTest(UART_P1), 结果经串口输出.
 *   单独运行某项自检可调用 LCD_SelfTest() / TP_SelfTest() / UART_SelfTest()
 *
 * @note   UART_SelfTest() 依赖应用注册的回显帧 (header_len=0 逐字节回调),
 *         等待期间内部会调用 UART_Task() 处理收包
 ******************************************************************************
 */
#ifndef __SELF_TEST_H
#define __SELF_TEST_H

#include "uart.h"

/**
 * @brief  LCD 自检: 五色条显示 + 边框线 + 读点回读校验
 * @retval 1=通过, 0=读点校验失败
 * @note   屏幕内容会被自检图案覆盖
 */
uint8_t LCD_SelfTest(void);

/**
 * @brief  触摸自检: 等待一次有效按压 (坐标落在屏幕范围内)
 * @retval 1=通过, 0=超时/读数无效
 * @note   屏幕提示触摸, 5 秒内未检测到有效按压判失败
 */
uint8_t TP_SelfTest(void);

/**
 * @brief  串口自检: 发送测试帧并期待回显匹配
 * @param  port: 串口端口
 * @retval 1=通过, 0=超时/回显不匹配
 * @note   等待期间内部调用 UART_Task()
 */
uint8_t UART_SelfTest(UART_Port port);

/**
 * @brief  全量自检: 依次执行 LCD/TP/UART 自检, 结果经串口输出
 * @param  port: 结果输出与回环测试所用串口端口
 * @retval 1=全部通过, 0=存在失败项
 */
uint8_t LIB_SelfTest(UART_Port port);

#endif /* __SELF_TEST_H */
