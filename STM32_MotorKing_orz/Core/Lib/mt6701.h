/**
 ******************************************************************************
 * @file    mt6701.h
 * @brief   MT6701 磁编码器 I²C 驱动 — 公开 API (句柄化, 多实例)
 *
 * 传感器: MagnTek MT6701 14-bit 霍尔磁编码器
 * 接口:   I²C 硬件外设 (I2C1: PB6=SCL PB7=SDA, I2C2: PB10=SCL PB11=SDA, 400kHz)
 * 分辨率: 14-bit (16384 点/圈, ≈0.022°/LSB)
 * 地址:   0x06 (7-bit)
 *
 * 多电机: 每个电机一个 MT6701_HandleTypeDef, 绑定各自的 I2C 句柄,
 *         所有 API 接收句柄指针, 传哪个句柄就读哪个编码器.
 ******************************************************************************
 */
#ifndef __MT6701_H
#define __MT6701_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ========================================================================= */
/*  I²C 地址                                                                  */
/* ========================================================================= */
#define MT6701_I2C_ADDR       0x06    /* 7-bit 地址 */

/* ========================================================================= */
/*  寄存器映射                                                                */
/* ========================================================================= */
#define MT6701_REG_CTRL       0x01    /* 控制寄存器 */
#define MT6701_REG_STATUS     0x02    /* 状态寄存器 */
#define MT6701_REG_ANGLE_MSB  0x03    /* 角度高字节 (13:6) */
#define MT6701_REG_ANGLE_LSB  0x04    /* 角度低字节 (5:0) */
#define MT6701_REG_ZERO_MSB   0x05    /* 零位补偿高字节 */
#define MT6701_REG_ZERO_LSB   0x06    /* 零位补偿低字节 */
#define MT6701_REG_WHOAMI     0x0F    /* 芯片 ID (部分版本返回 0x00, 勿用作校验) */

/* ========================================================================= */
/*  分辨率常量                                                                */
/* ========================================================================= */
#define MT6701_CPR            16384.0f   /* 每圈计数值 (2^14) */
#define MT6701_LSB_DEG        0.02197265625f /* 每 LSB 角度 */

/* ========================================================================= */
/*  控制寄存器位定义                                                           */
/* ========================================================================= */
#define MT6701_CTRL_FILTER_MASK  0x0C   /* 滤波级别 [3:2] */
#define MT6701_CTRL_FILTER_LOW   0x00   /* 低滤波, 响应 0.5ms */
#define MT6701_CTRL_FILTER_MED   0x04   /* 中滤波, 响应 2ms */
#define MT6701_CTRL_FILTER_HIGH  0x08   /* 高滤波, 响应 8ms */

/* ========================================================================= */
/*  编码器句柄                                                                 */
/* ========================================================================= */
typedef struct {
    I2C_HandleTypeDef *hi2c;    /* 绑定的 I2C 外设 */
    uint8_t           inited;   /* 初始化标志 */
    float             last_angle; /* 多圈跟踪: 上次角度 */
    int32_t           turn_cnt;   /* 多圈跟踪: 累计整圈数 */
    uint8_t           is_first;   /* 多圈跟踪: 首读标志 */
} MT6701_HandleTypeDef;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/**
 * @brief  初始化 MT6701 (含总线恢复)
 * @param  h     编码器句柄
 * @param  hi2c  绑定的 I2C 句柄 (I2C1 / I2C2)
 * @retval  0 = 成功 (地址 0x06 应答)
 * @retval -1 = 无器件 (未接线/未上电)
 */
int MT6701_Init(MT6701_HandleTypeDef *h, I2C_HandleTypeDef *hi2c);

/**
 * @brief  读取原始 14-bit 角度值
 * @return 0~16383
 */
uint16_t MT6701_ReadRaw(MT6701_HandleTypeDef *h);

/**
 * @brief  读取角度 (度)
 * @return 0.0f ~ 360.0f
 */
float MT6701_ReadDegrees(MT6701_HandleTypeDef *h);

/**
 * @brief  读取角度 (弧度)
 * @return 0.0f ~ 2π
 */
float MT6701_ReadRadians(MT6701_HandleTypeDef *h);

/**
 * @brief  读取累积圈数和圈内角度 (多圈跟踪)
 * @param  h     编码器句柄
 * @param  turns  输出: 累计整圈数 (带符号)
 * @return 圈内角度 (0~360°)
 */
float MT6701_ReadMultiTurn(MT6701_HandleTypeDef *h, int32_t *turns);

/**
 * @brief  检查磁铁是否正常
 * @retval 1 = 正常, 0 = 太远/太近/缺失
 */
uint8_t MT6701_IsMagnetOK(MT6701_HandleTypeDef *h);

/**
 * @brief  检查数据是否更新 (自上次读取后)
 * @retval 1 = 有新数据, 0 = 无更新
 */
uint8_t MT6701_IsDataUpdated(MT6701_HandleTypeDef *h);

/**
 * @brief  设置零位 (将当前角度作为 0°)
 * @retval 0 = 成功
 */
int MT6701_SetZeroPosition(MT6701_HandleTypeDef *h);

/**
 * @brief  设置滤波级别
 * @param  level: MT6701_CTRL_FILTER_LOW / MED / HIGH
 */
void MT6701_SetFilter(MT6701_HandleTypeDef *h, uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* __MT6701_H */