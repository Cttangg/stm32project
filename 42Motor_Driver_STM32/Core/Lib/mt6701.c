/**
 ******************************************************************************
 * @file    mt6701.c
 * @brief   MT6701 磁编码器驱动 — 硬件 I²C (HAL_I2C) 实现
 *
 * 读取规则 (MT6701 datasheet):
 *   - 7-bit 从机地址 0x06, 写地址 0x0C / 读地址 0x0D
 *   - 读寄存器: 写寄存器地址(8bit) + Repeated START + 读数据  (= 标准内存读)
 *   - 角度 14-bit: 0x03 = D13..D6, 0x04 = D5..D0(高 6 位)
 *                  raw = (buf[0] << 6) | (buf[1] >> 2)
 *   - WHOAMI (0x0F) = 0x67
 ******************************************************************************
 */
#include "mt6701.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"

/* ========================================================================= */
/*  常量                                                                      */
/* ========================================================================= */
#define MT6701_I2C_TIMEOUT  50U   /* ms, 阻塞超时 */

/* ========================================================================= */
/*  内部状态                                                                  */
/* ========================================================================= */
static uint8_t g_inited = 0;

/* ========================================================================= */
/*  I²C 事务 (HAL 内存读/写 = 寄存器地址 + 数据)                              */
/* ========================================================================= */

/** 读寄存器: 写地址(0x0C)+寄存器 → Repeated START → 读地址(0x0D)+数据 */
static uint8_t mt6701_read_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(MT6701_I2C_ADDR << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, MT6701_I2C_TIMEOUT) != HAL_OK) {
        return 0;
    }
    return 1;
}

/** 写寄存器: 写地址(0x0C)+寄存器+数据 */
static uint8_t mt6701_write_reg(uint8_t reg, uint8_t val) {
    if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(MT6701_I2C_ADDR << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, &val, 1, MT6701_I2C_TIMEOUT) != HAL_OK) {
        return 0;
    }
    return 1;
}

/* ========================================================================= */
/*  公开 API                                                                  */
/* ========================================================================= */

int MT6701_Init(void) {
    /* 总线恢复: 设备热插拔可能导致 I2C 外设 BUSY 卡死, 9 脉冲 + STOP 清除 */
    {
        GPIO_InitTypeDef g = {0};
        uint8_t i;
        g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        g.Mode = GPIO_MODE_OUTPUT_OD;
        g.Pull = GPIO_PULLUP;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &g);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        for (i = 0; i < 9; i++) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_Delay(1);
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_Delay(1);
        hi2c1.Instance->CR1 &= ~I2C_CR1_PE;
        hi2c1.Instance->CR1 |=  I2C_CR1_PE;
        g.Mode = GPIO_MODE_AF_OD;
        g.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &g);
    }

    /* 验证器件存在: 地址 0x06 应答 (实测部分版本 WHOAMI(0x0F) 返回 0x00, 不可用作校验) */
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(MT6701_I2C_ADDR << 1), 3, 10) != HAL_OK)
        return -1;

    g_inited = 1;
    return 0;
}

uint16_t MT6701_ReadRaw(void) {
    uint8_t buf[2];
    if (!g_inited) return 0;
    if (!mt6701_read_reg(MT6701_REG_ANGLE_MSB, buf, 2)) return 0;

    /* MT6701 14-bit 寄存器拼合：[0x03]: D13~D6, [0x04]: D5~D0 在高 6 位 */
    uint16_t raw = ((uint16_t)buf[0] << 6) | (buf[1] >> 2);
    return raw & 0x3FFF;
}

float MT6701_ReadDegrees(void) {
    return (float)MT6701_ReadRaw() * MT6701_LSB_DEG;
}

float MT6701_ReadRadians(void) {
    return (float)MT6701_ReadRaw() * (6.283185307179586f / MT6701_CPR);
}

float MT6701_ReadMultiTurn(int32_t *turns) {
    static float  last_angle = 0.0f;
    static int32_t turn_cnt  = 0;
    static uint8_t is_first  = 1;

    float current = MT6701_ReadDegrees();

    if (is_first) {
        last_angle = current;
        is_first = 0;
    } else {
        float delta = current - last_angle;
        if (delta > 180.0f) {
            turn_cnt--;
        } else if (delta < -180.0f) {
            turn_cnt++;
        }
        last_angle = current;
    }

    if (turns) *turns = turn_cnt;
    return current;
}

uint8_t MT6701_IsMagnetOK(void) {
    uint8_t status = 0;
    if (!g_inited) return 0;
    if (!mt6701_read_reg(MT6701_REG_STATUS, &status, 1)) return 0;
    return ((status & 0x01) == 0) ? 1 : 0;
}

uint8_t MT6701_IsDataUpdated(void) {
    uint8_t status = 0;
    if (!g_inited) return 0;
    if (!mt6701_read_reg(MT6701_REG_STATUS, &status, 1)) return 0;
    return (status & 0x02) ? 1 : 0;
}

int MT6701_SetZeroPosition(void) {
    if (!g_inited) return -1;
    uint16_t raw = MT6701_ReadRaw();

    uint8_t hi = (uint8_t)(raw >> 6);
    uint8_t lo = (uint8_t)((raw & 0x3F) << 2);

    if (!mt6701_write_reg(MT6701_REG_ZERO_MSB, hi)) return -2;
    if (!mt6701_write_reg(MT6701_REG_ZERO_LSB, lo)) return -2;
    return 0;
}

void MT6701_SetFilter(uint8_t level) {
    if (!g_inited) return;
    uint8_t ctrl = 0;
    if (!mt6701_read_reg(MT6701_REG_CTRL, &ctrl, 1)) return;

    ctrl &= ~MT6701_CTRL_FILTER_MASK;
    ctrl |= (level & MT6701_CTRL_FILTER_MASK);

    mt6701_write_reg(MT6701_REG_CTRL, ctrl);
}
