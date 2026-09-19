/**
 ******************************************************************************
 * @file    mt6701.c
 * @brief   MT6701 磁编码器驱动 — 硬件 I²C (HAL_I2C) 实现 (句柄化, 多实例)
 *
 * 读取规则 (MT6701 datasheet):
 *   - 7-bit 从机地址 0x06, 写地址 0x0C / 读地址 0x0D
 *   - 读寄存器: 写寄存器地址(8bit) + Repeated START + 读数据  (= 标准内存读)
 *   - 角度 14-bit: 0x03 = D13..D6, 0x04 = D5..D0(高 6 位)
 *                  raw = (buf[0] << 6) | (buf[1] >> 2)
 *   - WHOAMI (0x0F) = 0x67
 *
 * I2C 引脚映射 (STM32F407VET6):
 *   I2C1: SCL=PB6  SDA=PB7  AF=GPIO_AF4_I2C1
 *   I2C2: SCL=PB10 SDA=PB11 AF=GPIO_AF4_I2C2
 ******************************************************************************
 */
#include "mt6701.h"

/* ========================================================================= */
/*  常量                                                                      */
/* ========================================================================= */
#define MT6701_I2C_TIMEOUT  50U   /* ms, 阻塞超时 */

/* ========================================================================= */
/*  I²C 事务 (HAL 内存读/写 = 寄存器地址 + 数据)                              */
/* ========================================================================= */

/** 读寄存器: 写地址(0x0C)+寄存器 → Repeated START → 读地址(0x0D)+数据 */
static uint8_t mt6701_read_reg(MT6701_HandleTypeDef *h, uint8_t reg, uint8_t *buf, uint8_t len) {
    if (HAL_I2C_Mem_Read(h->hi2c, (uint16_t)(MT6701_I2C_ADDR << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, MT6701_I2C_TIMEOUT) != HAL_OK) {
        return 0;
    }
    return 1;
}

/** 写寄存器: 写地址(0x0C)+寄存器+数据 */
static uint8_t mt6701_write_reg(MT6701_HandleTypeDef *h, uint8_t reg, uint8_t val) {
    if (HAL_I2C_Mem_Write(h->hi2c, (uint16_t)(MT6701_I2C_ADDR << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, &val, 1, MT6701_I2C_TIMEOUT) != HAL_OK) {
        return 0;
    }
    return 1;
}

/* ========================================================================= */
/*  公开 API                                                                  */
/* ========================================================================= */

int MT6701_Init(MT6701_HandleTypeDef *h, I2C_HandleTypeDef *hi2c) {
    GPIO_TypeDef *scl_port, *sda_port;
    uint16_t      scl_pin,  sda_pin;
    uint8_t       af;

    h->hi2c     = hi2c;
    h->inited   = 0;
    h->last_angle = 0.0f;
    h->turn_cnt   = 0;
    h->is_first   = 1;

    /* I2C 实例 → 引脚/复用映射 */
    if (hi2c->Instance == I2C1) {
        scl_port = GPIOB; scl_pin = GPIO_PIN_6;
        sda_port = GPIOB; sda_pin = GPIO_PIN_7;
        af = GPIO_AF4_I2C1;
    } else if (hi2c->Instance == I2C2) {
        scl_port = GPIOB; scl_pin = GPIO_PIN_10;
        sda_port = GPIOB; sda_pin = GPIO_PIN_11;
        af = GPIO_AF4_I2C2;
    } else {
        return -1;
    }

    /* 总线恢复: 设备热插拔可能导致 I2C 外设 BUSY 卡死, 9 脉冲 + STOP 清除 */
    {
        GPIO_InitTypeDef g = {0};
        uint8_t i;
        g.Pin = scl_pin | sda_pin;
        g.Mode = GPIO_MODE_OUTPUT_OD;
        g.Pull = GPIO_PULLUP;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(scl_port, &g);
        HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
        for (i = 0; i < 9; i++) {
            HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
            HAL_Delay(1);
        }
        HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
        HAL_Delay(1);
        hi2c->Instance->CR1 &= ~I2C_CR1_PE;
        hi2c->Instance->CR1 |=  I2C_CR1_PE;
        g.Mode = GPIO_MODE_AF_OD;
        g.Alternate = af;
        HAL_GPIO_Init(scl_port, &g);
    }

    /* 验证器件存在: 地址 0x06 应答 (实测部分版本 WHOAMI(0x0F) 返回 0x00, 不可用作校验) */
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(MT6701_I2C_ADDR << 1), 3, 10) != HAL_OK)
        return -1;

    h->inited = 1;
    return 0;
}

uint16_t MT6701_ReadRaw(MT6701_HandleTypeDef *h) {
    uint8_t buf[2];
    if (!h->inited) return 0;
    if (!mt6701_read_reg(h, MT6701_REG_ANGLE_MSB, buf, 2)) return 0;

    /* MT6701 14-bit 寄存器拼合：[0x03]: D13~D6, [0x04]: D5~D0 在高 6 位 */
    uint16_t raw = ((uint16_t)buf[0] << 6) | (buf[1] >> 2);
    return raw & 0x3FFF;
}

float MT6701_ReadDegrees(MT6701_HandleTypeDef *h) {
    return (float)MT6701_ReadRaw(h) * MT6701_LSB_DEG;
}

float MT6701_ReadRadians(MT6701_HandleTypeDef *h) {
    return (float)MT6701_ReadRaw(h) * (6.283185307179586f / MT6701_CPR);
}

float MT6701_ReadMultiTurn(MT6701_HandleTypeDef *h, int32_t *turns) {
    float current = MT6701_ReadDegrees(h);

    if (h->is_first) {
        h->last_angle = current;
        h->is_first = 0;
    } else {
        float delta = current - h->last_angle;
        if (delta > 180.0f) {
            h->turn_cnt--;
        } else if (delta < -180.0f) {
            h->turn_cnt++;
        }
        h->last_angle = current;
    }

    if (turns) *turns = h->turn_cnt;
    return current;
}

uint8_t MT6701_IsMagnetOK(MT6701_HandleTypeDef *h) {
    uint8_t status = 0;
    if (!h->inited) return 0;
    if (!mt6701_read_reg(h, MT6701_REG_STATUS, &status, 1)) return 0;
    return ((status & 0x01) == 0) ? 1 : 0;
}

uint8_t MT6701_IsDataUpdated(MT6701_HandleTypeDef *h) {
    uint8_t status = 0;
    if (!h->inited) return 0;
    if (!mt6701_read_reg(h, MT6701_REG_STATUS, &status, 1)) return 0;
    return (status & 0x02) ? 1 : 0;
}

int MT6701_SetZeroPosition(MT6701_HandleTypeDef *h) {
    if (!h->inited) return -1;
    uint16_t raw = MT6701_ReadRaw(h);

    uint8_t hi = (uint8_t)(raw >> 6);
    uint8_t lo = (uint8_t)((raw & 0x3F) << 2);

    if (!mt6701_write_reg(h, MT6701_REG_ZERO_MSB, hi)) return -2;
    if (!mt6701_write_reg(h, MT6701_REG_ZERO_LSB, lo)) return -2;
    return 0;
}

void MT6701_SetFilter(MT6701_HandleTypeDef *h, uint8_t level) {
    uint8_t ctrl = 0;
    if (!h->inited) return;
    if (!mt6701_read_reg(h, MT6701_REG_CTRL, &ctrl, 1)) return;

    ctrl &= ~MT6701_CTRL_FILTER_MASK;
    ctrl |= (level & MT6701_CTRL_FILTER_MASK);

    mt6701_write_reg(h, MT6701_REG_CTRL, ctrl);
}