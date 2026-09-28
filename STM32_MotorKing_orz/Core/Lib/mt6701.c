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
#define MT6701_I2C_TIMEOUT  5U   /* ms, 阻塞超时 (短: 避免 I2C 故障拖死主循环) */

/* ========================================================================= */
/*  I²C 总线恢复 (9 SCK 法)                                                  */
/* ========================================================================= */

/**
 * @brief I²C 总线恢复 — 发送 9 个 SCK + STOP 释放被从机拉低的总线
 *
 * I2C 偶发故障 (从机拉低 SDA / 外设 BUSY 卡死) 的恢复方法:
 *   1. SCL/SDA 切为 GPIO 开漏输出 (带上拉)
 *   2. SCL 手动产生 9 个时钟脉冲 — 从机在收到第 9 个边沿后释放 SDA
 *   3. 产生 STOP 条件 (SDA 低→高, SCL 高期间)
 *   4. 复位 I2C 外设 (PE 关→开)
 *   5. 切回 AF 开漏, 由 HAL 重新接管
 *
 * 调用时机: MT6701_Init 初始化时 + 每次 I2C 事务失败后 (自愈重试)
 */
void MT6701_BusRecover(I2C_HandleTypeDef *hi2c) {
    GPIO_TypeDef *scl_port, *sda_port;
    uint16_t      scl_pin,  sda_pin;
    uint8_t       af;
    GPIO_InitTypeDef g = {0};
    uint8_t i;

    if (hi2c == NULL) return;

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
        return;
    }

    /* 先关外设, 避免 GPIO 切换时外设仍在驱动总线 */
    hi2c->Instance->CR1 &= ~I2C_CR1_PE;

    /* 切 GPIO: 开漏输出 + 上拉 */
    g.Pin  = scl_pin | sda_pin;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(scl_port, &g);
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);

    /* 9 个 SCK 脉冲 */
    for (i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET);
        for (volatile uint32_t d = 0; d < 1000u; d++) {}   /* ~µs 级忙等, 非阻塞 SysTick */
        HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
        for (volatile uint32_t d = 0; d < 1000u; d++) {}
    }

    /* STOP 条件: SDA 在 SCL 高期间 低→高 */
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET);
    for (volatile uint32_t d = 0; d < 1000u; d++) {}
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
    for (volatile uint32_t d = 0; d < 1000u; d++) {}
    HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
    for (volatile uint32_t d = 0; d < 1000u; d++) {}

    /* 重新使能外设, 切回 AF 开漏 */
    hi2c->Instance->CR1 |=  I2C_CR1_PE;
    g.Mode = GPIO_MODE_AF_OD;
    g.Alternate = af;
    HAL_GPIO_Init(scl_port, &g);
}

/* ========================================================================= */
/*  I²C 事务 (HAL 内存读/写 = 寄存器地址 + 数据, 失败自动恢复+重试)            */
/* ========================================================================= */

/** 读寄存器: 写地址(0x0C)+寄存器 → Repeated START → 读地址(0x0D)+数据 */
static uint8_t mt6701_read_reg(MT6701_HandleTypeDef *h, uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t retry;
    uint32_t now;
    for (retry = 0; retry < 2; retry++) {
        if (HAL_I2C_Mem_Read(h->hi2c, (uint16_t)(MT6701_I2C_ADDR << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len, MT6701_I2C_TIMEOUT) == HAL_OK) {
            return 1;
        }
        /* 偶发故障: 9 SCK 释放总线后重试 (限频, 避免持续故障拖死主循环) */
        now = HAL_GetTick();
        if ((now - h->last_recover_ms) >= 1000u) {
            h->last_recover_ms = now;
            MT6701_BusRecover(h->hi2c);
        } else {
            break;
        }
    }
    return 0;
}

/** 写寄存器: 写地址(0x0C)+寄存器+数据 */
static uint8_t mt6701_write_reg(MT6701_HandleTypeDef *h, uint8_t reg, uint8_t val) {
    uint8_t retry;
    uint32_t now;
    for (retry = 0; retry < 2; retry++) {
        if (HAL_I2C_Mem_Write(h->hi2c, (uint16_t)(MT6701_I2C_ADDR << 1), reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1, MT6701_I2C_TIMEOUT) == HAL_OK) {
            return 1;
        }
        now = HAL_GetTick();
        if ((now - h->last_recover_ms) >= 1000u) {
            h->last_recover_ms = now;
            MT6701_BusRecover(h->hi2c);
        } else {
            break;
        }
    }
    return 0;
}

/* ========================================================================= */
/*  公开 API                                                                  */
/* ========================================================================= */

int MT6701_Init(MT6701_HandleTypeDef *h, I2C_HandleTypeDef *hi2c) {
    h->hi2c     = hi2c;
    h->inited   = 0;
    h->last_angle = 0.0f;
    h->turn_cnt   = 0;
    h->is_first   = 1;
    h->last_raw   = 0xFFFFu;   /* 未初始化标记: 首次读数直接接受 */
    h->mag_ok     = 1;   /* 默认认为磁铁正常, 首次读取后校准 */
    h->last_recover_ms = 0;

    /* 总线恢复: 设备热插拔可能导致 I2C 外设 BUSY 卡死, 9 脉冲 + STOP 清除 */
    MT6701_BusRecover(hi2c);

    /* 验证器件存在: 地址 0x06 应答 (实测部分版本 WHOAMI(0x0F) 返回 0x00, 不可用作校验) */
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(MT6701_I2C_ADDR << 1), 3, 10) != HAL_OK)
        return -1;

    h->inited = 1;
    return 0;
}

uint16_t MT6701_ReadRaw(MT6701_HandleTypeDef *h) {
    uint8_t buf[2];
    uint16_t raw, delta;
    if (!h->inited) return 0xFFFFu;
    if (!mt6701_read_reg(h, MT6701_REG_ANGLE_MSB, buf, 2)) {
        /* 读失败: 返回哨兵 0xFFFF (非法值, 调用方据此判定读失败).
           注意: 绝不能返回 last_raw —— 闭环中冻结读数会让 PID
           在编码器失效时继续满速驱动电机 (电机空转/绕线). */
        return 0xFFFFu;
    }

    /* MT6701 14-bit 寄存器拼合：[0x03]: D13~D6, [0x04]: D5~D0 在高 6 位 */
    raw = ((uint16_t)buf[0] << 6) | (buf[1] >> 2);
    raw &= 0x3FFF;

    /* 跳变校验: I2C 偶发故障可能读到脏数据 (角度突变).
       计算与上次值的带环绕最短差, 超过阈值视为脏读数, 丢弃并保持上次值.
       阈值 = 30° / 0.022° per LSB ≈ 1365 LSB (正常运动 5ms 采样 < 6°, 安全裕量).
       首读 (last_raw 未初始化 = 0xFFFF) 直接接受.
       注意: 脏读数也返回哨兵 0xFFFF, 由调用方决定本次更新是否使用 */
    if (h->last_raw != 0xFFFFu) {
        delta = (uint16_t)((raw > h->last_raw) ? (raw - h->last_raw) : (h->last_raw - raw));
        if (delta > MT6701_CPR / 2u) {
            delta = (uint16_t)(MT6701_CPR - delta);   /* 环绕最短差 */
        }
        if (delta > 1365u) {
            return 0xFFFFu;                           /* 脏读数: 本次无效 */
        }
    }

    h->last_raw = raw;
    return raw;
}

float MT6701_ReadDegrees(MT6701_HandleTypeDef *h) {
    /* 读失败返回哨兵 raw 0xFFFF → 719.8° (非法值, 调用方检测) */
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
    if (!mt6701_read_reg(h, MT6701_REG_STATUS, &status, 1)) {
        /* I2C 偶发读失败: 保持上次判定结果, 避免偶发故障误触发磁检异常 */
        return h->mag_ok;
    }
    h->mag_ok = ((status & 0x01) == 0) ? 1 : 0;
    return h->mag_ok;
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