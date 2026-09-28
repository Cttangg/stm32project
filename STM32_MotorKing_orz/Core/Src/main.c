/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "mt6701.h"
#include "tmc2209.h"
#include "motor_pid.h"
#include "motor_stepper.h"
#include "uart.h"
#include "axis.h"
#include "safety.h"
#include "motion.h"
#include "app_boot_interface.h"
#include <string.h>
#include <stdio.h>
/* Phase 4-8 (暂时注释, 后续按流水线解锁):
#include "communication.h"
#include "tracking_manager.h"
#include "app.h"
#include "telemetry.h"
*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* 电机1 = Tilt (竖直轴, TIM14 + I2C1 + EN_1/DIR_1/STEP_1/MS1_1/MS2_1) */
static MT6701_HandleTypeDef enc_tilt;
static TMC2209_HandleTypeDef motor_tilt;
MotorStepper stepper_tilt;      /* 全局: stm32f4xx_it.c 中断分发需要 */
static MotorPID pid_tilt;

/* 电机2 = Pan (水平轴, TIM13 + I2C2 + EN_2/DIR_2/STEP_2/MS1_2/MS2_2) */
static MT6701_HandleTypeDef enc_pan;
static TMC2209_HandleTypeDef motor_pan;
MotorStepper stepper_pan;       /* 全局: stm32f4xx_it.c 中断分发需要 */
static MotorPID pid_pan;

/* 串口库设备: USART1 调试口, USART2 摄像头 */
static UART_Device uart_dbg, uart_cam;

/* 单轴抽象: 统一封装 PID/驱动/编码器/步进引擎 (Pan/Tilt 共用同一结构) */
static Axis axis_tilt;
static Axis axis_pan;

/* 安全管理器 */
static SafetyConfig safety_cfg;

/* 运动管理器 */
static MotionConfig motion_cfg;

/* Phase 4-8 (暂时注释):
static TrackingConfig tracking_cfg;
static AppConfig app_cfg;
static uint32_t telemetry_tick;
*/

/* 调试命令 (Phase 3 验收驱动): go <pan> <tilt> / stop / swing <amp> */
static UART_Frame  dbg_cmd_frame;
static uint8_t     dbg_cmd_buf[32];
static uint16_t    dbg_cmd_len;
static char        dbg_last_cmd[33];   /* 诊断: 最近完整命令 */

/* 摆动测试状态: 以启动时的位置为原点, 小幅摆动 ±amp 度 (仅 Tilt) */
static uint8_t  swing_active;
static float    swing_amp;
static float    swing_tilt_origin;
static int      swing_dir;         /* +1 / -1 */
static uint32_t swing_next_ms;
static uint32_t swing_period_ms;   /* 每个方向停留时间 */

/* 开环走圈测试状态 (spin): MoveSteps 走 1 圈, 周期采样编码器角度, 验证回正 */
static uint8_t  spin_active;
static float    spin_start_angle;
static float    spin_last_angle;
static uint32_t spin_t0;
static uint32_t spin_sample_next;

/* 编码器失效保护: 连续读失败计数 (裸 PID 模式, 超阈值停轴防空转) */
static uint8_t  enc_fail_pan;
static uint8_t  enc_fail_tilt;
#define ENC_FAIL_LIMIT  5   /* 连续 5 次失败 → 停轴 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
static void DbgCmd_OnByte(UART_Device *dev, uint8_t *data, uint16_t len);
static void DbgCmd_SwingUpdate(uint32_t now_ms);
static void DbgCmd_SpinUpdate(uint32_t now_ms);
static float DbgCmd_ParseFloat(const char *s, char **end);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* TIM 中断处理由 stm32f4xx_it.c 生成区接管 (CubeMX 已勾选 NVIC):
   TIM14 → stepper_tilt, TIM13 → stepper_pan, 分发见 stm32f4xx_it.c USER CODE 区 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  APP_SetVectorTable();   /* 必须为第一行: 重定向向量表到 App (0x08008000), 在 HAL_Init 之前 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  /* MT6701 编码器 (I2C1/I2C2, 400kHz) */
  if (MT6701_Init(&enc_tilt, &hi2c1) != 0) {
    /* 电机1 编码器未应答 */
  }
  if (MT6701_Init(&enc_pan, &hi2c2) != 0) {
    /* 电机2 编码器未应答 */
  }

  /* TMC2209 驱动板 (GPIO 由 MX_GPIO_Init 配置) */
  TMC2209_Init(&motor_tilt, &(TMC2209_PinConfig){
      .en   = {EN_1_GPIO_Port,   EN_1_Pin},
      .step = {STEP_1_GPIO_Port, STEP_1_Pin},
      .dir  = {DIR_1_GPIO_Port,  DIR_1_Pin},
      .ms1  = {MS1_1_GPIO_Port,  MS1_1_Pin},
      .ms2  = {MS2_1_GPIO_Port,  MS2_1_Pin},
  });
  TMC2209_Init(&motor_pan, &(TMC2209_PinConfig){
      .en   = {EN_2_GPIO_Port,   EN_2_Pin},
      .step = {STEP_2_GPIO_Port, STEP_2_Pin},
      .dir  = {DIR_2_GPIO_Port,  DIR_2_Pin},
      .ms1  = {MS1_2_GPIO_Port,  MS1_2_Pin},
      .ms2  = {MS2_2_GPIO_Port,  MS2_2_Pin},
  });

  /* STEP 脉冲引擎: Tilt→TIM14, Pan→TIM13 */
  MotorStepper_Init(&stepper_tilt, &motor_tilt, &htim14);
  MotorStepper_Init(&stepper_pan,  &motor_pan,  &htim13);

  /* 位置闭环 PID (默认 1/32 细分标定) */
  MotorPID_Init(&pid_tilt);
  MotorPID_Init(&pid_pan);
  /* Pan PID 整定 (实测 2026-09-29): kp=700 ki=100 kd=350
     正向过冲 0.17°, 无高频抖动, 精确锁定 ±0.04°
     (默认 kp=1500/kd=125 会导致 Pan 目标附近极限环振荡 ±2.5°) */
  MotorPID_SetGains(&pid_pan, 700.0f, 100.0f, 350.0f);

  /* 串口库: USART1 调试口 + USART2 摄像头 (RX DMA 需为 Circular) */
  UART_Init(&uart_dbg, &huart1);
  UART_Init(&uart_cam, &huart2);
  UART_Open(&uart_dbg);
  UART_Open(&uart_cam);

  /* 通信管理器: 摄像头视觉帧解析 → VisionTarget (Phase 4, 暂时注释)
  Communication_Init(&uart_cam); */

  /* 单轴抽象: 包装 编码器 + PID + 步进引擎 + 驱动
     (裸 PID 测试模式: 仅电机1/Tilt 使能, 电机2/Pan 保持禁用) */
  Axis_Init(&axis_tilt, &pid_tilt, &motor_tilt, &enc_tilt, &stepper_tilt);
  Axis_Init(&axis_pan,  &pid_pan,  &motor_pan,  &enc_pan,  &stepper_pan);

  /* 启动时目标 = 当前位置 (避免 PID 全速追默认 0° 导致乱转) */
  Axis_SetTarget(&axis_tilt, MT6701_ReadDegrees(&enc_tilt));
  Axis_SetTarget(&axis_pan,  MT6701_ReadDegrees(&enc_pan));

  /* 限位: 编码器绝对角度 0-360°, 先用全范围避免误触发 (物理行程限位后按机械标定) */
  Axis_SetLimits(&axis_tilt, 0.0f, 360.0f);
  Axis_SetLimits(&axis_pan,  0.0f, 360.0f);

  Axis_Enable(&axis_tilt);
  Axis_Enable(&axis_pan);

  /* 裸 PID 模式: 用 MotorPID_SetTarget 使能双轴 PID, 目标 = 当前位置 */
  MotorPID_SetTarget(&pid_tilt, MT6701_ReadDegrees(&enc_tilt));
  MotorPID_SetTarget(&pid_pan,  MT6701_ReadDegrees(&enc_pan));

  /* 安全: 绑定轴, 使能按键暂停 */
  safety_cfg.pan = &axis_pan;
  safety_cfg.tilt = &axis_tilt;
  safety_cfg.enable_key_pause = true;
  Safety_Init(&safety_cfg);

  /* 运动: 绑定 Pan/Tilt 轴 + 角度限位 (编码器坐标 0-360, 物理行程标定后收紧) */
  motion_cfg.pan = &axis_pan;
  motion_cfg.tilt = &axis_tilt;
  motion_cfg.pan_min  = 0.0f;
  motion_cfg.pan_max  = 360.0f;
  motion_cfg.tilt_min = 0.0f;
  motion_cfg.tilt_max = 360.0f;
  Motion_Init(&motion_cfg);

  /* 调试命令 (Phase 3 验收): go <pan> <tilt> / stop / swing <amp> */
  dbg_cmd_frame.header_len = 0;              /* 逐字节回调 */
  dbg_cmd_frame.max_len    = sizeof(dbg_cmd_buf);
  dbg_cmd_frame.get_length = NULL;
  dbg_cmd_frame.check      = NULL;
  dbg_cmd_frame.callback   = DbgCmd_OnByte;
  dbg_cmd_frame.timeout_ms = 0;
  dbg_cmd_len = 0;
  UART_RegisterFrame(&uart_dbg, &dbg_cmd_frame);

  /* 摆动测试初始关闭 */
  swing_active     = 0;
  swing_amp        = 5.0f;                   /* 默认 ±5° */
  swing_tilt_origin = 0.0f;
  swing_dir        = 1;
  swing_next_ms    = 0;
  swing_period_ms  = 1000;                   /* 每方向 1s */

  /* 编码器失效保护计数清零 */
  enc_fail_pan  = 0;
  enc_fail_tilt = 0;

  /* 开环走圈测试初始关闭 */
  spin_active      = 0;
  spin_start_angle = 0.0f;
  spin_last_angle  = 0.0f;
  spin_t0          = 0;
  spin_sample_next = 0;

  /* 上电自检标志: 验证 App 是否正常启动 (诊断用) */
  {
      static const uint8_t ready[] = "APP-READY\r\n";
      uint16_t w = 0;
      UART_Send(&uart_dbg, ready, sizeof(ready) - 1, &w);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 串口帧解析/回调 (必须周期调用) */
    UART_Task(&uart_dbg);

    /* Phase 4-8 (暂时注释):
    Communication_Update(HAL_GetTick());
    {
        VisionTarget vt;
        if (Communication_PopVisionTarget(&vt)) {
            Tracking_SetVisionTarget(&vt);
        }
    }
    Tracking_Update(HAL_GetTick());
    App_Update(HAL_GetTick());
    {
        uint32_t now = HAL_GetTick();
        if ((now - telemetry_tick) >= 20) {
            telemetry_tick = now;
            Telemetry_Sample(now);
        }
    }
    */

    /* ── PID 闭环测试模式: 双轴裸 PID (编码器失效保护) ── */
    DbgCmd_SwingUpdate(HAL_GetTick());   /* 摆动目标直接喂 PID (Tilt 单轴) */
    DbgCmd_SpinUpdate(HAL_GetTick());    /* 开环走圈 (与 PID 互斥, 由命令控制) */

    /* 位置闭环 5ms: 编码器 → MotorPID → MotorStepper (双轴裸 PID)
       spin 开环走圈期间暂停 PID, 避免 MoveSteps 与 SetVelocity 冲突
       编码器读失败 (0xFFFF 哨兵) 连续 ENC_FAIL_LIMIT 次 → 停轴, 防堵转/空转 */
    if (!spin_active) {
        float t_ang = MT6701_ReadDegrees(&enc_tilt);
        float p_ang = MT6701_ReadDegrees(&enc_pan);

        /* Tilt: 读失败则计数, 超阈值停轴 */
        if (t_ang > 360.0f) {
            if (++enc_fail_tilt >= ENC_FAIL_LIMIT) {
                MotorStepper_SetVelocity(&stepper_tilt, 0.0f);
            }
        } else {
            enc_fail_tilt = 0;
            MotorPID_Update(&pid_tilt, t_ang, 0.005f);
            MotorStepper_SetVelocity(&stepper_tilt, -pid_tilt.vel_ref);
        }

        /* Pan: 同上 */
        if (p_ang > 360.0f) {
            if (++enc_fail_pan >= ENC_FAIL_LIMIT) {
                MotorStepper_SetVelocity(&stepper_pan, 0.0f);
            }
        } else {
            enc_fail_pan = 0;
            MotorPID_Update(&pid_pan, p_ang, 0.005f);
            MotorStepper_SetVelocity(&stepper_pan, -pid_pan.vel_ref);
        }
    } else {
        MT6701_ReadDegrees(&enc_tilt);   /* spin 期间 I2C 只读 */
    }

    /* 电机2/Pan: 不调用任何 Update (驱动已 Disable) */

    /* 安全监控: 按键/轴故障/限位 (故障时阻止运动) */
    Safety_Update(HAL_GetTick());

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_DC_Pin|LCD_RES_Pin|DIR_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_BL_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DIR_2_Pin|STEP_2_Pin|MS2_2_Pin|MS1_2_Pin
                          |EN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, EN_1_Pin|MS1_1_Pin|MS2_1_BK_Pin|STEP_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_DC_Pin LCD_RES_Pin DIR_1_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_RES_Pin|DIR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_2_Pin STEP_2_Pin MS2_2_Pin MS1_2_Pin
                           EN_2_Pin */
  GPIO_InitStruct.Pin = DIR_2_Pin|STEP_2_Pin|MS2_2_Pin|MS1_2_Pin
                          |EN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_1_Pin MS1_1_Pin MS2_1_BK_Pin STEP_1_Pin */
  GPIO_InitStruct.Pin = EN_1_Pin|MS1_1_Pin|MS2_1_BK_Pin|STEP_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MS2_1_Pin */
  GPIO_InitStruct.Pin = MS2_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MS2_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_PAUSE_Pin KEY_RESET_Pin KEY_TRACK_Pin KEY_BORDER_Pin */
  GPIO_InitStruct.Pin = KEY_PAUSE_Pin|KEY_RESET_Pin|KEY_TRACK_Pin|KEY_BORDER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_CALIB_Pin KEY_A4_Pin */
  GPIO_InitStruct.Pin = KEY_CALIB_Pin|KEY_A4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief 调试命令逐字节回调 (Phase 3 验收驱动)
 *
 * 命令格式 (USART1 调试口, 以 \r 或 \n 结尾):
 *   go <pan> <tilt> → 双轴裸 PID 运动到指定角度
 *   stop            → 停止双轴 (保持当前位置)
 *   swing <amp>     → 以当前位置为原点, Tilt 小幅摆动 ±amp 度
 *   swing stop      → 停止摆动, 回原点
 *   spin            → Tilt 开环走 1 圈 (验证回正)
 *   pid <kp> <ki> <kd> → 在线调整 Pan PID 增益
 *   boot            → 跳回 Bootloader (等主机烧录)
 *   pos             → 打印当前 Pan/Tilt 位置
 *   例: "go 200 30\r"  "swing 5\r"  "stop\r"  "boot\r"  "pos\r"
 */
static void DbgCmd_OnByte(UART_Device *dev, uint8_t *data, uint16_t len)
{
    float pan, tilt, amp;
    char cmd[33];
    char *endp;
    (void)dev;

    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        if (b == '\r' || b == '\n') {
            if (dbg_cmd_len > 0) {
                dbg_cmd_buf[dbg_cmd_len] = '\0';
                cmd[0] = '\0';
                strncpy(dbg_last_cmd, (char *)dbg_cmd_buf, sizeof(dbg_last_cmd) - 1);
                dbg_last_cmd[sizeof(dbg_last_cmd) - 1] = '\0';
                if (sscanf((char *)dbg_cmd_buf, "%31s", cmd) == 1) {
                    if (strcmp(cmd, "go") == 0) {
                        /* 手动解析浮点, 双轴裸 PID */
                        endp = NULL;
                        pan  = DbgCmd_ParseFloat((char *)dbg_cmd_buf + 3, &endp);
                        if (endp) {
                            tilt = DbgCmd_ParseFloat(endp, &endp);
                            if (endp) {
                                swing_active = 0;              /* 手动目标覆盖摆动 */
                                MotorPID_SetTarget(&pid_pan,  pan);
                                MotorPID_SetTarget(&pid_tilt, tilt);
                                { uint8_t e[48]; int l = snprintf((char*)e, sizeof(e), "GO->PAN %.1f TILT %.1f\r\n", pan, tilt); uint16_t w; if (l>0) UART_Send(&uart_dbg,e,(uint16_t)l,&w); }
                            } else {
                                { uint8_t e[]="GO-TILT-FAIL\r\n"; uint16_t w; UART_Send(&uart_dbg,e,sizeof(e)-1,&w); }
                            }
                        } else {
                            { uint8_t e[]="GO-PAN-FAIL\r\n"; uint16_t w; UART_Send(&uart_dbg,e,sizeof(e)-1,&w); }
                        }
                    } else if (strcmp(cmd, "stop") == 0) {
                        swing_active = 0;
                        MotorPID_SetTarget(&pid_pan,  MT6701_ReadDegrees(&enc_pan));
                        MotorPID_SetTarget(&pid_tilt, MT6701_ReadDegrees(&enc_tilt));
                        MotorStepper_SetVelocity(&stepper_pan,  0.0f);
                        MotorStepper_SetVelocity(&stepper_tilt, 0.0f);
                    } else if (strcmp(cmd, "pid") == 0) {
                        /* 在线调整 Pan PID: pid <kp> <ki> <kd> (作用于 Pan 轴) */
                        {
                            float kp, ki, kd;
                            char *p = (char *)dbg_cmd_buf + 4;
                            kp = DbgCmd_ParseFloat(p, &endp);
                            if (endp) { ki = DbgCmd_ParseFloat(endp, &endp); }
                            if (endp) { kd = DbgCmd_ParseFloat(endp, &endp); }
                            if (endp && kp >= 0 && ki >= 0 && kd >= 0) {
                                MotorPID_SetGains(&pid_pan, kp, ki, kd);
                                { uint8_t e[48]; int l = snprintf((char*)e, sizeof(e),
                                      "PID-PAN kp=%.0f ki=%.0f kd=%.0f\r\n", kp, ki, kd);
                                  uint16_t w; if (l>0) UART_Send(&uart_dbg,e,(uint16_t)l,&w); }
                            } else {
                                { uint8_t e[]="PID-FMT: pid <kp> <ki> <kd>\r\n"; uint16_t w; UART_Send(&uart_dbg,e,sizeof(e)-1,&w); }
                            }
                        }
                    } else if (strcmp(cmd, "swing") == 0) {
                        /* 手动解析 amp */
                        endp = NULL;
                        amp  = DbgCmd_ParseFloat((char *)dbg_cmd_buf + 6, &endp);
                        if (endp) {
                            /* 启动摆动: 以 Tilt 当前位置为原点 */
                            swing_active      = 1;
                            swing_amp         = (amp < 1.0f) ? 1.0f : amp;
                            swing_tilt_origin = MT6701_ReadDegrees(&enc_tilt);
                            swing_dir         = 1;
                            swing_next_ms     = HAL_GetTick() + swing_period_ms;
                            MotorPID_SetTarget(&pid_tilt,
                                               swing_tilt_origin + swing_dir * swing_amp);
                        } else {
                            swing_active = 0;          /* "swing stop" */
                            MotorPID_SetTarget(&pid_tilt, swing_tilt_origin);
                        }
                    } else if (strcmp(cmd, "spin") == 0) {
                        /* 开环走 1 整圈 (6400 步 @ 2000 steps/s ≈ 3.2s), I2C 周期采样验证回正 */
                        swing_active = 0;
                        spin_active      = 1;
                        spin_start_angle = MT6701_ReadDegrees(&enc_tilt);
                        spin_last_angle  = spin_start_angle;
                        spin_t0          = HAL_GetTick();
                        spin_sample_next = spin_t0;
                        MotorStepper_MoveSteps(&stepper_tilt,
                                               (uint32_t)STEPS_PER_REV, 2000.0f);
                        { uint8_t e[48]; int l = snprintf((char*)e, sizeof(e),
                              "SPIN-START a=%.1f steps=%u\r\n",
                              spin_start_angle, (unsigned)STEPS_PER_REV);
                          uint16_t w; if (l>0) UART_Send(&uart_dbg,e,(uint16_t)l,&w); }
                    } else if (strcmp(cmd, "boot") == 0) {
                        /* 跳回 Bootloader 等待主机烧录 */
                        APP_JumpToBootloader();
                    } else if (strcmp(cmd, "pos") == 0) {
                        /* 打印双轴当前位置 (裸 PID 模式) */
                        {
                            uint8_t tmp[80];
                            int len = snprintf((char *)tmp, sizeof(tmp),
                                               "PAN=%.2f (tgt=%.2f) TILT=%.2f (tgt=%.2f)\r\n",
                                               MT6701_ReadDegrees(&enc_pan), pid_pan.target,
                                               MT6701_ReadDegrees(&enc_tilt), pid_tilt.target);
                            uint16_t w = 0;
                            if (len > 0) UART_Send(&uart_dbg, tmp, (uint16_t)len, &w);
                        }
                    } else if (strcmp(cmd, "fault") == 0) {
                        /* 读取上次 HardFault 现场 (诊断) */
                        {
                            volatile uint32_t *f = (volatile uint32_t *)0x2001FF00u;
                            uint8_t tmp[96];
                            int len;
                            if (f[6] == 0xA5A5A5A5u) {
                                len = snprintf((char *)tmp, sizeof(tmp),
                                    "FAULT MSP=%08x PSP=%08x RET=%08x LR=%08x\r\n",
                                    (unsigned)f[2], (unsigned)f[3],
                                    (unsigned)f[4], (unsigned)f[5]);
                            } else {
                                len = snprintf((char *)tmp, sizeof(tmp),
                                    "FAULT none\r\n");
                            }
                            uint16_t w = 0;
                            if (len > 0) UART_Send(&uart_dbg, tmp, (uint16_t)len, &w);
                        }
                    } else if (strcmp(cmd, "echo") == 0) {
                        /* 回显最近收到的命令 (诊断) */
                        {
                            uint8_t tmp[48];
                            int len = snprintf((char *)tmp, sizeof(tmp),
                                               "LAST[%s]\r\n", dbg_last_cmd);
                            uint16_t w = 0;
                            if (len > 0) UART_Send(&uart_dbg, tmp, (uint16_t)len, &w);
                        }
                    } else if (strcmp(cmd, "stat") == 0) {
                        /* 打印双轴 PID 状态 (诊断) */
                        {
                            uint8_t tmp[160];
                            int len = snprintf((char *)tmp, sizeof(tmp),
                                               "P:tgt=%.1f cur=%.1f vr=%.0f st=%d | T:tgt=%.1f cur=%.1f vr=%.0f st=%d\r\n",
                                               pid_pan.target,
                                               pid_pan.current,
                                               pid_pan.vel_ref,
                                               (int)pid_pan.state,
                                               pid_tilt.target,
                                               pid_tilt.current,
                                               pid_tilt.vel_ref,
                                               (int)pid_tilt.state);
                            uint16_t w = 0;
                            if (len > 0) UART_Send(&uart_dbg, tmp, (uint16_t)len, &w);
                        }
                    }
                }
                dbg_cmd_len = 0;
            }
        } else if (dbg_cmd_len < sizeof(dbg_cmd_buf) - 1) {
            dbg_cmd_buf[dbg_cmd_len++] = b;
        }
    }
}

/**
 * @brief 手动浮点解析 (兼容 nano.specs, 不依赖 strtof/scanf %f)
 * @param s  起始指针 (跳过前导空白)
 * @param end 输出: 解析结束位置 (NULL = 无有效数字)
 * @return 解析值
 */
static float DbgCmd_ParseFloat(const char *s, char **end)
{
    float sign = 1.0f, ipart = 0.0f, fpart = 0.0f, fdiv = 1.0f;
    const char *p = s;
    int found = 0;

    while (*p == ' ' || *p == '\t') p++;
    if (*p == '-') { sign = -1.0f; p++; }
    else if (*p == '+') { p++; }

    while (*p >= '0' && *p <= '9') { ipart = ipart * 10.0f + (*p - '0'); p++; found = 1; }
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') { fdiv *= 10.0f; fpart = fpart * 10.0f + (*p - '0'); p++; found = 1; }
    }

    if (end) {
        *end = found ? (char *)p : NULL;
    }
    return sign * (ipart + fpart / fdiv);
}

/**
 * @brief 摆动测试周期驱动 (主循环调用): 按周期切换方向
 *
 * 以启动时的位置为原点, Pan/Tilt 同幅摆动 ±swing_amp,
 * 用于验证: 双轴同步运动、限位、编码器反馈、I2C 总线自愈 (偶发故障下不丢步).
 */
static void DbgCmd_SwingUpdate(uint32_t now_ms)
{
    if (!swing_active) return;

    if ((now_ms - swing_next_ms) < 0x7FFFFFFFu &&
        now_ms >= swing_next_ms) {
        swing_dir = -swing_dir;
        swing_next_ms = now_ms + swing_period_ms;
        MotorPID_SetTarget(&pid_tilt,
                           swing_tilt_origin + swing_dir * swing_amp);
    }
}

/**
 * @brief 开环走圈测试状态机 (主循环调用)
 *
 * spin 命令触发 MoveSteps 走 1 圈 (6400 步 @ 2000 steps/s ≈ 3.2s),
 * 期间每 100ms 采样一次编码器角度上报, 走完后打印起始/结束角度差,
 * 用于验证: 电机机械转 1 圈 → 编码器读数回正 (回到起始角度).
 * I2C 只读, 不参与闭环.
 */
static void DbgCmd_SpinUpdate(uint32_t now_ms)
{
    uint8_t e[48];
    uint16_t w;
    int l;

    if (!spin_active) return;

    /* 周期采样上报 */
    if ((now_ms - spin_sample_next) >= 100u) {
        spin_sample_next = now_ms;
        spin_last_angle = MT6701_ReadDegrees(&enc_tilt);
        l = snprintf((char *)e, sizeof(e), "SPIN a=%.1f\r\n", spin_last_angle);
        if (l > 0) UART_Send(&uart_dbg, e, (uint16_t)l, &w);
    }

    /* 完成判定: 目标步数已清 (MoveSteps 到点自动停) 且启动超过 500ms */
    if (stepper_tilt.target_steps == 0 &&
        (now_ms - spin_t0) >= 500u) {
        spin_active = 0;
        spin_last_angle = MT6701_ReadDegrees(&enc_tilt);
        l = snprintf((char *)e, sizeof(e),
                     "SPIN-DONE start=%.1f end=%.1f\r\n",
                     spin_start_angle, spin_last_angle);
        if (l > 0) UART_Send(&uart_dbg, e, (uint16_t)l, &w);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
