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
#include "mt6701.h"
#include "uart.h"
#include "tmc2209.h"
#include "motor_pid.h"
#include "motor_stepper.h"
#include <string.h>
#include <stdlib.h>
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
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
static uint32_t g_last_report = 0;
/* 电机 X 句柄 (全局, 供应用各模块使用) */
TMC2209_HandleTypeDef motor_x;
/* 位置闭环 PID */
MotorPID g_pid;
static uint32_t g_last_ctrl = 0;   /* PID 更新节拍 */
/* 串口命令行缓冲 */
#define CMD_BUF_SIZE 64
static char     g_cmd_buf[CMD_BUF_SIZE];
static uint16_t g_cmd_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void send_str(const char *s) {
    uint16_t w;
    UART_Send(UART_P1, (const uint8_t *)s, (uint16_t)strlen(s), &w);
}

static void send_putdec(uint32_t n) {
    char buf[12]; uint8_t i = 0;
    if (n == 0) { send_str("0"); return; }
    while (n) { buf[i++] = (char)('0' + (n % 10)); n /= 10; }
    for (uint8_t j = 0; j < i / 2; j++) {
        char t = buf[j]; buf[j] = buf[i - 1 - j]; buf[i - 1 - j] = t;
    }
    buf[i] = 0;
    send_str(buf);
}

/* 输出浮点数, 2 位小数, 带符号, 如 "-12.34" */
static void send_putfloat(float v) {
    uint32_t ip, dp;
    if (v < 0.0f) { send_str("-"); v = -v; }
    ip = (uint32_t)v;
    dp = (uint32_t)((v - (float)ip) * 100.0f + 0.5f);
    if (dp >= 100) { dp -= 100; ip++; }
    send_putdec(ip);
    send_str(".");
    if (dp < 10) send_str("0");
    send_putdec(dp);
}

/* 输出 0~360° 一位小数, 如 "117.3" */
static void send_putdeg(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f)    deg += 360.0f;
    send_putfloat(deg);
}

/* ── 串口命令行 (Linux tty 风格) ── */
static void send_line(const char *s) {
    send_str(s);
    send_str("\r\n");
}

/* 执行一行命令 */
static void motor_cmd_process(char *line) {
    char *cmd = line, *arg;
    long v;

    while (*cmd == ' ' || *cmd == '\t') cmd++;
    arg = cmd;
    while (*arg && *arg != ' ' && *arg != '\t') arg++;
    if (*arg) { *arg = 0; arg++; }
    while (*arg == ' ' || *arg == '\t') arg++;

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        send_line("cmds:");
        send_line("  enable 0/1 | ms 8/16/32/64 | dir 0/1 | step N");
        send_line("  pos <deg 0-360>      set PID target & lock");
        send_line("  pid on/off           enable/disable PID");
        send_line("  pkp <v> pki <v> pkd <v>   online tune gains");
        return;
    }
    if (strcmp(cmd, "enable") == 0) {
        v = strtol(arg, NULL, 10);
        if (v) { TMC2209_Enable(&motor_x); }
        else { MotorStepper_Stop(); TMC2209_Disable(&motor_x); }
        send_str("OK: enabled="); send_putdec(motor_x.enabled); send_line("");
        return;
    }
    if (strcmp(cmd, "ms") == 0) {
        int old_ms = 8, new_ms = 8;
        v = strtol(arg, NULL, 10);
        switch (motor_x.microstep) {   /* 当前细分 → 数值 */
            case TMC2209_MICROSTEP_16: old_ms = 16; break;
            case TMC2209_MICROSTEP_32: old_ms = 32; break;
            case TMC2209_MICROSTEP_64: old_ms = 64; break;
            default: old_ms = 8; break;
        }
        switch (v) {
            case 8:  TMC2209_SetMicrostep(&motor_x, TMC2209_MICROSTEP_8);  new_ms = 8;  break;
            case 16: TMC2209_SetMicrostep(&motor_x, TMC2209_MICROSTEP_16); new_ms = 16; break;
            case 32: TMC2209_SetMicrostep(&motor_x, TMC2209_MICROSTEP_32); new_ms = 32; break;
            case 64: TMC2209_SetMicrostep(&motor_x, TMC2209_MICROSTEP_64); new_ms = 64; break;
            default: send_line("ERR: ms must be 8/16/32/64"); return;
        }
        /* 细分变化 → PID 重标定 (速度/增益按比例, 物理行为不变) */
        if (new_ms != old_ms)
            MotorPID_RescaleMicrostep(&g_pid, (float)old_ms, (float)new_ms);
        send_str("OK: microstep="); send_putdec((uint32_t)v); send_line("");
        return;
    }
    if (strcmp(cmd, "dir") == 0) {
        v = strtol(arg, NULL, 10);
        if (v != 0 && v != 1) { send_line("ERR: dir must be 0/1"); return; }
        TMC2209_SetDirection(&motor_x, (TMC2209_Dir)v);
        send_str("OK: dir="); send_putdec(motor_x.dir); send_line("");
        return;
    }
    if (strcmp(cmd, "step") == 0) {
        v = strtol(arg, NULL, 10);
        if (v <= 0) { send_line("ERR: step must be > 0"); return; }
        if (g_pid.enabled) { send_line("ERR: pid active, 'pid off' first"); return; }
        if (!motor_x.enabled) send_line("WARN: motor disabled, send 'enable 1' to make it move");
        MotorStepper_MoveSteps((uint32_t)v, 500);   /* 非阻塞, 500 steps/s */
        send_str("OK: moving "); send_putdec((uint32_t)v); send_line(" steps");
        return;
    }
    if (strcmp(cmd, "pos") == 0) {
        float target = strtof(arg, NULL);
        if (target < 0.0f || target >= 360.0f) { send_line("ERR: pos must be 0~359.9"); return; }
        TMC2209_Enable(&motor_x);
        MotorPID_SetTarget(&g_pid, target);
        send_str("OK: target="); send_putdeg(g_pid.target); send_line("");
        return;
    }
    if (strcmp(cmd, "pid") == 0) {
        if (strcmp(arg, "off") == 0) {
            MotorPID_Disable(&g_pid);
            MotorStepper_Stop();
            send_line("OK: pid off");
            return;
        }
        if (strcmp(arg, "on") == 0) {
            MotorPID_SetTarget(&g_pid, g_pid.target);   /* 干净重启用, 无 kick */
            TMC2209_Enable(&motor_x);
            send_line("OK: pid on");
            return;
        }
        send_str("  kp="); send_putfloat(g_pid.kp);
        send_str(" ki="); send_putfloat(g_pid.ki);
        send_str(" kd="); send_putfloat(g_pid.kd);
        send_str(" en="); send_putdec(g_pid.enabled); send_line("");
        return;
    }
    if (strcmp(cmd, "pkp") == 0) {
        g_pid.kp = strtof(arg, NULL);
        send_str("OK: kp="); send_putfloat(g_pid.kp); send_line("");
        return;
    }
    if (strcmp(cmd, "pki") == 0) {
        g_pid.ki = strtof(arg, NULL);
        send_str("OK: ki="); send_putfloat(g_pid.ki); send_line("");
        return;
    }
    if (strcmp(cmd, "pkd") == 0) {
        g_pid.kd = strtof(arg, NULL);
        send_str("OK: kd="); send_putfloat(g_pid.kd); send_line("");
        return;
    }
    send_line("ERR: unknown command (help)");
}

/* 逐字节接收: 累积到换行后执行命令 */
static void cmd_rx_cb(UART_Port port, uint8_t *data, uint16_t len) {
    (void)port;
    while (len--) {
        char c = (char)*data++;
        if (c == '\n' || c == '\r') {
            if (g_cmd_len > 0) {
                g_cmd_buf[g_cmd_len] = 0;
                g_cmd_len = 0;
                motor_cmd_process(g_cmd_buf);
                send_str("> ");   /* tty 提示符 */
            }
        } else if (g_cmd_len < CMD_BUF_SIZE - 1) {
            g_cmd_buf[g_cmd_len++] = c;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* 通用串口库: 基于 CubeMX 的 HAL_UART + DMA 环形接收 */
  UART_Init();
  if (UART_Open(UART_P1) != UART_OK) {
      Error_Handler();
  }

  send_str("\r\n=== 42Motor Angle (STM32F103) ===\r\n");

  send_str("MT6701: ");
  {
      int ok = 0, try;
      for (try = 0; try < 3; try++) {   /* 热插拔可能导致 I2C BUSY, 重试 */
          if (MT6701_Init() == 0) { ok = 1; break; }
          HAL_Delay(100);
      }
      if (!ok) {
          send_str("FAILED (no sensor?)\r\n");
          while (1) { UART_Task(); }
      }
  }
  send_str("OK\r\n");

  /* 注册电机 X 实例: 绑定 CubeMX 生成的 GPIO (MX_GPIO_Init 已配好引脚/时钟) */
  TMC2209_Init(&motor_x, &(TMC2209_PinConfig){
      .en   = {EN_GPIO_Port,   EN_Pin},
      .step = {STEP_GPIO_Port, STEP_Pin},
      .dir  = {DIR_GPIO_Port,  DIR_Pin},
      .ms1  = {MS1_GPIO_Port,  MS1_Pin},
      .ms2  = {MS2_GPIO_Port,  MS2_Pin},
  });
  send_str("MOTOR_X: registered\r\n");

  /* STEP 脉冲引擎: TIM3 硬件定时器非阻塞产生脉冲 */
  MotorStepper_Init(&motor_x, &htim3);

  /* 注册串口命令帧 (逐字节, 换行触发) */
  static UART_Frame cmd_frame = {
      .header_len = 0,
      .callback   = cmd_rx_cb,
  };
  UART_RegisterFrame(UART_P1, &cmd_frame);

  send_line("--- motor debug shell ---");
  send_line("> enable 0/1 | ms 8/16/32/64 | dir 0/1 | step N | pos <deg> | pid | pkp/pki/pkd | help");
  send_str("> ");

  /* 位置闭环 PID 初始化 */
  MotorPID_Init(&g_pid);
  g_last_ctrl = HAL_GetTick();
  g_last_report = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    UART_Task();

    /* 5ms PID 更新: 读角度 → PID → 更新速度参考.
       STEP 脉冲由硬件定时器独立生成, 此处只更新定时器频率/方向, 不阻塞 */
    if (HAL_GetTick() - g_last_ctrl >= 5) {
        uint32_t dt_ms = HAL_GetTick() - g_last_ctrl;
        g_last_ctrl = HAL_GetTick();
        MotorPID_Update(&g_pid, MT6701_ReadDegrees(), (float)dt_ms * 0.001f);
        MotorStepper_SetVelocity(g_pid.vel_ref);
    }

    /* 每 10ms 输出 VOFA+ firewater:
       angle,target,error,vel_cmd,vel_ref,d_term,i_term */
    if (HAL_GetTick() - g_last_report >= 10) {
        g_last_report = HAL_GetTick();
        send_putfloat(MT6701_ReadDegrees());
        send_str(",");
        send_putfloat(g_pid.target);
        send_str(",");
        send_putfloat(MotorPID_GetError(&g_pid));
        send_str(",");
        send_putfloat(g_pid.vel_cmd);
        send_str(",");
        send_putfloat(g_pid.vel_ref);
        send_str(",");
        send_putfloat(g_pid.d_term);
        send_str(",");
        send_putfloat(g_pid.i_term);
        send_str("\r\n");
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_Pin|DIR_Pin|STEP_Pin|MS2_Pin
                          |MS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_Pin DIR_Pin STEP_Pin MS2_Pin
                           MS1_Pin */
  GPIO_InitStruct.Pin = EN_Pin|DIR_Pin|STEP_Pin|MS2_Pin
                          |MS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
