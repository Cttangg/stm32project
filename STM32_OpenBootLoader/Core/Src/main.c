/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Open Bootloader entry point (STM32F407VET6)
  ******************************************************************************
  * Boot flow:
  *   1. If the application requested a bootloader entry (magic flag in RAM
  *      written by APP_JumpToBootloader()): wait for the host forever.
  *   2. Else if a valid application exists at 0x08008000: wait up to
  *      OPENBL_HOST_WAIT_MS for a host sync byte, then jump to the app.
  *   3. Else (no application): wait for the host forever.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "openbl_core.h"
#include "openbl_usart_cmd.h"
#include "openbl_mem.h"
#include "usart_interface.h"
#include "flash_interface.h"
#include "ram_interface.h"
#include "optionbytes_interface.h"
#include "openbootloader_conf.h"
#include "app_boot_interface.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OPENBL_HOST_WAIT_MS              1000U  /* host sync window before jumping to the app */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static OPENBL_OpsTypeDef Usart_Ops =
{
  OPENBL_USART_Configuration,
  OPENBL_USART_DeInit,
  OPENBL_USART_ProtocolDetection,
  OPENBL_USART_GetCommandOpcode,
  OPENBL_USART_SendByte
};

static OPENBL_HandleTypeDef Usart_Interface = { &Usart_Ops, NULL };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t OPENBL_CheckAppValidity(void);
static uint8_t OPENBL_BootRequestCheckAndClear(void);
static uint8_t OPENBL_WaitHostForMs(uint32_t Ms);
static void OPENBL_HostLoop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Check whether a valid application is present at APP_BASE_ADDRESS.
  * @retval 1 if valid, else 0.
  */
static uint8_t OPENBL_CheckAppValidity(void)
{
  uint32_t app_sp    = *(volatile uint32_t *)APP_BASE_ADDRESS;
  uint32_t app_reset = *(volatile uint32_t *)(APP_BASE_ADDRESS + 4U);

  if ((app_sp >= RAM_START_ADDRESS) && (app_sp < RAM_END_ADDRESS) &&
      (app_reset >= FLASH_APP_START_ADDRESS) && (app_reset < FLASH_END_ADDRESS))
  {
    return 1U;
  }

  return 0U;
}

/**
  * @brief  Check and clear the "enter bootloader" request flag left in RAM
  *         by the application (APP_JumpToBootloader).
  * @retval 1 if the flag was set (bootloader must stay), else 0.
  */
static uint8_t OPENBL_BootRequestCheckAndClear(void)
{
  volatile uint32_t *flag = (volatile uint32_t *)APP_BOOT_FLAG_ADDR;

  if (*flag == APP_BOOT_MAGIC)
  {
    *flag = 0U;
    return 1U;
  }

  return 0U;
}

/**
  * @brief  Poll the USART interface for a host sync byte during Ms milliseconds.
  * @param  Ms Window length in milliseconds.
  * @retval 1 if the host was detected, else 0.
  */
static uint8_t OPENBL_WaitHostForMs(uint32_t Ms)
{
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < Ms)
  {
    if (OPENBL_InterfaceDetection() == 1U)
    {
      return 1U;
    }
  }

  return 0U;
}

/**
  * @brief  Wait for host commands forever.
  * @retval None.
  */
static void OPENBL_HostLoop(void)
{
  while (1U)
  {
    if (OPENBL_InterfaceDetection() == 1U)
    {
      OPENBL_CommandProcess();
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
  /* USER CODE BEGIN 2 */

  /* Register the USART interface and the supported memories */
  Usart_Interface.p_Cmd = OPENBL_USART_GetCommandsList();
  OPENBL_RegisterInterface(&Usart_Interface);

  OPENBL_MEM_RegisterMemory(&FLASH_Descriptor);
  OPENBL_MEM_RegisterMemory(&RAM_Descriptor);
  OPENBL_MEM_RegisterMemory(&OB_Descriptor);

  /* Initialize the Open Bootloader interfaces (starts the USART) */
  OPENBL_Init();

  /* Boot flow (see header comment) */
  if (OPENBL_BootRequestCheckAndClear() == 1U)
  {
    /* The application asked to enter the bootloader */
    OPENBL_HostLoop();
  }
  else if (OPENBL_CheckAppValidity() == 1U)
  {
    /* Valid application found: give the host a short window to take control */
    if (OPENBL_WaitHostForMs(OPENBL_HOST_WAIT_MS) == 1U)
    {
      OPENBL_HostLoop();
    }
    else
    {
      OPENBL_MEM_JumpToAddress(APP_BASE_ADDRESS);
    }
  }
  else
  {
    /* No application found: wait for the host forever */
    OPENBL_HostLoop();
  }

  /* USER CODE END 2 */

  /* Infinite loop (normally never reached) */
  while (1)
  {
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable (USART1 PA9/PA10 are configured by the OpenBL
     USART interface itself) */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
