/***************************************************************************//**
 *  @brief      System management
 *  @file       sys.c
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       06/10/2015
 ******************************************************************************/

/* ST/GCC includes */
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>

/* Project includes */
#include "dma_irq_priority.h"
#include "Diag/dwt_cyccnt.h"
#include "Diag/trace.h"
#include "Periph/adc.h"
#include "Periph/debounce.h"
#include "Periph/gpio.h"
#include "Periph/hd44780.h"
#include "Periph/encoder.h"
#include "Periph/ic.h"
#include "Periph/pwm.h"
#include "System/sys.h"

/* Variables */
TIM_HandleTypeDef htim_state_machine;                                           //!< TIM handle for TIM_STATE_MACHINE
struct Motor ac;                                                                //!< Struct for motor

/** @brief  Enable all clocks                                                   <br>
 */
static void enable_all_clks()
{
  /* GPIO Ports Clock Enable */
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();
  __DMA2_CLK_ENABLE();

  /* Peripheral clocks enable */
  __ADC_VBUS_CLK_ENABLE();
  __ADC_I_CLK_ENABLE();
  __TIM_DEBOUNCE_CLK_ENABLE();
  __TIM_ENCODER_CLK_ENABLE();
  __TIM_ENCODER_READ_CLK_ENABLE();
  __TIM_HD44780_CLK_ENABLE();
  __TIM_HD44780_TASK_CLK_ENABLE();
  __TIM_IC_CLK_ENABLE();
  __TIM_ACC_CLK_ENABLE();
  __TIM_PWM_CLK_ENABLE();
  __TIM_STATE_MACHINE_CLK_ENABLE();
}

/** @brief  State machine init                                                  <br>
 *  @return HAL status
 */
static HAL_StatusTypeDef stateMachineInit()
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(TIM_STATE_MACHINE_IRQn, IRQ_PRIO_TIM_STATE_MACHINE, 0);
  HAL_NVIC_EnableIRQ(TIM_STATE_MACHINE_IRQn);

  /* Peripheral TIM_STATE_MACHINE init */
  htim_state_machine.Instance = TIM_STATE_MACHINE;
  htim_state_machine.Init.Prescaler = (uint32_t) ((SystemCoreClock / 2)
                                      / TIM_STATE_MACHINE_CLK) - 1;
  htim_state_machine.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_state_machine.Init.Period = TIM_STATE_MACHINE_PERIOD - 1;
  htim_state_machine.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim_state_machine) != HAL_OK)
    return HAL_ERROR;

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim_state_machine, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_state_machine, &sMasterConfig);

  return HAL_OK;
}

/** @brief  System initialization                                               <br>
 *  @return HAL status
 */
HAL_StatusTypeDef sys_init()
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* 4 bits for pre-emption priority, 0 bits for subpriority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, IRQ_PRIO_SYSTICK, 0);

  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, IRQ_PRIO_MEM_MANAGEMENT, 0);

  /* Enable clocks */
  enable_all_clks();

  /* Global GPIO initialization */
  gpioInit();
  /* LEDs off */
  YELLOW_LED_OFF();
  RED_LED_OFF();

  /* ADC initialization */
  if (adcInit() != HAL_OK)
    return HAL_ERROR;

  /* Acceleration timer initialization */
  if (accInit() != HAL_OK)
    return HAL_ERROR;

  /* Wait for the capacitors (3mF) to charge */
  HAL_Delay(2000);

  /* PWM initialization */
  if (pwmInit() != HAL_OK)
    return HAL_ERROR;
  memset(&ac, 0, sizeof(ac));                                             // Flush motor struct

  /* Encoder interface initialization */
  if (encoderInit() != HAL_OK)
    return HAL_ERROR;

  /* HD44780 initialization */
  if (hd44780Init() != HAL_OK)
    return HAL_ERROR;
  if (hd44780TaskInit() != HAL_OK)
    return HAL_ERROR;

  /* Debounce timer initialization */
  if (debounceInit() != HAL_OK)
    return HAL_ERROR;

  /* Input capture initialization */
  if (icInit() != HAL_OK)
    return HAL_ERROR;

  /* State machine initialization */
  if (stateMachineInit() != HAL_OK)
    return HAL_ERROR;

  /* PWM start */
  pwmStartIT();

  return HAL_OK;
}

/** @brief  System Clock Configuration
 */
void SystemClock_Config()
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 210;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
                                | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/** @brief  This function handles SysTick                                       <br>
 *          (__weak version found in startup_stm32f405xx.s)
 */
void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/** @brief  This function handles Memory management fault.                      <br>
 *          (__weak version found in startup_stm32f405xx.s)
 */
void MemManage_Handler()
{
  while (1)
  {
  }
}
