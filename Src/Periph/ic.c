/***************************************************************************//**
 *  @brief      Input capture
 *  @file       ic.c
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       23/08/2015
 ******************************************************************************/

/* ST/GCC includes */
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Project includes */
#include "dma_irq_priority.h"
#include "Periph/ic.h"
#include "Periph/pwm.h"
#include "System/sys.h"

/* Variables */
TIM_HandleTypeDef htim_ic;                                                      //!< TIM handle for input capture timer
struct InputCapture ic;                                                         //!< Declaration of struct for input capture

/** @brief  Input capture initialization
 *  @return HAL status
 */
HAL_StatusTypeDef icInit()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_IC_InitTypeDef sConfigIC;

  /* TIM4 GPIO Configuration
   * PB6     ------> TIM4_CH1
   */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(TIM_IC_IRQn, IRQ_PRIO_TIM_IC, 0);
  HAL_NVIC_EnableIRQ(TIM_IC_IRQn);

  /* Peripheral TIM_IC init */
  htim_ic.Instance = TIM_IC;
  htim_ic.Init.Prescaler = (uint32_t) ((SystemCoreClock / 2) / TIM_IC_CLK) - 1;
  htim_ic.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_ic.Init.Period = 0xFFFF;
  htim_ic.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim_ic) != HAL_OK)
    return HAL_ERROR;

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim_ic, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim_ic);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&htim_ic, &sSlaveConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim_ic, &sConfigIC, TIM_CHANNEL_1);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  HAL_TIM_IC_ConfigChannel(&htim_ic, &sConfigIC, TIM_CHANNEL_2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_ic, &sMasterConfig);

  /* Enable update and capture&compare interrupt */
  TIM_IC->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE;
  /* Enable channel 1 and 2 */
  TIM_IC->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
  /* Enable timer */
  TIM_IC->CR1 |= TIM_CR1_CEN;

  return HAL_OK;
}

/** @brief  This function handles TIM_IC global interrupt
 *          (__weak version found in startup_stm32f405xx.s)
 */
void TIM_IC_IRQHandler()
{
  static uint32_t zero_100_cnt = 0, one_99_cnt = 0;
  static uint16_t cmp0 = 0, cmp1 = 0;

  /* Update interrupt */
  if (TIM_IC->SR & TIM_SR_UIF)
  {
    /* Clear update interrupt flag */
    TIM_IC->SR &= ~TIM_SR_UIF;

    /* Cover cases with dutycycle 0 and 100% */
    if (++zero_100_cnt > IC_CNT)
    {
      zero_100_cnt = 0;
      if (GPIOB->IDR & GPIO_PIN_6)
        ic.dutyc = 0;
      else
        ic.dutyc = 100;
    }
  }

  /* Capture and compare interrupt */
  if (TIM_IC->SR & TIM_SR_CC1IF)
  {
    /* Clear capture and compare interrupt flag */
    TIM_IC->SR &= ~TIM_SR_CC1IF;

    zero_100_cnt = 0;
    ic.freq = TIM_IC_CLK / TIM_IC->CCR1;
    cmp0 = (TIM_IC->CCR2 * 100) / TIM_IC->CCR1;

    /* Only accept dutycycle if IC_CNT measurements are equal */
    if (cmp0 == cmp1)
    {
      if (++one_99_cnt > IC_CNT)
      {
        one_99_cnt = 0;
        ic.dutyc = cmp0;
      }
    }
    else
      one_99_cnt = 0;

    /* Propagate dutycycle */
    cmp1 = cmp0;
  }

  /* Calculate PWM from dutycycle */
  if (ic.dutyc > 0)
  {
    ac.pwm.f_ref = (uint16_t) (MIN_FREQ
                   + ((MAX_FREQ - MIN_FREQ) / (100.0 - 1.0))
                   * (ic.dutyc - 1));
    ac.pwm.is_on = true;
  }
  else
  {
    ac.pwm.f_ref = MIN_FREQ;
    ac.pwm.is_on = false;
  }
}
