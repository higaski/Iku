/***************************************************************************//**
 *  @brief      Encoder
 *  @file       encoder.c
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 ******************************************************************************/

/* ST/GCC includes */
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Project includes */
#include "dma_irq_priority.h"
#include "Diag/trace.h"
#include "Periph/encoder.h"
#include "Periph/pwm.h"
#include "System/sys.h"

/* Variables */
TIM_HandleTypeDef htim_encoder;                                                 //!< TIM handle for TIM_ENCODER
TIM_HandleTypeDef htim_encoder_read;                                            //!< TIM handle for TIM_ENCODER_READ

/** @brief  sgn                                                                 <br>
 */
__STATIC_INLINE int sgn(int32_t x)
{
  if (x > 0)
    return 1;
  else if (x < 0)
    return -1;
  else
    return 0;
}

/** @brief  Encoder initialization                                              <br>
 *  @return HAL status
 */
HAL_StatusTypeDef encoderInit()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  /* TIM2 GPIO Configuration
   * PA0-WKUP   ------> TIM2_CH1
   * PA1   ------> TIM2_CH2
   */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral TIM_ENCODER init */
  htim_encoder.Instance = TIM_ENCODER;
  htim_encoder.Init.Prescaler = 0;
  htim_encoder.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_encoder.Init.Period = 0xFFFFFFFF;
  htim_encoder.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim_encoder, &sConfig) != HAL_OK)
    return HAL_ERROR;

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_encoder, &sMasterConfig);

  /* Start encoder */
  HAL_TIM_Encoder_Start(&htim_encoder, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  TIM_ENCODER->CNT = (uint32_t) MIN_FREQ;

  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(TIM_ENCODER_READ_IRQn, IRQ_PRIO_TIM_ENCODER_READ, 0);
  HAL_NVIC_EnableIRQ(TIM_ENCODER_READ_IRQn);

  /* Peripheral TIM_ENCODER_READ init */
  htim_encoder_read.Instance = TIM_ENCODER_READ;
  htim_encoder_read.Init.Prescaler = (uint32_t) ((SystemCoreClock / 2)
                                     / TIM_ENCODER_READ_CLK) - 1;
  htim_encoder_read.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_encoder_read.Init.Period = TIM_ENCODER_READ_PERIOD - 1;
  htim_encoder_read.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim_encoder_read);

  /* Enable update interrupt and timer */
  TIM_ENCODER_READ->DIER |= TIM_DIER_UIE;
  TIM_ENCODER_READ->CR1 |= TIM_CR1_CEN;

  return HAL_OK;
}

/** @brief  This function handles TIM_ENCODER_READ global interrupt             <br>
 *          (__weak version found in startup_stm32f405xx.s)
 */
void TIM_ENCODER_READ_IRQHandler()
{
  static uint32_t TIM_ENCODER_CNT1 = (uint32_t) MIN_FREQ;
  static int32_t cnt = 0;
  int32_t tmp;

  /* Update interrupt */
  if (TIM_ENCODER_READ->SR & TIM_SR_UIF)
  {
    /* Clear update interrupt flag */
    TIM_ENCODER_READ->SR &= ~TIM_SR_UIF;

    /* If current and last counter value are not equal */
    tmp = TIM_ENCODER->CNT - TIM_ENCODER_CNT1;
    if (tmp != 0)
    {
      if (++cnt > 3)
      {
        /* Cap the dynamic amount somehow */
        if(cnt >= 20)
          cnt = 20;

        /* Start adding dynamically */
        TIM_ENCODER->CNT += cnt*5*sgn(tmp);
      }
    }
    else
    {
      if(--cnt <= 0)
        cnt = 0;
    }

    /* Cap frequency on both ends */
    if (TIM_ENCODER->CNT < MIN_FREQ)
    {
      TIM_ENCODER->CNT = (uint32_t) MIN_FREQ;
      cnt = 0;
    }
    else if (TIM_ENCODER->CNT > MAX_FREQ)
    {
      TIM_ENCODER->CNT = (uint32_t) MAX_FREQ;
      cnt = 0;
    }

    ac.man.f_ref = TIM_ENCODER->CNT;
    TIM_ENCODER_CNT1 = TIM_ENCODER->CNT;
  }
}
