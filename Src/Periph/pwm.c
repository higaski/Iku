/***************************************************************************//**
 *  @brief      PWM
 *  @file       pwm.c
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 ******************************************************************************/

/* ST/GCC includes */
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

/* Project includes */
#include "dma_irq_priority.h"
#include "Diag/dwt_cyccnt.h"
#include "Periph/adc.h"
#include "Periph/pwm.h"
#include "System/sys.h"

/* Variables */
TIM_HandleTypeDef htim_acc;                                                     //!< TIM handle for acceleration timer
TIM_HandleTypeDef htim_pwm;                                                     //!< TIM handle for PWM timer
struct SvPwm svm;                                                               //!< Struct for SVM algorithm

/** @brief  Acceleration timer initialization
 *  @return HAL status
 */
HAL_StatusTypeDef accInit()
{
  /* Peripheral TIM_ACC init */
  htim_acc.Instance = TIM_ACC;
  htim_acc.Init.Prescaler = (uint32_t) ((SystemCoreClock / 2) / TIM_ACC_CLK)
      - 1;    // 1 MHz;
  htim_acc.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_acc.Init.Period = ACC_PERIOD - 1;
  htim_acc.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim_acc);

  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(TIM_ACC_IRQn, IRQ_PRIO_TIM_ACC, 0);
  HAL_NVIC_EnableIRQ(TIM_ACC_IRQn);

  /* Enable update interrupt and counter */
  TIM_ACC->DIER |= TIM_DIER_UIE;
  TIM_ACC->CR1 |= TIM_CR1_CEN;

  return HAL_OK;
}

/** @brief  PWM initialization
 *  @return HAL status
 */
HAL_StatusTypeDef pwmInit()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  /* TIM1 GPIO Configuration
   * PB13   ------> TIM1_CH1N
   * PB14   ------> TIM1_CH2N
   * PB15   ------> TIM1_CH3N
   * PA8   ------> TIM1_CH1
   * PA9   ------> TIM1_CH2
   * PA10   ------> TIM1_CH3
   */
  GPIO_InitStruct.Pin = TIM_PWM_CH1N | TIM_PWM_CH2N | TIM_PWM_CH3N;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TIM_PWM_CH1 | TIM_PWM_CH2 | TIM_PWM_CH3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(TIM_PWM_IRQn, IRQ_PRIO_TIM_PWM, 0);
  HAL_NVIC_EnableIRQ(TIM_PWM_IRQn);

  /* Peripheral TIM_PWM init */
  htim_pwm.Instance = TIM_PWM;
  htim_pwm.Init.Prescaler = 0;
  htim_pwm.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim_pwm.Init.Period = PWM_PERIOD;
  htim_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim_pwm.Init.RepetitionCounter = 1;
  if (HAL_TIM_PWM_Init(&htim_pwm) != HAL_OK)
    return HAL_ERROR;

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_pwm, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;                      // OC/OCN outputs are forced first with their idle level as soon as CCxE=1 or CCxNE=1
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = (uint32_t) (DEADTIME * TIM_PWM_CLK
                                              * pow(10.0, -9.0));
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_pwm, &sBreakDeadTimeConfig);

  /* 110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
   * else inactive. In downcounting, channel 1 is inactive (OC1REF=�0�) as long as
   * TIMx_CNT>TIMx_CCR1 else active (OC1REF=�1�).
   * 111: PWM mode 2 - In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1
   * else active. In downcounting, channel 1 is active as long as TIMx_CNT>TIMx_CCR1 else
   * inactive.
   */
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim_pwm, &sConfigOC, TIM_CHANNEL_3);

  /* Initialize SVM struct */
  memset(&svm, 0, sizeof(svm));
  svm.dutyc_pha = PWM_PERIOD;
  svm.dutyc_phb = PWM_PERIOD;
  svm.dutyc_phc = PWM_PERIOD;

  return HAL_OK;
}

/** @brief  This function starts the PWM output on CH1-CH3 and CHN1-CHN3
 */
void pwmStartIT()
{
  /* Enable capture & compare channels */
  TIM_PWM->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E
                   | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;

  /* Set capture & compare interrupt flags */
  TIM_PWM->SR |= TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF;

  /* Trigger update before starting the timer */
  TIM_PWM->EGR |= TIM_EGR_UG;

  /* Enable update interrupt */
  TIM_PWM->DIER |= TIM_DIER_UIE;

  /* Counter enable */
  TIM_PWM->CR1 |= TIM_CR1_CEN;
}

/** @brief  This function stops the PWM output on CH1-CH3 and CHN1-CHN3
 */
void pwmStopIT()
{
  /* Counter disabled */
  TIM_PWM->CR1 &= ~TIM_CR1_CEN;

  /* Disable update interrupt */
  TIM_PWM->DIER &= ~TIM_DIER_UIE;

  /* Main output disabled */
  TIM_PWM->BDTR &= ~TIM_BDTR_MOE;

  /* Disable capture & compare channels */
  TIM_PWM->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E
                     | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

  /* Trigger update after stopping the timer */
  TIM_PWM->EGR |= TIM_EGR_UG;

  /* Clear capture & compare interrupt flags */
  TIM_PWM->SR &= ~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
}

/** @brief  This function handles TIM_ACC global interrupt                      <br>
 *          (__weak version found in startup_stm32f405xx.s)
 */
void TIM_ACC_IRQHandler()
{
  /* Update interrupt */
  if (TIM_ACC->SR & TIM_SR_UIF)
  {
    /* Clear update interrupt flag */
    TIM_ACC->SR &= ~TIM_SR_UIF;

    /* Compare current and desired frequency if main output is enabled */
    if (TIM_PWM->BDTR & TIM_BDTR_MOE)
    {
      if (ac.f_act < *ac.f_ref)
        ac.f_act++;
      else if (ac.f_act > *ac.f_ref)
        ac.f_act--;
    }
  }
}

/** @brief  This function handles TIM_PWM global interrupt                      <br>
 *            (__weak version found in startup_stm32f405xx.s)
 *  @time   "O1" 5957 cycles (35.5us)
 */
//void __attribute__((optimize("O1"))) TIM_PWM_IRQHandler(void)
void TIM_PWM_IRQHandler()
{
  /* Update interrupt */
  if (TIM_PWM->SR & TIM_SR_UIF)
  {
    /* Clear update interrupt flag */
    TIM_PWM->SR &= ~TIM_SR_UIF;

    /* Change ADC external trigger
     * ADC2 measures negative currents
     * ADC3 would measure positive ones, but the digital isolator is only unidirectional
     * Bits 27:24 EXTSEL[3:0]: External event select for regular group
     * These bits select the external event used to trigger the start of conversion of a regular group:
     * 0000: Timer 1 CC1 event
     * 0001: Timer 1 CC2 event
     * 0010: Timer 1 CC3 event
     */

    /* Disable EXTEN during external trigger change */
    ADC2->CR2 &= ~(ADC_CR2_EXTEN | ADC_CR2_EXTSEL);
    //ADC3->CR2 &= ~(ADC_CR2_EXTEN | ADC_CR2_EXTSEL);

    switch (svm.nsector)
    {
    case 0:
      /* Well... actually I'd say we can only measure positive currents
       * with our unipolar hcnr201 circuit... but what do I know? */
      ADC2->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC2);             // -Ic
      //ADC3->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC1);           // +Ia
      break;

    case 1:
      ADC2->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC1);             // -Ic
      //ADC3->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC2);           // +Ib
      break;

    case 2:
      ADC2->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC3);             // -Ia
      //ADC3->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC2);           // +Ib
      break;

    case 3:
      ADC2->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC2);             // -Ia
      //ADC3->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC3);           // +Ic
      break;

    case 4:
      ADC2->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC1);             // -Ib
      //ADC3->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC3);           // +Ic
      break;

    case 5:
      ADC2->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC3);             // -Ib
      //ADC3->CR2 |= (ADC_CR2_EXTEN_0 | ADC_EXTERNALTRIGCONV_T1_CC1);           // +Ia
      break;
    }

    /* If main output is enabled... */
    if (TIM_PWM->BDTR & TIM_BDTR_MOE)
    {
      /* Set dutycycles */
      TIM_PWM->CCR1 = svm.dutyc_pha;
      TIM_PWM->CCR2 = svm.dutyc_phb;
      TIM_PWM->CCR3 = svm.dutyc_phc;

      /* Calculate step size */
      svm.phase_step = M_TWOPI * ac.f_act * TS;

      /* Check direction */
      if (*ac.is_dir_cw)
      {
        /* Update phase with phase step size */
        svm.phase = svm.phase + svm.phase_step;

        /* Check if sector has changed */
        if (svm.phase >= M_PI_3)
        {
          svm.phase = svm.phase - M_PI_3;

          /* Increment sector and check for roll over */
          if (++svm.nsector > 5)
            svm.nsector = 0;
        }
      }
      else
      {
        /* Update phase with phase step size */
        svm.phase = svm.phase - svm.phase_step;

        /* Check if sector has changed */
        if (svm.phase <= 0.0)
        {
          svm.phase = svm.phase + M_PI_3;

          /* Decrement sector and check for roll over */
          if (--svm.nsector < 0)
            svm.nsector = 5;
        }
      }

      /* Get motor voltage */
      ac.vm = (double) (MAX_VOLTAGE * ac.f_act) / MAX_FREQ;

      /* Get modulation index */
      svm.m = (double) ac.vm / adc.vbus.si;

      /* Get Ta */
      svm.ta = (uint16_t) (svm.m * sin(M_PI_3 - svm.phase) * TS_REG);

      /* Get Tb */
      svm.tb = (uint16_t) (svm.m * sin(svm.phase) * TS_REG);

      /* Get T0/2 */
      svm.t0_2 = TS_REG - (svm.ta + svm.tb);
      svm.t0_2 = svm.t0_2 >> 1;

      switch (svm.nsector)
      {
      case 0:
        svm.dutyc_pha = svm.t0_2;
        svm.dutyc_phb = svm.t0_2 + svm.ta;
        svm.dutyc_phc = TS_REG - svm.t0_2;
        break;

      case 1:
        svm.dutyc_pha = svm.t0_2 + svm.tb;
        svm.dutyc_phb = svm.t0_2;
        svm.dutyc_phc = TS_REG - svm.t0_2;
        break;

      case 2:
        svm.dutyc_pha = TS_REG - svm.t0_2;
        svm.dutyc_phb = svm.t0_2;
        svm.dutyc_phc = svm.t0_2 + svm.ta;
        break;

      case 3:
        svm.dutyc_pha = TS_REG - svm.t0_2;
        svm.dutyc_phb = svm.t0_2 + svm.tb;
        svm.dutyc_phc = svm.t0_2;
        break;

      case 4:
        svm.dutyc_pha = svm.t0_2 + svm.ta;
        svm.dutyc_phb = TS_REG - svm.t0_2;
        svm.dutyc_phc = svm.t0_2;
        break;

      case 5:
        svm.dutyc_pha = svm.t0_2;
        svm.dutyc_phb = TS_REG - svm.t0_2;
        svm.dutyc_phc = svm.t0_2 + svm.tb;
        break;
      }
    }
    /* Main output is disabled */
    else
    {
      /* Reset motor variables */
      ac.vm = 0;
      ac.f_act = 0;

      /* Reset PWM variables */
      svm.nsector = 0;
      svm.phase = 0;
      svm.phase_step = 0;
      svm.m = 0;
      svm.ta = 0;
      svm.tb = 0;
      svm.t0_2 = 0;
      svm.dutyc_pha = PWM_PERIOD;
      svm.dutyc_phb = PWM_PERIOD;
      svm.dutyc_phc = PWM_PERIOD;

      /* Reset ADC variables */
      adc.imot.si[0] = 0;
      adc.imot.si[1] = 0;
      adc.imot.si[2] = 0;

      /* Reset dutycycles */
      TIM_PWM->CCR1 = svm.dutyc_pha;
      TIM_PWM->CCR2 = svm.dutyc_phb;
      TIM_PWM->CCR3 = svm.dutyc_phc;
    }
  }
}
