/***************************************************************************//**
 *  @brief      Debounce
 *  @file       debounce.c
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 ******************************************************************************/

/* ST/GCC includes */
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Project includes */
#include "dma_irq_priority.h"
#include "Periph/debounce.h"
#include "Periph/gpio.h"
#include "System/sys.h"

/* Variables */
TIM_HandleTypeDef htim_debounce;                                                //!< TIM handle for TIM_DEBOUNCE
__IO uint16_t key_press;
__IO uint16_t key_release;
__IO uint16_t key_rpt;
__IO uint16_t key_state;

/** @brief  Debounce initialization                                             <br>
 *  @return HAL status
 */
HAL_StatusTypeDef debounceInit()
{
  TIM_MasterConfigTypeDef sMasterConfig;

  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(TIM_DEBOUNCE_IRQn, IRQ_PRIO_TIM_DEBOUNCE, 0);
  HAL_NVIC_EnableIRQ(TIM_DEBOUNCE_IRQn);

  /* Peripheral TIM_DEBOUNCE init */
  htim_debounce.Instance = TIM_DEBOUNCE;
  htim_debounce.Init.Prescaler = (uint32_t) ((SystemCoreClock / 2)
                                 / TIM_DEBOUNCE_CLK) - 1;
  htim_debounce.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_debounce.Init.Period = TIM_DEBOUNCE_PERIOD - 1;
  htim_debounce.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim_debounce) != HAL_OK)
    return HAL_ERROR;

  /* Enable update interrupt and timer */
  TIM_DEBOUNCE->DIER |= TIM_DIER_UIE;
  TIM_DEBOUNCE->CR1 |= TIM_CR1_CEN;

  return HAL_OK;
}

/** @brief  Get key press                                                       <br>
 *  @param  key_mask:   Key mask
 */
uint16_t getKeyPress(uint16_t key_mask)
{
  TIM_DEBOUNCE->DIER &= ~TIM_DIER_UIE;
  key_mask &= key_press;
  key_press ^= key_mask;
  TIM_DEBOUNCE->DIER |= TIM_DIER_UIE;
  return key_mask;
}

/** @brief  Get simulated key presses when button is held                       <br>
 *  @param  key_mask:   Key mask
 */
uint16_t getKeyRpt(uint16_t key_mask)
{
  TIM_DEBOUNCE->DIER &= ~TIM_DIER_UIE;
  key_mask &= key_rpt;
  key_rpt ^= key_mask;
  TIM_DEBOUNCE->DIER |= TIM_DIER_UIE;
  return key_mask;
}

/** @brief  Get key press short (takes till key release to be recognized)       <br>
 *  @param  key_mask:   Key mask
 */
uint16_t getKeyShort(uint16_t key_mask)
{
  TIM_DEBOUNCE->DIER &= ~TIM_DIER_UIE;
  return getKeyPress(~key_state & key_mask);
}

/** @brief  Get key press long (takes till key repeart start to be recognized)  <br>
 *  @param  key_mask:   Key mask
 */
uint16_t getKeyLong(uint16_t key_mask)
{
  return getKeyPress(getKeyRpt(key_mask));
}

/** @brief  Get key release
 *  @param  key_mask:   Key mask
 */
uint16_t getKeyRelease(uint16_t key_mask)
{
  TIM_DEBOUNCE->DIER &= ~TIM_DIER_UIE;
  key_mask &= key_release;
  key_release ^= key_mask;
  TIM_DEBOUNCE->DIER |= TIM_DIER_UIE;
  return key_mask;
}

/** @brief  Get key state                                                       <br>
 *  @param  key_mask:   Key mask
 */
uint16_t getKeyState(uint16_t key_mask)
{
  key_mask &= key_state;
  return key_mask;
}

/** @brief  Clear key press, release and repeat                                 <br>
 */
void clearKeyPress()
{
  key_press = 0;
  key_release = 0;
  key_rpt = 0;
}

/** @brief  This function handles TIM_DEBOUNCE global interrupt                 <br>
 *          (__weak version found in startup_stm32f405xx.s)
 */
void TIM_DEBOUNCE_IRQHandler()
{
  static uint16_t ct0, ct1, rpt;
  uint16_t y;

  /* Update interrupt */
  if (TIM_DEBOUNCE->SR & TIM_SR_UIF)
  {
    /* Clear update interrupt flag */
    TIM_DEBOUNCE->SR &= ~TIM_SR_UIF;

    y = key_state ^ ~(GPIOC->IDR);                                              // For active high keys remove "~"
    ct0 = ~(ct0 & y);
    ct1 = ct0 ^ (ct1 & y);
    y &= ct0 & ct1;
    key_state ^= y;
    key_press |= key_state & y;                                                 // 0->1; key press detect
    key_release |= ~key_state & y;                                              // 1->0; key release detect
    rpt--;

    if ((key_state & REPEAT_MASK) == 0)
      rpt = REPEAT_START;
    if (rpt == 0)
    {
      rpt = REPEAT_NEXT;
      key_rpt |= key_state & REPEAT_MASK;
    }
  }
}
