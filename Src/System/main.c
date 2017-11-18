/***************************************************************************//**
 * @mainpage
 * This project was part of a CNC build which has used a low-voltage high speed
 * spindle as a tool. The spindle type (an EWL 4025) draws rather large currents
 * in combination with very low voltages. This can't be handled by most standard
 * VFDs and requires special (rather expensive) HF drives in a price range well
 * above 1000 bucks.
 *
 * The application is based on a STM32F405 and Microchips appnote AN955 which
 * describes the implementation of a space vector control for ASM machines.
 *
 * Further details can be found here:
 * http://higaski.at/projects/variable_frequency_drive/variable_frequency_drive.html
 *
 *  @file       main.c
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 ******************************************************************************/

/* ST/GCC includes */
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include <float.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

/* Project includes */
#include "dma_irq_priority.h"
#include "Periph/debounce.h"
#include "Periph/gpio.h"
#include "Periph/pwm.h"
#include "System/sys.h"

/** @brief  This function handles TIM_STATE_MACHINE global interrupt            <br>
 *          It basically acts as state machine for the whole motor control.
 *          (__weak version found in startup_stm32f405xx.s)
 */
void TIM_STATE_MACHINE_IRQHandler(void)
{
  /* Update interrupt */
  if (TIM_STATE_MACHINE->SR & TIM_SR_UIF)
  {
    /* Clear update interrupt flag */
    TIM_STATE_MACHINE->SR &= ~TIM_SR_UIF;

    switch (ac.state)
    {
    case START:
      /* Indicate that motor is running */
      RED_LED_ON();

      /* Main output enabled */
      TIM_PWM->BDTR |= TIM_BDTR_MOE;

      /* Propagate state */
      ac.state = RUN;
      break;

    case RUN:
      /* Propagate state */
      if (!(*ac.is_on))
        ac.state = HALT;
      break;

    case HALT:
      /* Main output disabled */
      TIM_PWM->BDTR &= ~TIM_BDTR_MOE;

      /* Kill pointers */
      ac.f_ref = 0;
      ac.is_dir_cw = 0;
      ac.is_on = 0;

      /* Disable update interrupt and timer */
      TIM_STATE_MACHINE->DIER &= ~TIM_DIER_UIE;
      TIM_STATE_MACHINE->CR1 &= ~TIM_CR1_CEN;

      /* Indicate that motor has stopped */
      HAL_Delay(10000);
      RED_LED_OFF();

      /* Propagate state */
      ac.state = START;
      break;
    }
  }
}

/** @brief  Main
 */
int main()
{
  /* Initialize system */
  if (sys_init() != HAL_OK)
    return HAL_ERROR;

  /* Clear eventually pressed buttons till here */
  clearKeyPress();

  /* Enable driver */
  ENABLE_SI8233();

  /* Infinite loop */
  while (1)
  {
    /* Be aware that delay between key repeats REPEAT_NEXT*timer (~20ms) must be kept */
    HAL_Delay(100);

    /* Check for key press, turn motor ON/OFF for MAN source */
    if (getKeyPress(1 << SW_ON))
      ac.man.is_on = !ac.man.is_on;

    /* Check source
     * Every setting depends on the source switch being set
     * E.g. if the direction switch changes while running from PWM, nothing happens...
     */
    if (SW_SRC == PWM)
    {
      /* If motor is not running... */
      if (!(*ac.is_on))
      {
        /* Set source */
        ac.is_src_pc = PWM;

        /* Set direction */
        if (PC_DIR == CW)
          ac.pwm.is_dir_cw = CW;
        else if (PC_DIR == CCW)
          ac.pwm.is_dir_cw = CCW;

        if (ac.pwm.is_on && (!TIM_STATE_MACHINE->CR1 & TIM_CR1_CEN))
        {
          /* Assign pointers */
          ac.f_ref = &ac.pwm.f_ref;
          ac.is_dir_cw = &ac.pwm.is_dir_cw;
          ac.is_on = &ac.pwm.is_on;

          /* Enable update interrupt and timer */
          TIM_STATE_MACHINE->DIER |= TIM_DIER_UIE;
          TIM_STATE_MACHINE->CR1 |= TIM_CR1_CEN;
        }
      }
    }
    else if (SW_SRC == MAN)
    {
      /* If motor is not running... */
      if (!(*ac.is_on))
      {
        /* Set source */
        ac.is_src_pc = MAN;

        /* Set direction */
        if (SW_DIR == CW)
          ac.man.is_dir_cw = CW;
        else if (SW_DIR == CCW)
          ac.man.is_dir_cw = CCW;

        if (ac.man.is_on && (!TIM_STATE_MACHINE->CR1 & TIM_CR1_CEN))
        {
          /* Assign pointers */
          ac.f_ref = &ac.man.f_ref;
          ac.is_dir_cw = &ac.man.is_dir_cw;
          ac.is_on = &ac.man.is_on;

          /* Enable update interrupt and timer */
          TIM_STATE_MACHINE->DIER |= TIM_DIER_UIE;
          TIM_STATE_MACHINE->CR1 |= TIM_CR1_CEN;
        }
      }
    }
  }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}

#endif
