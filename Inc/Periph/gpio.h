/***************************************************************************//**
 *  @brief      GPIO
 *  @file       gpio.h
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 *  @addtogroup input
 *  @{
 ******************************************************************************/

#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Macros */
#define PC_DIR              ((GPIOB->IDR & GPIO_PIN_7)>>7)                      //!< Direction input from PC
#define SW_SRC              ((GPIOC->IDR & GPIO_PIN_10)>>10)                    //!< Source switch (true PC-PWM / false manual)
#define SW_DIR              ((GPIOC->IDR & GPIO_PIN_11)>>11)                    //!< Direction switch (true CW / false CCW)

/* Prototypes */
HAL_StatusTypeDef gpioInit();

#ifdef __cplusplus
}
#endif

#endif // __GPIO_H
/** @}
 */
