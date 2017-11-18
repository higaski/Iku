/***************************************************************************//**
 *  @brief      Input capture
 *  @file       ic.h
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       23/08/2015
 *  @addtogroup input
 *  @{
 ******************************************************************************/

#ifndef __IC_H
#define __IC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Macros */
/* Peripheral */
#define TIM_IC                      TIM4
#define __TIM_IC_CLK_ENABLE()       __TIM4_CLK_ENABLE()                         //!< Enable TIM_IC clock
#define TIM_IC_IRQn                 TIM4_IRQn                                   //!< TIM_IC IRQn_Type enumeration
#define TIM_IC_IRQHandler           TIM4_IRQHandler                             //!< TIM_IC IRQ handler
#define TIM_IC_CLK                  4000000                                     //!< TIM_IC_CLK in [Hz]

/* Other */
#define IC_CNT						10											//!< Times input capture must be equal before the value is taken over

/* Variable definitions */
/** @struct InputCapture
 */
struct InputCapture                                                             //! Struct for input capture
{
  uint32_t freq;                                                                //!< Frequency
  uint32_t dutyc;                                                               //!< Dutycycle
};

/* Prototypes */
HAL_StatusTypeDef icInit();
void TIM_IC_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif // __IC_H
/** @}
 */
