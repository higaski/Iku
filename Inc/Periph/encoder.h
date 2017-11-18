/***************************************************************************//**
 *  @brief        Encoder
 *  @file        encoder.h
 *  @version    0.1
 *  @author        Vincent Hamp
 *  @date        12/05/2015
 *  @addtogroup input
 *  @{
 ******************************************************************************/

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Macros */
/* Peripheral */
#define TIM_ENCODER                         TIM2
#define __TIM_ENCODER_CLK_ENABLE()          __TIM2_CLK_ENABLE()                 //!< Enable TIM_ENCODER clock
#define IC_FILTER                           0xF

#define TIM_ENCODER_READ                    TIM12
#define __TIM_ENCODER_READ_CLK_ENABLE()     __TIM12_CLK_ENABLE()                //!< Enable TIM_ENCODER_READ clock
#define TIM_ENCODER_READ_IRQn               TIM8_BRK_TIM12_IRQn                 //!< TIM_ENCODER_READ IRQn_Type enumeration
#define TIM_ENCODER_READ_IRQHandler         TIM8_BRK_TIM12_IRQHandler           //!< TIM_ENCODER_READ IRQ handler
#define TIM_ENCODER_READ_CLK                100000                              //!< TIM_ENCODER_READ_CLK in [Hz]
#define TIM_ENCODER_READ_PERIOD             25000                               //!< TIM_ENCODER_READ_PERIOD [250ms]

/* Prototypes */
HAL_StatusTypeDef encoderInit();
void TIM_ENCODER_READ_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif // __ENCODER_H
/** @}
 */
