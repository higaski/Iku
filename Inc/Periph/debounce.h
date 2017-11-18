/***************************************************************************//**
 *  @brief      Debounce
 *  @file       debounce.h
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 *  @addtogroup input
 *  @{
 ******************************************************************************/

#ifndef __DEBOUNCE_H
#define __DEBOUNCE_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Macros */
/* Peripheral */
#define TIM_DEBOUNCE                        TIM14
#define __TIM_DEBOUNCE_CLK_ENABLE()         __TIM14_CLK_ENABLE()                //!< Enable TIM_DEBOUNCE clock
#define TIM_DEBOUNCE_IRQn                   TIM8_TRG_COM_TIM14_IRQn             //!< TIM_DEBOUNCE IRQn_Type enumeration
#define TIM_DEBOUNCE_IRQHandler             TIM8_TRG_COM_TIM14_IRQHandler       //!< TIM_DEBOUNCE IRQ handler
#define TIM_DEBOUNCE_CLK                    10000                               //!< TIM_DEBOUNCE_CLK in [Hz]
#define TIM_DEBOUNCE_PERIOD                 100                                 //!< TIM_DEBOUNCE_PERIOD [10ms]

/* Other */
#define SW_ON                               12

#define REPEAT_MASK                         (1<<SW_ON)
#define REPEAT_START                        80                                  //!< Time till pressed key gets recognized as "repeat" (value * timer)
#define REPEAT_NEXT                         20                                  //!< Time between each simulated key press (value * timer)

/* Prototypes */
HAL_StatusTypeDef debounceInit();
uint16_t getKeyPress(uint16_t key_mask);
uint16_t getKeyRpt(uint16_t key_mask);
uint16_t getKeyShort(uint16_t key_mask);
uint16_t getKeyLong(uint16_t key_mask);
uint16_t getKeyRelease(uint16_t key_mask);
uint16_t getKeyState(uint16_t key_mask);
void clearKeyPress();
void TIM_DEBOUNCE_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif // __DEBOUNCE_H
/** @}
 */
