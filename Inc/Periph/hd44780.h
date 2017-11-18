/***************************************************************************//**
 *  @brief      HD44780 display driver
 *  @file       hd44780.h
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 *  @addtogroup display
 *  @{
 ******************************************************************************/

#ifndef __HD44780_H
#define __HD44780_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Macros */
/* Peripheral */
#define TIM_HD44780                         TIM7
#define __TIM_HD44780_CLK_ENABLE()          __TIM7_CLK_ENABLE()                 //!< Enable TIM_HD44780 clock
#define TIM_HD44780_IRQn                    TIM7_IRQn                           //!< TIM_HD44780 IRQn_Type enumeration
#define TIM_HD44780_IRQHandler              TIM7_IRQHandler                     //!< TIM_HD44780 IRQ handler
#define HD44780_QUEUE_SIZE                  500                                 //!< Queue size
#define HD44780_TIM_CLK                     1000000                             //!< TIM_HD44780_CLK in [Hz]
#define HD44780_TIM_PERIOD                  40                                  //!< Timer period for HD44780 update [us]
#define HD44780_TIMEOUT                     100000                              //!< Period after which busy flag is ignored [us]
#define HD44780_IGNORE_BUSY_PERIOD          2000                                //!< Timer period in case busy flag isn't working [us]

#define TIM_HD44780_TASK                    TIM5
#define __TIM_HD44780_TASK_CLK_ENABLE()     __TIM5_CLK_ENABLE()                 //!< Enable TIM_HD44780_TASK clock
#define TIM_HD44780_TASK_IRQn               TIM5_IRQn                           //!< TIM_HD44780_TASK IRQn_Type enumeration
#define TIM_HD44780_TASK_IRQHandler         TIM5_IRQHandler                     //!< TIM_HD44780_TASK IRQ handler
#define TIM_HD44780_TASK_CLK                10000                               //!< TIM_HD44780_TASK_CLK in [Hz]
#define TIM_HD44780_TASK_PERIOD             5000                                //!< TIM_HD44780_TASK_PERIOD [0.5s]

#define GPIO_CTRL                           GPIOC
#define E                                   GPIO_PIN_13                         //!< PC13
#define RW                                  GPIO_PIN_14                         //!< PC14
#define RS                                  GPIO_PIN_15                         //!< PC15

#define GPIO_DB                             GPIOB
#define DB4                                 GPIO_PIN_0                          //!< PB0
#define DB5                                 GPIO_PIN_1                          //!< PB1
#define DB6                                 GPIO_PIN_2                          //!< PB2
#define DB7                                 GPIO_PIN_10                         //!< PB10

/* Other */
#define CMD                                 true
#define CMD_CLEAR                           0x01
#define CMD_ON                              0x0C
#define CMD_CURSOR_ON                       0x0E
#define CMD_BLINKING_ON                     0x0D
#define CMD_RETURN_HOME                     0x02
#define HD44780_CGRAM                       0x0040
#define HD44780_DDRAM                       0x0080

#define HD44780_ADR_TILDE                   0x0                                 //!< Address of tilde character in custom ram
#define HD44780_ADR_EMPTY_RECT              0x1                                 //!< Address of empty rectangular in custom ram
#define HD44780_ADR_FULL_RECT               0x2                                 //!< Address of full rectangular in custom ram
#define HD44780_ADR_CW                      0x3                                 //!< Address of CW sign in custom ram
#define HD44780_ADR_CCW                     0x4                                 //!< Address of CCW sign in custom ram
#define HD44780_ADR_BRAKE                   0x5                                 //!< Address of brake sign in custom ram
#define HD44780_ADR_EXP_M1					0x6									//!< Address of ^-1 exponent sign in custom ram

/* Prototypes */
void hd44780SendChar(uint8_t data);
void hd44780SendString(uint8_t* string);
void hd44780SendCmd(bool display, bool cursor, bool blinking);
void hd44780Clear();
void hd44780Home();
void hd44780Goto(uint8_t row, uint8_t col);
HAL_StatusTypeDef hd44780Init();
HAL_StatusTypeDef hd44780TaskInit();
void TIM_HD44780_IRQHandler();
void TIM_HD44780_TASK_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif // __HD44780_H
/** @}
 */
