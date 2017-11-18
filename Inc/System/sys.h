/***************************************************************************//**
 *  @brief      System management
 *  @file       sys.h
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       06/10/2015
 ******************************************************************************/

#ifndef __SYS_H
#define __SYS_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Macros */
/* Peripheral */
#define TIM_STATE_MACHINE                   TIM3
#define __TIM_STATE_MACHINE_CLK_ENABLE()    __TIM3_CLK_ENABLE()                 //!< Enable TIM_STATE_MACHINE clock
#define TIM_STATE_MACHINE_IRQn              TIM3_IRQn                           //!< TIM_STATE_MACHINE IRQn_Type enumeration
#define TIM_STATE_MACHINE_IRQHandler        TIM3_IRQHandler                     //!< TIM_STATE_MACHINE IRQ handler
#define TIM_STATE_MACHINE_CLK               100000                              //!< TIM_STATE_MACHINE_CLK in [Hz]
#define TIM_STATE_MACHINE_PERIOD            100                                 //!< TIM_STATE_MACHINE_PERIOD [10ms]

#define RED_LED_OFF()                       GPIOA->BSRR = GPIO_PIN_5
#define RED_LED_ON()                        GPIOA->BSRR = (uint32_t)GPIO_PIN_5 << 16
#define RED_LED_TOGGLE()                    GPIOA->ODR ^= GPIO_PIN_5
#define RED_LED_IS_ON()                     (!(GPIOA->ODR & GPIO_PIN_5))
#define YELLOW_LED_OFF()                    GPIOA->BSRR = GPIO_PIN_6
#define YELLOW_LED_ON()                     GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16
#define YELLOW_LED_TOGGLE()                 GPIOA->ODR ^= GPIO_PIN_6

/* Other */
#define PWM                                 true
#define MAN                                 false
#define CW                                  true
#define CCW                                 false

/* Variable definitions */
/** @enum MotorStates
 */
enum MotorStates                                                                //! Enumeration for motor states
{
  START = 0,                                                                    //!< Motor starts
  RUN,                                                                          //!< Motor runs
  HALT                                                                          //!< Motor stops
};

/** @struct Input
 */
struct Input                                                                    //! Struct for input
{
  uint16_t f_ref;                                                               //!< Reference frequency used [Hz]
  bool is_dir_cw;                                                               //!< Direction (true CW / false CCW)
  bool is_on;                                                                   //!< Motor flag (true ON / false OFF)
};

/** @struct Motor
 */
struct Motor                                                                    //! Struct for motor
{
  struct Input man;                                                             //!< Struct for encoder input
  struct Input pwm;                                                             //!< Struct for PWM input
  enum MotorStates state;                                                       //!< Motor state
  double vm;                                                                    //!< Motor voltage
  uint16_t f_act;                                                               //!< Actual frequency [Hz]
  uint16_t* f_ref;                                                              //!< Pointer to reference frequency [Hz]
  bool* is_dir_cw;                                                              //!< Pointer to direction (true CW / false CCW)
  bool* is_on;                                                                  //!< Pointer to motor flag (true ON / false OFF)
  bool is_src_pc;                                                               //!< Source (true PC-PWM / false manual)
  bool is_err;                                                                  //!< Error flag
};

/* Variable definitions (external) */
extern struct Motor ac;                                                         //!< Used extern

/* Prototypes */
HAL_StatusTypeDef sys_init();
void SystemClock_Config();
void SysTick_Handler();
void MemManage_Handler();

#ifdef __cplusplus
}
#endif

#endif // __SYS_H
