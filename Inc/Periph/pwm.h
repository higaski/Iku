/***************************************************************************//**
 *  @brief      PWM
 *  @file       pwm.h
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 ******************************************************************************/

#ifndef __PWM_H
#define __PWM_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Macros */
/* Peripheral */
#define TIM_ACC                         TIM13
#define __TIM_ACC_CLK_ENABLE()          __TIM13_CLK_ENABLE()                    //!< Enable TIM_ACC clock
#define TIM_ACC_IRQn                    TIM8_UP_TIM13_IRQn                      //!< TIM_ACC IRQn_Type enumeration
#define TIM_ACC_IRQHandler              TIM8_UP_TIM13_IRQHandler                //!< TIM_ACC IRQ handler
#define TIM_ACC_CLK                     1000000                                 //!< Desired timer frequency
#define ACC_TIME                        6                                       //!< Desired acceleration time for maximum speed of 1000Hz
#define ACC_PERIOD                      ((ACC_TIME * TIM_ACC_CLK) / 750)

#define TIM_PWM                         TIM1
#define __TIM_PWM_CLK_ENABLE()          __TIM1_CLK_ENABLE()                     //!< Enable TIM_PWM clock
#define TIM_PWM_IRQn                    TIM1_UP_TIM10_IRQn                      //!< TIM_PWM IRQn_Type enumeration
#define TIM_PWM_IRQHandler              TIM1_UP_TIM10_IRQHandler                //!< TIM_PWM IRQ handler
#define TIM_PWM_CLK                     168000000                               //!< Desired timer frequency
#define TIM_PWM_CH1                     GPIO_PIN_8
#define TIM_PWM_CH2                     GPIO_PIN_9
#define TIM_PWM_CH3                     GPIO_PIN_10
#define TIM_PWM_CH1N                    GPIO_PIN_13
#define TIM_PWM_CH2N                    GPIO_PIN_14
#define TIM_PWM_CH3N                    GPIO_PIN_15
#define DISABLE                         GPIO_PIN_8

#define DISABLE_SI8233()                GPIOC->BSRR = GPIO_PIN_8
#define ENABLE_SI8233()                 GPIOC->BSRR = (uint32_t)GPIO_PIN_8 << 16

/* Other */
#define M_PI_3                          1.0471975511965976
#define MIN_FREQ                        250                                     //!< Minimum motor frequency
#define MAX_FREQ                        1000                                    //!< Maximum motor frequency
#define FREQ_SPAN                       (MAX_FREQ - MIN_FREQ)                   //!< Frequency span

/* Standard is a 10kHz PWM frequency. Please use one of the following defines to change
 * that. Be aware that the actual FOC frequency, and so the interrupt frequency, is
 * only halve of the PWM frequency.
 */
#define USE_20KHZ_PWM
//#define USE_30KHZ_PWM
//#define USE_40KHZ_PWM
//#define USE_50KHZ_PWM
//#define USE_60KHZ_PWM
#ifdef USE_20KHZ_PWM
#define FOC_FREQUENCY                   10000
#define PWM_PERIOD                      TIM_PWM_CLK / 20000                     //!< 20kHz (8400)
#define TS                              0.0001                                  //!< FOC Period = 2*PWM Period = 1/FOC Frequency
#elif USE_30KHZ_PWM
#define FOC_FREQUENCY                   15000
#define PWM_PERIOD                      TIM_PWM_CLK / 30000                     //!< 30kHz (5600)
#define TS                              0.00006666666666666670                  //!< FOC Period = 2*PWM Period = 1/FOC Frequency
#elif USE_40KHZ_PWM
#define FOC_FREQUENCY                   20000
#define PWM_PERIOD                      TIM_PWM_CLK / 40000                     //!< 40kHz (4200)
#define TS                              0.00005                                 //!< FOC Period = 2*PWM Period = 1/FOC Frequency
#elif USE_50KHZ_PWM
#define FOC_FREQUENCY                   25000
#define PWM_PERIOD                      TIM_PWM_CLK / 50000                     //!< 50kHz (3360)
#define TS                              0.00004                                 //!< FOC Period = 2*PWM Period = 1/FOC Frequency
#else
#define FOC_FREQUENCY                   5000
#define PWM_PERIOD                      TIM_PWM_CLK / 10000                     //!< 10kHz (16800)
#define TS                              0.0002                                  //!< FOC Period = 2*PWM Period = 1/FOC Frequency
#endif

#define TS_REG                          (PWM_PERIOD - 1)                        //!< Register value of ARR for a PWM period

#define DEADTIME                        91.0                                    //!< Deadtime in nanoseconds (see FDPF390N15A datasheet + 30%)

/* Variable definitions */
/** @struct SvPwm
 */
struct SvPwm                                                                    //! Struct for SVM algorithm
{
  double phase;                                                                 //!< Current phase angle [rad]
  double phase_step;                                                            //!< Current chase angle step size [rad]
  double m;                                                                     //!< Modulation index
  uint16_t ta;                                                                  //!< Time Ta already expressed in relation to the PWM period
  uint16_t tb;                                                                  //!< Time Tb already expressed in relation to the PWM period
  uint16_t t0_2;                                                                //!< Time T0/2 already expressed in relation to the PWM period
  uint16_t dutyc_pha;                                                           //!< Duty cycle for phase A
  uint16_t dutyc_phb;                                                           //!< Duty cycle for phase B
  uint16_t dutyc_phc;                                                           //!< Duty cycle for phase C
  int8_t nsector;                                                               //!< Sector number
};

/* Variable definitions (external) */
extern struct SvPwm svm;                                                        //!< Used extern

/* Prototypes */
HAL_StatusTypeDef accInit();
HAL_StatusTypeDef pwmInit();
void pwmStartIT();
void pwmStopIT();
void TIM_PWM_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif // __PWM_H
