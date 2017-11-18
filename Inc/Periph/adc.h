/***************************************************************************//**
 *  @brief      ADC
 *  @file       adc.h
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 ******************************************************************************/

#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Macros */
/* Peripheral */
#define ADC_VBUS                        ADC1                                    //!< ADC for bus voltage measurements
#define __ADC_VBUS_CLK_ENABLE()         __ADC1_CLK_ENABLE()                     //!< Enable ADC_VBUS clock
#define DMA_ADC_VBUS_STREAM             DMA2_Stream0                            //!< ADC_VBUS DMA channel
#define DMA_ADC_VBUS_IRQn               DMA2_Stream0_IRQn                       //!< ADC_VBUS IRQn_Type enumeration
#define DMA_ADC_VBUS_IRQHandler         DMA2_Stream0_IRQHandler                 //!< ADC_VBUS DMA IRQ handler

#define ADC_IMOT                        ADC2                                    //!< ADC for phase current measurements
#define __ADC_I_CLK_ENABLE()            __ADC2_CLK_ENABLE()                     //!< Enable ADC_I clock
#define DMA_ADC_IMOT_STREAM             DMA2_Stream2                            //!< ADC_IMOT DMA channel
#define DMA_ADC_IMOT_IRQn               DMA2_Stream2_IRQn                       //!< ADC_IMOT IRQn_Type enumeration
#define DMA_ADC_IMOT_IRQHandler         DMA2_Stream2_IRQHandler                 //!< ADC_IMOT DMA IRQ handler

#define ADC_CH_VBUS                     ADC_CHANNEL_10                          //!< ADC channel measuring the voltage on the right track
#define ADC_CH_IMOT                     ADC_CHANNEL_12                          //!< ADC channel measuring the voltage on the left track

/* Size */
#define ADC_VBUS_BUFFER_SIZE            64                                      //!< Size of ADC_VBUS buffer
#define ADC_IMOT_BUFFER_SIZE            16                                      //!< Size of ADC_IMOT buffer

/* Other */
#define ADC_VBUS_AVG_WINDOW             64                                      //!< Moving average window size
#define MAX_VOLTAGE                     40                                      //!< Maximum motor voltage
#define MAX_CURRENT                     20.0                                    //!< Maximum motor current
#define MAX_CURRENT_N_OVERSHOOT         500                                     //!< Times maximum motor current must occure before error
#define ADC_VBUS_CONV_FACTOR            (3.3 * 45.5)                    \
                                        / (4096 * 1.5                   \
                                        * ADC_VBUS_BUFFER_SIZE          \
                                        * ADC_VBUS_AVG_WINDOW)
#define ADC_IMOT_CONV_FACTOR            (3.3 * 20 * 33) / (4096 * 100)

/* Variable definitions */
/** @struct AdcVbusBuf
 */
struct AdcVbusBuf                                                               //! Struct for bus measurements
{
  double si;                                                                    //!< SI units
  uint32_t avg;                                                                 //!< Average
  uint16_t buf[ADC_VBUS_BUFFER_SIZE];                                           //!< Buffer
};

/** @struct AdcIBuf
 */
struct AdcIBuf                                                                  //! Struct for current measurements
{
  double si[3];                                                                 //!< SI units
  uint16_t buf[ADC_IMOT_BUFFER_SIZE];                                           //!< Buffer
};

/** @struct Adc
 */
struct Adc                                                                      //! Struct for ADCs
{
  struct AdcVbusBuf vbus;                                                       //!< Struct for bus measurements
  struct AdcIBuf imot;                                                          //!< Struct for current measurements
};

/* Variable definitions (external) */
extern struct Adc adc;                                                          //!< Used extern

/* Prototypes */
HAL_StatusTypeDef adcInit();
void DMA_ADC_VBUS_IRQHandler();
void ADC_IMOT_DMA_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif // __ADC_H
