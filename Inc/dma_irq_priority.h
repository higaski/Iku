/***************************************************************************//**
 *  @brief      DMA and IRQ priorities
 *  @file       dma_irq_priority.h
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 ******************************************************************************/

#ifndef __DMA_IRQ_PRIO_H
#define __DMA_IRQ_PRIO_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Macros */
#define IRQ_PRIO_TIM_PWM                    0                                   //!< Interrupt priority of PWM
#define IRQ_PRIO_MEM_MANAGEMENT             1                                   //!< Interrupt priority of memory management fault
#define IRQ_PRIO_DMA_ADC_IMOT               1                                   //!< Interrupt priority of DMA transfer for ADC_IMOT
#define IRQ_PRIO_DMA_ADC_VBUS               2                                   //!< Interrupt priority of DMA transfer for ADC_VBUS
#define IRQ_PRIO_SYSTICK                    3                                   //!< Interrupt priority of system tick
#define IRQ_PRIO_TIM_ENCODER_READ           8                                   //!< Interrupt priority of encoder check timer
#define IRQ_PRIO_TIM_DEBOUNCE               10                                  //!< Interrupt priority of debounce timer
#define IRQ_PRIO_TIM_ACC                    11                                  //!< Interrupt priority of acceleration timer
#define IRQ_PRIO_TIM_IC                     12                                  //!< Interrupt priority of input capture
#define IRQ_PRIO_TIM_HD44780                13                                  //!< Interrupt priority of HD44780 driver timer
#define IRQ_PRIO_TIM_HD44780_TASK           14                                  //!< Interrupt priority of HD44780 display updates timer
#define IRQ_PRIO_TIM_STATE_MACHINE          15                                  //!< Interrupt priority of state machine

#define DMA_PRIO_ADC_VBUS                   DMA_PRIORITY_LOW                    //!< DMA priority of ADC_VBUS
#define DMA_PRIO_ADC_IMOT                   DMA_PRIORITY_VERY_HIGH              //!< DMA priority of ADC_IMOT

#ifdef __cplusplus
}
#endif

#endif // __DMA_IRQ_PRIO_H
