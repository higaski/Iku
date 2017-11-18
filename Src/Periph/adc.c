/***************************************************************************//**
 *  @brief      ADC
 *  @file       adc.c
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 ******************************************************************************/

/* ST/GCC includes */
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Project includes */
#include "dma_irq_priority.h"
#include "Periph/adc.h"
#include "Periph/gpio.h"
#include "Periph/pwm.h"
#include "System/sys.h"

/* Variables */
ADC_HandleTypeDef hadc_vbus;                                                    //!< ADC handle for ADC_VBUS
ADC_HandleTypeDef hadc_imot;                                                    //!< ADC handle for ADC_IMOT
DMA_HandleTypeDef hdma_adc_vbus;                                                //!< DMA handle for ADC_VBUS DMA
DMA_HandleTypeDef hdma_adc_imot;                                                //!< DMA handle for ADC_IMOT DMA
struct Adc adc;                                                                 //!< Struct for ADCs

/** @brief  Fast sum calculation of uint16                                      <br>
 *          This function utilizes the UXTAH instruction which extracts two
 *          16bit values from a 32bit memory address. Furthermore the ALU can
 *          do 2x16bit additions within a 32bit ALU register. This practically
 *          halves the needed additions.
 */
static void sumUint16(uint16_t *p_src, uint32_t size, uint32_t *p_dest)
{
  uint32_t sum = 0;                                                             // Temporary result storage
  uint32_t blk_cnt;                                                             // Loop counter

  /* Run the below code for Cortex-M4 and Cortex-M3 */
  uint32_t in;

  /*Loop unrolling */
  blk_cnt = size >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   * a second loop below computes the remaining 1 to 3 samples.
   */
  while (blk_cnt > 0u)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    in = *(*(uint32_t **) &(p_src))++;
    sum += ((in << 16) >> 16);
    sum += (in >> 16);
    in = *(*(uint32_t **) &(p_src))++;
    sum += ((in << 16) >> 16);
    sum += (in >> 16);

    /* Decrement the loop counter */
    blk_cnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   * No loop unrolling is used.
   */
  blk_cnt = size % 0x4u;

  while (blk_cnt > 0u)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    sum += *p_src++;

    /* Decrement the loop counter */
    blk_cnt--;
  }

  /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) / blockSize
   * Store the result to the destination */
  *p_dest = sum;
}

/** @brief  ADC_VBUS initialization                                             <br>
 *          This function initializes ADC1 to continuously measure the bus
 *          voltage.                                                            <br>
 *          CLK:                    42 MHz                                      <br>
 *          Sampling time:          480 cycles                                  <br>
 *          Conversion time:        11.7 us                                     <br>
 *          Samples:                64                                          <br>
 *          Total conversion time:  749.7 us                                    <br>
 *  @return HAL status
 */
static HAL_StatusTypeDef adcVbusInit()
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_MultiModeTypeDef multimode;
  GPIO_InitTypeDef GPIO_InitStruct;
  __IO uint32_t cnt = 0;

  /* Peripheral DMA init for vbus measurements */
  hdma_adc_vbus.Instance = DMA_ADC_VBUS_STREAM;
  hdma_adc_vbus.Init.Channel = DMA_CHANNEL_0;
  hdma_adc_vbus.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc_vbus.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc_vbus.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc_vbus.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc_vbus.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc_vbus.Init.Mode = DMA_NORMAL;
  hdma_adc_vbus.Init.Priority = DMA_PRIO_ADC_VBUS;
  hdma_adc_vbus.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_adc_vbus);

  __HAL_LINKDMA(&hadc_vbus, DMA_Handle, hdma_adc_vbus);

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA_ADC_VBUS_IRQn, IRQ_PRIO_DMA_ADC_VBUS, 0);
  HAL_NVIC_EnableIRQ(DMA_ADC_VBUS_IRQn);

  /* ADC1 GPIO Configuration
   * PC0     ------> ADC1_IN10
   */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure the global features of the ADC_VBUS */
  hadc_vbus.Instance = ADC_VBUS;
  hadc_vbus.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc_vbus.Init.Resolution = ADC_RESOLUTION12b;
  hadc_vbus.Init.ScanConvMode = DISABLE;
  hadc_vbus.Init.ContinuousConvMode = ENABLE;
  hadc_vbus.Init.DiscontinuousConvMode = DISABLE;
  hadc_vbus.Init.NbrOfDiscConversion = 1;
  hadc_vbus.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc_vbus.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc_vbus.Init.NbrOfConversion = 1;
  hadc_vbus.Init.DMAContinuousRequests = ENABLE;
  hadc_vbus.Init.EOCSelection = EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc_vbus) != HAL_OK)
    return HAL_ERROR;

  /* Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
  sConfig.Channel = ADC_CH_VBUS;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc_vbus, &sConfig);

  /* Enable ADC_VBUS DMA mode */
  ADC_VBUS->CR2 |= ADC_CR2_DMA;
  /* Set DMA data transfer direction to read from peripheral */
  DMA_ADC_VBUS_STREAM->CR &= ~DMA_SxCR_DIR;
  /* Configure DMA stream data length */
  DMA_ADC_VBUS_STREAM->NDTR = ADC_VBUS_BUFFER_SIZE;
  /* Configure DMA stream source address */
  DMA_ADC_VBUS_STREAM->PAR = (uint32_t) &ADC_VBUS->DR;
  /* Configure DMA stream destination address */
  DMA_ADC_VBUS_STREAM->M0AR = (uint32_t) &adc.vbus.buf;
  /* Enable the transfer complete and transfer error interrupt and the DMA stream */
  DMA_ADC_VBUS_STREAM->CR |= DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_EN;

  /* Enable ADC_VBUS */
  ADC_VBUS->CR2 |= ADC_CR2_ADON;

  /* Delay for ADC stabilization time
   * Compute number of CPU cycles to wait for
   */
  cnt = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000));
  while (cnt != 0)
  {
    cnt--;
  }

  /* Enable the selected ADC software conversion for regular group */
  ADC_VBUS->CR2 |= ADC_CR2_SWSTART;

  return HAL_OK;
}

/** @brief  ADC_IMOT initialization                                             <br>
 *          This function initializes ADC2 to measre the phase currents. The ADC
 *          is triggered by a timer1 capture and compare trigger.
 *  @return HAL status
 */
static HAL_StatusTypeDef adcImotInit()
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_MultiModeTypeDef multimode;
  GPIO_InitTypeDef GPIO_InitStruct;
  __IO uint32_t cnt = 0;

  /* Peripheral DMA init for phase current measurements */
  hdma_adc_imot.Instance = DMA_ADC_IMOT_STREAM;
  hdma_adc_imot.Init.Channel = DMA_CHANNEL_1;
  hdma_adc_imot.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc_imot.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc_imot.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc_imot.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc_imot.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc_imot.Init.Mode = DMA_CIRCULAR;
  hdma_adc_imot.Init.Priority = DMA_PRIO_ADC_IMOT;
  hdma_adc_imot.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_adc_imot);

  __HAL_LINKDMA(&hadc_imot, DMA_Handle, hdma_adc_imot);

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA_ADC_IMOT_IRQn, IRQ_PRIO_DMA_ADC_IMOT, 0);
  HAL_NVIC_EnableIRQ(DMA_ADC_IMOT_IRQn);

  /* ADC2 GPIO Configuration
   * PC2     ------> ADC2_IN12
   */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure the global features of the ADC_IMOT */
  hadc_imot.Instance = ADC_IMOT;
  hadc_imot.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc_imot.Init.Resolution = ADC_RESOLUTION12b;
  hadc_imot.Init.ScanConvMode = ENABLE;
  hadc_imot.Init.ContinuousConvMode = DISABLE;
  hadc_imot.Init.DiscontinuousConvMode = DISABLE;
  hadc_imot.Init.NbrOfDiscConversion = 1;
  hadc_imot.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc_imot.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc_imot.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc_imot.Init.NbrOfConversion = 16;
  hadc_imot.Init.DMAContinuousRequests = ENABLE;
  hadc_imot.Init.EOCSelection = EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc_imot) != HAL_OK)
    return HAL_ERROR;

  /* Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
  sConfig.Channel = ADC_CH_IMOT;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  for (uint8_t i = 1; i <= 16; i++)
  {
    sConfig.Rank = i;
    HAL_ADC_ConfigChannel(&hadc_imot, &sConfig);
  }

  /* Enable ADC_IMOT DMA mode */
  ADC_IMOT->CR2 |= ADC_CR2_DMA;
  /* Set DMA data transfer direction to read from peripheral */
  DMA_ADC_IMOT_STREAM->CR &= ~DMA_SxCR_DIR;
  /* Configure DMA stream data length */
  DMA_ADC_IMOT_STREAM->NDTR = 16;
  /* Configure DMA stream source address */
  DMA_ADC_IMOT_STREAM->PAR = (uint32_t) &ADC_IMOT->DR;
  /* Configure DMA stream destination address */
  DMA_ADC_IMOT_STREAM->M0AR = (uint32_t) &adc.imot.buf;
  /* Enable the transfer complete and transfer error interrupt and the DMA stream */
  DMA_ADC_IMOT_STREAM->CR |= DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_EN;

  /* Enable ADC_IMOT */
  ADC_IMOT->CR2 |= ADC_CR2_ADON;

  /* Delay for ADC stabilization time
   * Compute number of CPU cycles to wait for
   */
  cnt = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000));
  while (cnt != 0)
  {
    cnt--;
  }

  return HAL_OK;
}

/** @brief  ADC initialization                                                  <br>
 *          This function initializes ADC_VBUS and ADC_IMOT
 *  @return HAL status
 */
HAL_StatusTypeDef adcInit()
{
  if (adcVbusInit() != HAL_OK)
    return HAL_ERROR;

  if (adcImotInit() != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}

/** @brief  This function handles the DMA_ADC_VBUS_IRQHandler interrupt         <br>
 *          (__weak version found in startup_stm32f405xx.s)
 */
void DMA_ADC_VBUS_IRQHandler()
{
  uint32_t sum = 0;

  /* Transfer complete */
  if (DMA2->LISR & DMA_LISR_TCIF0)
  {
    /* Stop ADC_IMOT */
    ADC_VBUS->CR2 &= ~ADC_CR2_CONT;
    ADC_VBUS->SR &= ~ADC_SR_OVR;

    /* Disable the transfer complete interrupt and channel */
    DMA_ADC_VBUS_STREAM->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_EN);

    /* Clear the transfer complete flag */
    DMA2->LIFCR |= DMA_LIFCR_CTCIF0;

    /* Get sum, moving average and SI units */
    sumUint16((uint16_t*) &adc.vbus.buf, ADC_VBUS_BUFFER_SIZE, &sum);
    adc.vbus.avg = (adc.vbus.avg + sum)
                   - (adc.vbus.avg / ADC_VBUS_AVG_WINDOW);
    adc.vbus.si = adc.vbus.avg * ADC_VBUS_CONV_FACTOR;

    /* Restart ADC_IMOT DMA
     * Configure DMA stream data length
     */
    DMA_ADC_VBUS_STREAM->NDTR = ADC_VBUS_BUFFER_SIZE;
    /* Configure DMA stream destination address */
    DMA_ADC_VBUS_STREAM->M0AR = (uint32_t) &adc.vbus.buf;
    /* Enable the transfer complete and transfer error interrupt and the DMA stream */
    DMA_ADC_VBUS_STREAM->CR |= DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_EN;

    /* Enable the selected ADC software conversion for regular group */
    ADC_VBUS->CR2 |= ADC_CR2_SWSTART | ADC_CR2_CONT;
  }

  /* Transfer error */
  if (DMA2->LISR & DMA_LISR_TEIF0)
  {
    /* Clear the transfer error flag */
    DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
  }
}

/** @brief  This function handles the DMA_ADC_IMOT_IRQHandler interrupt            <br>
 *          (__weak version found in startup_stm32f405xx.s)
 */
void DMA_ADC_IMOT_IRQHandler()
{
  static uint32_t err_cnt = 0;
  uint32_t sum = 0;
  uint32_t mean = 0;
  float si;

  /* Transfer complete */
  if (DMA2->LISR & DMA_LISR_TCIF2)
  {
    /* Clear the transfer complete flag */
    DMA2->LIFCR |= DMA_LIFCR_CTCIF2;

    /* Get mean and SI units */
    sumUint16((uint16_t*) &adc.imot.buf, ADC_IMOT_BUFFER_SIZE, &sum);
    mean = sum / ADC_IMOT_BUFFER_SIZE;
    si = mean * ADC_IMOT_CONV_FACTOR;

    /* Check if maximum current is exceeded */
    if (si > MAX_CURRENT)
    {
      /* Shut down driver */
      if (++err_cnt > MAX_CURRENT_N_OVERSHOOT)
      {
        DISABLE_SI8233();
        ac.is_err = true;
      }
    }

    switch (svm.nsector)
    {
    case 0:
      adc.imot.si[2] = si;
      break;

    case 1:
      adc.imot.si[2] = si;
      break;

    case 2:
      adc.imot.si[0] = si;
      break;

    case 3:
      adc.imot.si[0] = si;
      break;

    case 4:
      adc.imot.si[1] = si;
      break;

    case 5:
      adc.imot.si[1] = si;
      break;
    }
  }

  /* Transfer error */
  if (DMA2->LISR & DMA_LISR_TEIF2)
  {
    /* Clear the transfer error flag */
    DMA2->LIFCR |= DMA_LIFCR_CTEIF2;
  }
}
