/***************************************************************************//**
 *  @brief      HD44780 display driver
 *  @file       hd44780.c
 *  @version    0.1
 *  @author     Vincent Hamp
 *  @date       12/05/2015
 *  @addtogroup display
 *  @{
 ******************************************************************************/

/* ST/GCC includes */
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>

/* Project includes */
#include "dma_irq_priority.h"
#include "Diag/dwt_cyccnt.h"
#include "Periph/adc.h"
#include "Periph/gpio.h"
#include "Periph/hd44780.h"
#include "Periph/pwm.h"
#include "System/sys.h"

/* Variables */
TIM_HandleTypeDef htim_hd44780;                                                 //!< TIM handle for TIM_HD44780
TIM_HandleTypeDef htim_hd44780_task;                                            //!< TIM handle for TIM_HD44780_TASK

/** @struct Hd44780
 */
static struct Hd44780                                                           //! Struct for HD44780
{
  uint8_t data;
  bool is_cmd;
} hd44780_queue[HD44780_QUEUE_SIZE];

static uint16_t qread = 0;                                                      //!< Index of current read position
static uint16_t qwrite = 0;                                                     //!< Index of next empty write position

/** @brief  This function delivers very short delays                            <br>
 *          10  iterations ~ 892ns (with call overhead)                         <br>
 *          255 iterations ~3090ns
 *  @param  delay:  Delay in ns
 */
static void delayNs(uint8_t delay)
{
  while (delay != 0)
  {
    delay--;
  }
}

/** @brief  This function writes the high nibble of a byte                      <br>
 *  @param  data:   Data to write
 */
static void highNibble(uint8_t data)
{
  HAL_GPIO_WritePin(GPIO_DB, DB4, (data & 0x10));
  HAL_GPIO_WritePin(GPIO_DB, DB5, (data & 0x20));
  HAL_GPIO_WritePin(GPIO_DB, DB6, (data & 0x40));
  HAL_GPIO_WritePin(GPIO_DB, DB7, (data & 0x80));
}

/** @brief  This function writes the low nibble of a byte                       <br>
 *  @param  data:   Data to write
 */
static void lowNibble(uint8_t data)
{
  HAL_GPIO_WritePin(GPIO_DB, DB4, (data & 0x01));
  HAL_GPIO_WritePin(GPIO_DB, DB5, (data & 0x02));
  HAL_GPIO_WritePin(GPIO_DB, DB6, (data & 0x04));
  HAL_GPIO_WritePin(GPIO_DB, DB7, (data & 0x08));
}

/** @brief  This function writes a byte to the LCD
 *  @param  data:   Data to write
 *  @param  is_cmd: Indicates if command or data is send                        <br>
 *                  0: data                                                     <br>
 *                  1: command
 */
static void writeByte(uint8_t data, bool is_cmd)
{
  /* Clear RS in case of command */
  if (is_cmd)
    *((uint32_t *) &GPIO_CTRL->BSRR) = RS << 16;
  else
    *((uint32_t *) &GPIO_CTRL->BSRR) = RS;

  /* Send high nibble */
  *((uint32_t *) &GPIO_CTRL->BSRR) = E;
  highNibble(data);
  delayNs(10);                                                                 // Min enable cycle time (>500ns)
  *((uint32_t *) &GPIO_CTRL->BSRR) = E << 16;

  /* Send low nibble */
  *((uint32_t *) &GPIO_CTRL->BSRR) = E;
  lowNibble(data);
  delayNs(10);
  *((uint32_t *) &GPIO_CTRL->BSRR) = E << 16;
}

/** @brief  Set direction of data bus pins
 *  @param  output: Indicates if GPIO is output                                 <br>
 *                  0: input                                                    <br>
 *                  1: output
 */
static void setDbOutput(bool output)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pins : DB4-DB7 */
  GPIO_InitStruct.Pin = DB4 | DB5 | DB6 | DB7;

  if (output)
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  else
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIO_DB, &GPIO_InitStruct);
}

/** @brief  Read busy flag
 */
static bool readBusy()
{
  bool is_busy;

  /* Set DB4-DB7 input */
  setDbOutput(false);

  /* Set RW and clear RS */
  *((uint32_t *) &GPIO_CTRL->BSRR) = RS << 16;
  *((uint32_t *) &GPIO_CTRL->BSRR) = RW;

  /* Read high nibble */
  *((uint32_t *) &GPIO_CTRL->BSRR) = E;
  is_busy = HAL_GPIO_ReadPin(GPIO_DB, DB7);
  delayNs(10);
  *((uint32_t *) &GPIO_CTRL->BSRR) = E << 16;

  /* Read low nibble */
  *((uint32_t *) &GPIO_CTRL->BSRR) = E;
  delayNs(10);
  *((uint32_t *) &GPIO_CTRL->BSRR) = E << 16;

  /* Set DB4-DB7 output */
  setDbOutput(true);

  /* Clear RW */
  *((uint32_t *) &GPIO_CTRL->BSRR) = RW << 16;

  return (is_busy);
}

/** @brief  Add task to hd44780_queue                                           <br>
 *  @param  data:   Data from task to add                                       <br>
 *  @param  is_cmd: Indicates if command or data is send
 *                  0: data                                                     <br>
 *                  1: command
 */
static void addTask(uint8_t data, bool is_cmd)
{
  /* Check if there still is space left in the circular buffer */
  if (qread != (qwrite + 1) % HD44780_QUEUE_SIZE)
  {
    hd44780_queue[qwrite].data = data;
    hd44780_queue[qwrite].is_cmd = is_cmd;
    qwrite = (qwrite + 1) % HD44780_QUEUE_SIZE;

  }
  else
  {
    /* Flush buffer and reset address */
    qread = 0;
    qwrite = 0;
  }
}

/** @brief  Delete task from hd44780_queue
 */
static void delTask()
{
  if (qread != qwrite)
    qread = (qread + 1) % HD44780_QUEUE_SIZE;

}

/** @brief  Execute task from hd44780_queue
 */
static void exec()
{
  static uint8_t error = 0;
  static bool ignore_busy = false;

  /* Only write if queue isn't empty */
  if (qread != qwrite)
  {
    if (!ignore_busy)
    {
      /* Check busy flag */
      if (!readBusy())
      {
        writeByte(hd44780_queue[qread].data, hd44780_queue[qread].is_cmd);
        delTask();
        error = 0;
      }
      else
      {
        if (error++ >= (HD44780_TIMEOUT / HD44780_TIM_PERIOD))
        {
          /* Ignore busy flag from now on */
          ignore_busy = true;

          /* Adjust timer period */
          htim_hd44780.Init.Period = HD44780_IGNORE_BUSY_PERIOD - 1;
          HAL_TIM_Base_Init(&htim_hd44780);
        }
      }
    }
    else
    {
      /* Now that busy flag isn't working any more we just send data blind */
      writeByte(hd44780_queue[qread].data, hd44780_queue[qread].is_cmd);
      delTask();
    }
  }
}

/** @brief  HD44780 Send single character to display
 *  @param  data:   Char to send
 */
void hd44780SendChar(uint8_t data)
{
  addTask(data, false);
}

/** @brief  HD44780 Send string to display
 *  @param  string: String to send
 */
void hd44780SendString(uint8_t* string)
{
  uint8_t i = 0;

  while (string[i])
  {
    addTask(string[i], false);
    i++;
  }
}

/** @brief  HD44780 Send command to display                                     <br>
 *  @param  display:    Display     ON/OFF
 *  @param  cursor:     Cursor      ON/OFF
 *  @param  blinking:   Blinking    ON/OFF
 */
void hd44780SendCmd(bool display, bool cursor, bool blinking)
{
  uint8_t cmd = 0;

  if (display)
    cmd |= CMD_ON;
  else
  {
    qread = 0;
    qwrite = 0;
  }

  if (cursor)
    cmd |= CMD_CURSOR_ON;

  if (blinking)
    cmd |= CMD_BLINKING_ON;

  addTask(cmd, true);
}

/** @brief  HD44780 Clear display
 */
void hd44780Clear()
{
  addTask(CMD_CLEAR, true);
}

/** @brief  HD44780 Set DDRAM address to 0
 */
void hd44780Home()
{
  addTask(CMD_RETURN_HOME, true);
}

/** @brief  HD44780 Set position                                                <br>
 *  @param  row:    Row to goto
 *  @param  col:    Column to goto
 */
void hd44780Goto(uint8_t row, uint8_t col)
{
  static const uint8_t offset[] =
  { 0x00, 0x40, 0x14, 0x54 };
  uint8_t cmd = HD44780_DDRAM;

  cmd |= col + offset[row];

  addTask(cmd, true);
}

/** @brief  HD44780 Define char
 */
static void hd44780DefineChars()
{
  uint8_t tilde[8] =
  { 0b00000, 0b00000, 0b00000, 0b01101, 0b10010, 0b00000, 0b00000, 0b00000 };

  uint8_t empty_rect[8] =
  { 0b11111, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111 };

  uint8_t full_rect[8] =
  { 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111 };

  uint8_t cw[8] =
  { 0b00000, 0b01110, 0b10001, 0b10000, 0b10111, 0b10011, 0b01101, 0b00000 };

  uint8_t ccw[8] =
  { 0b00000, 0b01110, 0b10001, 0b00001, 0b11101, 0b11001, 0b10110, 0b00000 };

  uint8_t brake[8] =
  { 0b11111, 0b10001, 0b10101, 0b10101, 0b10001, 0b10101, 0b10001, 0b11111 };

  uint8_t exp_m1[8] =
  { 0b00001, 0b00011, 0b11001, 0b00001, 0b00000, 0b00000, 0b00000, 0b00000 };

  addTask(0x40, true);
  addTask(tilde[0], false);
  addTask(tilde[1], false);
  addTask(tilde[2], false);
  addTask(tilde[3], false);
  addTask(tilde[4], false);
  addTask(tilde[5], false);
  addTask(tilde[6], false);
  addTask(tilde[7], false);

  addTask(empty_rect[0], false);
  addTask(empty_rect[1], false);
  addTask(empty_rect[2], false);
  addTask(empty_rect[3], false);
  addTask(empty_rect[4], false);
  addTask(empty_rect[5], false);
  addTask(empty_rect[6], false);
  addTask(empty_rect[7], false);

  addTask(full_rect[0], false);
  addTask(full_rect[1], false);
  addTask(full_rect[2], false);
  addTask(full_rect[3], false);
  addTask(full_rect[4], false);
  addTask(full_rect[5], false);
  addTask(full_rect[6], false);
  addTask(full_rect[7], false);

  addTask(cw[0], false);
  addTask(cw[1], false);
  addTask(cw[2], false);
  addTask(cw[3], false);
  addTask(cw[4], false);
  addTask(cw[5], false);
  addTask(cw[6], false);
  addTask(cw[7], false);

  addTask(ccw[0], false);
  addTask(ccw[1], false);
  addTask(ccw[2], false);
  addTask(ccw[3], false);
  addTask(ccw[4], false);
  addTask(ccw[5], false);
  addTask(ccw[6], false);
  addTask(ccw[7], false);

  addTask(brake[0], false);
  addTask(brake[1], false);
  addTask(brake[2], false);
  addTask(brake[3], false);
  addTask(brake[4], false);
  addTask(brake[5], false);
  addTask(brake[6], false);
  addTask(brake[7], false);

  addTask(exp_m1[0], false);
  addTask(exp_m1[1], false);
  addTask(exp_m1[2], false);
  addTask(exp_m1[3], false);
  addTask(exp_m1[4], false);
  addTask(exp_m1[5], false);
  addTask(exp_m1[6], false);
  addTask(exp_m1[7], false);
}

/** @brief  HD44780 initialization
 *  @return HAL status
 */
HAL_StatusTypeDef hd44780Init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pins : DB4-DB7 */
  setDbOutput(true);
  *((uint32_t *) &GPIO_DB->BSRR) = (DB4 | DB5 | DB6 | DB7) << 16;               // clear pins

  /* Configure GPIO pins : E, RW, RS */
  GPIO_InitStruct.Pin = E | RW | RS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIO_CTRL, &GPIO_InitStruct);
  *((uint32_t *) &GPIO_CTRL->BSRR) = (E | RW | RS) << 16;                       // clear pins

  /* 4-Bit initialization */
  HAL_Delay(40);                                                                // 40ms VCC rise time

  /* Issue 4bit interface */
  highNibble(0x30);

  /* Send 4bit interface cmd first time */
  *((uint32_t *) &GPIO_CTRL->BSRR) = E;
  delayNs(10);                                                                  // Min enable cycle time (>500ns)
  *((uint32_t *) &GPIO_CTRL->BSRR) = E << 16;
  HAL_Delay(5);                                                                 // 4.1ms

  /* Send 4bit interface cmd second time */
  *((uint32_t *) &GPIO_CTRL->BSRR) = E;
  delayNs(10);
  *((uint32_t *) &GPIO_CTRL->BSRR) = E << 16;
  HAL_Delay(1);

  /* Send 4bit interface cmd third time */
  *((uint32_t *) &GPIO_CTRL->BSRR) = E;
  delayNs(10);
  *((uint32_t *) &GPIO_CTRL->BSRR) = E << 16;
  HAL_Delay(1);

  /* Send commands (2x lines, display on, clear) */
  writeByte(0x28, CMD);
  HAL_Delay(2);
  writeByte(CMD_ON, CMD);
  HAL_Delay(2);
  writeByte(CMD_CLEAR, CMD);
  HAL_Delay(2);

  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(TIM_HD44780_IRQn, IRQ_PRIO_TIM_HD44780, 0);
  HAL_NVIC_EnableIRQ(TIM_HD44780_IRQn);

  /* Peripheral TIM_HD44780 init */
  htim_hd44780.Instance = TIM_HD44780;
  htim_hd44780.Init.Prescaler = (uint32_t) ((SystemCoreClock / 2)
                                / HD44780_TIM_CLK) - 1;                         // 1 MHz
  htim_hd44780.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_hd44780.Init.Period = HD44780_TIM_PERIOD - 1;                            // update rate CLK_CNT / (Period + 1)
  htim_hd44780.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim_hd44780) != HAL_OK)
    return HAL_ERROR;

  /* Enable update interrupt and timer */
  TIM_HD44780->DIER |= TIM_DIER_UIE;
  TIM_HD44780->CR1 |= TIM_CR1_CEN;

  /* Define custom chars */
  hd44780DefineChars();

  return HAL_OK;
}

/** @brief  HD44780 task initialization
 *  @return HAL status
 */
HAL_StatusTypeDef hd44780TaskInit()
{
  TIM_ClockConfigTypeDef sClockSourceConfig;

  /* Peripheral TIM_HD44780_TASK init */
  htim_hd44780_task.Instance = TIM_HD44780_TASK;
  htim_hd44780_task.Init.Prescaler = (uint32_t) ((SystemCoreClock / 2)
                                     / TIM_HD44780_TASK_CLK) - 1;               // 10 kHz;
  htim_hd44780_task.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_hd44780_task.Init.Period = TIM_HD44780_TASK_PERIOD - 1;                  // 0.5s
  htim_hd44780_task.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim_hd44780_task) != HAL_OK)
    return HAL_ERROR;

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim_hd44780_task, &sClockSourceConfig);

  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(TIM_HD44780_TASK_IRQn, IRQ_PRIO_TIM_HD44780_TASK, 0);
  HAL_NVIC_EnableIRQ(TIM_HD44780_TASK_IRQn);

  /* Enable update interrupt and counter */
  TIM_HD44780_TASK->DIER |= TIM_DIER_UIE;
  TIM_HD44780_TASK->CR1 |= TIM_CR1_CEN;

  return HAL_OK;
}

/** @brief  This function handles TIM_HD44780 global interrupt                  <br>
 *          (__weak version found in startup_stm32f405xx.s)
 */
void TIM_HD44780_IRQHandler()
{
  /* Update interrupt */
  if (TIM_HD44780->SR & TIM_SR_UIF)
  {
    /* Clear update interrupt flag */
    TIM_HD44780->SR &= ~TIM_SR_UIF;

    /* Execute task from hd44780_queue */
    exec();
  }
}

/** @brief  HD44780 task                                                        <br>
 *          (__weak version found in startup_stm32f405xx.s)
 */
void TIM_HD44780_TASK_IRQHandler()
{
  double tmp[3] = {};
  uint16_t n = 0;
  uint8_t array[20] = {};

  /* Update interrupt */
  if (TIM_HD44780_TASK->SR & TIM_SR_UIF)
  {
    /* Clear update interrupt flag */
    TIM_HD44780_TASK->SR &= ~TIM_SR_UIF;

    /* Blink LED to show that we're still operating */
    YELLOW_LED_TOGGLE();

    /* Headline */
    hd44780Goto(0, 0);
    hd44780SendString("DIY-VFD");

    /* Format rpm */
    hd44780Goto(0, 10);
    if (SW_SRC == PWM)
      n = ac.pwm.f_ref * 60;
    else if (SW_SRC == MAN)
      n = ac.man.f_ref * 60;
    sprintf(array, "RPM %d", n);
    hd44780SendString(array);
    hd44780SendChar(HD44780_ADR_EXP_M1);

    /* Format VBUS */
    hd44780Goto(1, 10);
    if (adc.vbus.si < 100)
    {
      if (adc.vbus.si < 10)
        sprintf(array, "VBUS    %dV", (uint16_t) adc.vbus.si);
      else
        sprintf(array, "VBUS   %dV", (uint16_t) adc.vbus.si);

    }
    else
      sprintf(array, "VBUS  %dV", (uint16_t) adc.vbus.si);
    hd44780SendString(array);

    /* Format phase currents */
    hd44780Goto(1, 7);
    hd44780SendChar(' ');
    hd44780Goto(2, 7);
    hd44780SendChar(' ');
    hd44780Goto(3, 7);
    hd44780SendChar(' ');
    hd44780Goto(1, 2);
    sprintf(array, "A %-0.1f", adc.imot.si[0]);
    hd44780SendString(array);
    hd44780Goto(2, 2);
    sprintf(array, "B %-0.1f", adc.imot.si[1]);
    hd44780SendString(array);
    hd44780Goto(3, 2);
    sprintf(array, "C %-0.1f", adc.imot.si[2]);
    hd44780SendString(array);
    hd44780Goto(1, 0);
    hd44780SendChar('I');
    hd44780Goto(1, 1);
    hd44780SendChar(HD44780_ADR_TILDE);
    hd44780Goto(2, 1);
    hd44780SendChar(HD44780_ADR_TILDE);
    hd44780Goto(3, 1);
    hd44780SendChar(HD44780_ADR_TILDE);
    hd44780Goto(1, 8);
    hd44780SendChar('A');
    hd44780Goto(2, 8);
    hd44780SendChar('A');
    hd44780Goto(3, 8);
    hd44780SendChar('A');

    /* Format direction */
    if (SW_SRC == PWM)
    {
      if (PC_DIR == CW)
      {
        hd44780Goto(2, 10);
        hd44780SendChar(HD44780_ADR_CW);
        hd44780SendString("DIR");
      }
      else if (PC_DIR == CCW)
      {
        hd44780Goto(2, 10);
        hd44780SendChar(HD44780_ADR_CCW);
        hd44780SendString("DIR");
      }
    }
    else if (SW_SRC == MAN)
    {
      if (SW_DIR == CW)
      {
        hd44780Goto(2, 10);
        hd44780SendChar(HD44780_ADR_CW);
        hd44780SendString("DIR");
      }
      else if (SW_DIR == CCW)
      {
        hd44780Goto(2, 10);
        hd44780SendChar(HD44780_ADR_CCW);
        hd44780SendString("DIR");
      }
    }

    /* Format motor on/off */
    hd44780Goto(2, 16);
    if (*ac.is_on && (&ac.is_on != 0))
      hd44780SendChar(HD44780_ADR_FULL_RECT);
    else
    {
      if (RED_LED_IS_ON())
        hd44780SendChar(HD44780_ADR_BRAKE);
      else
        hd44780SendChar(HD44780_ADR_EMPTY_RECT);
    }
    hd44780SendString("MOT");

    /* Format source or error */
    if (!ac.is_err)
    {
      hd44780Goto(3, 10);
      hd44780SendString(" PWM   MAN");
      hd44780Goto(3, 14);
      hd44780SendChar(':');
      hd44780SendChar(':');
      if (SW_SRC == PWM)
      {
        hd44780Goto(3, 10);
        hd44780SendChar(HD44780_ADR_FULL_RECT);
        hd44780Goto(3, 16);
        hd44780SendChar(HD44780_ADR_EMPTY_RECT);
      }
      else if (SW_SRC == MAN)
      {
        hd44780Goto(3, 10);
        hd44780SendChar(HD44780_ADR_EMPTY_RECT);
        hd44780Goto(3, 16);
        hd44780SendChar(HD44780_ADR_FULL_RECT);
      }
    }
    else
    {
      hd44780Goto(3, 10);
      hd44780SendString("Error!!!  ");
    }
  }
}
/** @}
 */
