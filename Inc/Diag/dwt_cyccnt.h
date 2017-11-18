/***************************************************************************//**
 *  @brief      Read DWT_CYCCNT register for instruction count analysis
 *
 *              This macros implement the CortexM3/M4 specific read of the
 *              DWT_CYCCNT register which enables the user to count core cycles
 *              over a short period of time. The technical reference manual
 *              mentions up to 50s on a 86MHz clock.
 *
 *              uint32_t it1, it2;                                              <br>
 *              float f = 1.01f;                                                <br>
 *              CORE_DWT_CYCCNT_EN();                                           <br>
 *              it1 = CORE_DWT_CYCCNT_GET();                                    <br>
 *              float f2 = f * 2.29f;                                           <br>
 *              it2 = CORE_DWT_CYCCNT_GET() - it1;                              <br>
 *              CORE_DWT_CYCCNT_DIS();                                          <br>
 *                                                                              <br>
 *              note: the calculations for it2 take 6 cycles
 *
 *  @file       dwt_cyccnt.h
 *  @version    1.0
 *  @author     Vincent Hamp
 *  @date       27/07/2015
 *  @addtogroup Diag
 *  @{
 ******************************************************************************/

#ifndef __DWT_CYCCNT_H
#define __DWT_CYCCNT_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Macros */
#define CORE_DWT_CYCCNT_EN()    *((volatile uint32_t*)0xE0001000) = 0x40000001  //!< Enable CYCCNT register
#define CORE_DWT_CYCCNT_DIS()   *((volatile uint32_t*)0xE0001000) = 0x40000000  //!< Disable CYCCNT register
#define CORE_DWT_CYCCNT_GET()   *((volatile uint32_t*)0xE0001004)               //!< Get value from CYCCNT register
#define CORE_DWT_CYCCNT_RES()   *((volatile uint32_t*)0xE0001004) = 0x00000000  //!< probably not possible and read only

#ifdef __cplusplus
}
#endif

#endif // __DWT_CYCCNT_H
/** @}
 */
