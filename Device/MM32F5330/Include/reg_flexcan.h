/***********************************************************************************************************************
    @file     reg_flexcan.h
    @author   VV TEAM
    @brief    This flie contains all the FLEXCAN's register and its field definition.
  **********************************************************************************************************************
    @attention

    <h2><center>&copy; Copyright(c) <2023> <MindMotion></center></h2>

      Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
    following conditions are met:
    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
       the following disclaimer in the documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
       promote products derived from this software without specific prior written permission.

      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
    INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *********************************************************************************************************************/

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/

#ifndef __REG_FLEXCAN_H
#define __REG_FLEXCAN_H


/* Files includes ----------------------------------------------------------------------------------------------------*/
#include "core_starmc1.h"



/**
  * @brief FLEXCAN Base Address Definition
  */
#define  FLEXCAN_BASE                              0x4000C000                                       /*!<Base Address: 0x4000C000*/

/**
  * @brief FLEXCAN Register Structure Definition
  */
typedef struct
{
    __IO uint32_t MCR;                            /*!<FLEXCAN Module Configuration Register          offset: 0x00       */
    __IO uint32_t CTRL1;                          /*!<FLEXCAN Control Register 1                     offset: 0x04       */
    __IO uint32_t TIMER;                          /*!<FLEXCAN Free Running Timer Register            offset: 0x08       */
    __IO uint32_t RESERVED0x0C[1];                /*!<                                               offset: 0x0C       */
    __IO uint32_t RXMGMASK;                       /*!<FLEXCAN Rx Mailboxes Global Mask Register      offset: 0x10       */
    __IO uint32_t RX14MASK;                       /*!<FLEXCAN Rx 14 Mask Registe                     offset: 0x14       */
    __IO uint32_t RX15MASK;                       /*!<FLEXCAN Rx 15 Mask Register                    offset: 0x18       */
    __IO uint32_t ECR;                            /*!<FLEXCAN Error Counter Register                 offset: 0x1C       */
    __IO uint32_t ESR1;                           /*!<FLEXCAN Error and Status Register 1            offset: 0x20       */
    __IO uint32_t RESERVED0x24[1];                /*!<                                               offset: 0x24       */
    __IO uint32_t IMASK1;                         /*!<FLEXCAN Interrupt Mask Register 1              offset: 0x28       */
    __IO uint32_t RESERVED0x2C[1];                /*!<                                               offset: 0x2C       */
    __IO uint32_t IFLAG1;                         /*!<FLEXCAN Interrupt Flag Register 1              offset: 0x30       */
    __IO uint32_t CTRL2;                          /*!<FLEXCAN Control Register 2                     offset: 0x34       */
    __IO uint32_t ESR2;                           /*!<FLEXCAN Error and Status Register 2            offset: 0x38       */
    __IO uint32_t RESERVED0x3C[2];                /*!<                                               offset: 0x3C~0x40  */
    __IO uint32_t CRCR;                           /*!<FLEXCAN CRC Register                           offset: 0x44       */
    __IO uint32_t RXFGMASK;                       /*!<FLEXCAN Rx FIFO Global Mask Register           offset: 0x48       */
    __IO uint32_t RXFIR;                          /*!<FLEXCAN Rx FIFO Information Register           offset: 0x4C       */
    __IO uint32_t CBT;                            /*!<FLEXCAN CAN Bit Timing Register                offset: 0x50       */
    __IO uint32_t RESERVED_4[11];

    struct                                        /*!< offset: 0x80 array step: 0x10 x 128bit */
    {
        __IO uint32_t CS;                         /*!< Message Buffer 0 CS Register..Message Buffer 15 CS Register, array offset: 0x80, array step: 0x10 */
        __IO uint32_t ID;                         /*!< Message Buffer 0 ID Register..Message Buffer 15 ID Register, array offset: 0x84, array step: 0x10 */
        __IO uint32_t WORD0;                      /*!< Message Buffer 0 WORD0 Register..Message Buffer 15 WORD0 Register, array offset: 0x88, array step: 0x10 */
        __IO uint32_t WORD1;                      /*!< Message Buffer 0 WORD1 Register..Message Buffer 15 WORD1 Register, array offset: 0x8C, array step: 0x10 */
    }MB[32];

    __IO uint32_t RESERVED_5[384];
    __IO uint32_t RXIMR[32];                      /*!<FLEXCAN Rx Individual Mask Registers           offset: 0x880      */
    __IO uint32_t RESERVED0x900[192];             /*!<                                               offset: 0x900~0xBFC*/
    __IO uint32_t FDCTRL;                         /*!<FLEXCAN CAN FD Control Register                offset: 0xC00      */
    __IO uint32_t FDCBT;                          /*!<FLEXCAN CAN FD Bit Timing Register             offset: 0xC04      */
    __IO uint32_t FDCRC;                          /*!<FLEXCAN CAN FD CRC Register                    offset: 0xC08      */
    __IO uint32_t ERFCR;                          /*!<FLEXCAN Enhanced Rx FIFO Control Register      offset: 0xC0C      */
    __IO uint32_t ERFIER;                         /*!<FLEXCAN Enhanced Rx FIFO Interrupt Enable      offset: 0xC10
                                                      Register                                                          */
    __IO uint32_t ERFSR;                          /*!<FLEXCAN Enhanced Rx FIFO Status Register       offset: 0xC14      */
    __IO uint32_t RESERVED0xC18[1274];                  /*!<                                                      offset: 0xC18~0x1FFC */                                    
    struct 
    {   
        __IO uint32_t ERFCS;                            /*!< Enhanced Rx FIFO Buffer 0 CS Register                offset: 0x2000 */
        __IO uint32_t ERFDID;                           /*!< Enhanced Rx FIFO Buffer 0 ID Register                offset: 0x2004 */
        __IO uint32_t ERFDWORD[16];                     /*!< Enhanced Rx FIFO Buffer 0 WORD Register              offset: 0x2008 */
        __IO uint32_t IHOFF;                            /*!< Identifier receive filter hit indication             offset: 0x2048 */
        __IO uint32_t RESERVED0x204C[1];                /*!<                                                      offset: 0x204C */
    }ERFMB[6];                                          /*!<                                                      offset: 0x2000~0x21DC */
    __IO uint32_t RESERVED0x21E0[904];                  /*!<                                                      offset: 0x21E0~0x2FFC */                                     
    __IO uint32_t ERFFEL[2];                      /*!<FLEXCAN Enhanced Rx FIFO Filter Element        offset: 0x3000
                                                      Registers                                                         */
} FLEXCAN_TypeDef;

/**
  * @brief FLEXCAN type pointer Definition
  */
#define FLEXCAN                                   ((FLEXCAN_TypeDef *)FLEXCAN_BASE)


/** @name MCR - Module Configuration Register
  * @{
  */
#define CAN_MCR_MAXMB_MASK                       (0x7FU)
#define CAN_MCR_MAXMB_SHIFT                      (0U)
#define CAN_MCR_MAXMB(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_MCR_MAXMB_SHIFT)) & CAN_MCR_MAXMB_MASK)
#define CAN_MCR_IDAM_MASK                        (0x300U)
#define CAN_MCR_IDAM_SHIFT                       (8U)
/* IDAM - ID Acceptance Mode */
/* 0b00..Format A: One full ID (standard and extended) per ID Filter Table element. */
/* 0b01..Format B: Two full standard IDs or two partial 14-bit (standard and extended) IDs per ID Filter Table element. */
/* 0b10..Format C: Four partial 8-bit Standard IDs per ID Filter Table element. */
/* 0b11..Format D: All frames rejected. */

#define CAN_MCR_IDAM(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_IDAM_SHIFT)) & CAN_MCR_IDAM_MASK)
#define CAN_MCR_FDEN_MASK                        (0x800U)
#define CAN_MCR_FDEN_SHIFT                       (11U)
/* FDEN - CAN FD Enable */
/* 0b0..CAN FD disabled. */
/* 0b1..CAN FD enabled. */

#define CAN_MCR_FDEN(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FDEN_SHIFT)) & CAN_MCR_FDEN_MASK)
#define CAN_MCR_AEN_MASK                         (0x1000U)
#define CAN_MCR_AEN_SHIFT                        (12U)
/* AEN - Abort Enable */
/* 0b0..Abort disabled. */
/* 0b1..Abort enabled. */

#define CAN_MCR_AEN(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_MCR_AEN_SHIFT)) & CAN_MCR_AEN_MASK)
#define CAN_MCR_LPRIOEN_MASK                     (0x2000U)
#define CAN_MCR_LPRIOEN_SHIFT                    (13U)
/* LPRIOEN - Local Priority Enable */
/* 0b0..Local Priority disabled. */
/* 0b1..Local Priority enabled. */

#define CAN_MCR_LPRIOEN(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_MCR_LPRIOEN_SHIFT)) & CAN_MCR_LPRIOEN_MASK)
#define CAN_MCR_DMA_MASK                         (0x8000U)
#define CAN_MCR_DMA_SHIFT                        (15U)
/* DMA - DMA Enable */
/* 0b0..DMA feature for RX FIFO disabled. */
/* 0b1..DMA feature for RX FIFO enabled. */

#define CAN_MCR_DMA(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_MCR_DMA_SHIFT)) & CAN_MCR_DMA_MASK)
#define CAN_MCR_IRMQ_MASK                        (0x10000U)
#define CAN_MCR_IRMQ_SHIFT                       (16U)
/* IRMQ - Individual Rx Masking And Queue Enable */
/* 0b0..Individual Rx masking and queue feature are disabled. For backward compatibility with legacy applications, the reading of C/S word locks the MB even if it is EMPTY. */
/* 0b1..Individual Rx masking and queue feature are enabled. */

#define CAN_MCR_IRMQ(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_IRMQ_SHIFT)) & CAN_MCR_IRMQ_MASK)
#define CAN_MCR_SRXDIS_MASK                      (0x20000U)
#define CAN_MCR_SRXDIS_SHIFT                     (17U)
/* SRXDIS - Self Reception Disable */
/* 0b0..Self reception enabled. */
/* 0b1..Self reception disabled. */

#define CAN_MCR_SRXDIS(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SRXDIS_SHIFT)) & CAN_MCR_SRXDIS_MASK)
#define CAN_MCR_WAKSRC_MASK                      (0x80000U)
#define CAN_MCR_WAKSRC_SHIFT                     (19U)
/* WAKSRC - Wake Up Source */
/* 0b0..FlexCAN uses the unfiltered Rx input to detect recessive to dominant edges on the CAN bus. */
/* 0b1..FlexCAN uses the filtered Rx input to detect recessive to dominant edges on the CAN bus. */

#define CAN_MCR_WAKSRC(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_WAKSRC_SHIFT)) & CAN_MCR_WAKSRC_MASK)
#define CAN_MCR_LPMACK_MASK                      (0x100000U)
#define CAN_MCR_LPMACK_SHIFT                     (20U)
/* LPMACK - Low-Power Mode Acknowledge */
/* 0b0..FlexCAN is not in a low-power mode. */
/* 0b1..FlexCAN is in a low-power mode. */

#define CAN_MCR_LPMACK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_LPMACK_SHIFT)) & CAN_MCR_LPMACK_MASK)
#define CAN_MCR_WRNEN_MASK                       (0x200000U)
#define CAN_MCR_WRNEN_SHIFT                      (21U)
/* WRNEN - Warning Interrupt Enable */
/* 0b0..TWRNINT and RWRNINT bits are zero, independent of the values in the error counters. */
/* 0b1..TWRNINT and RWRNINT bits are set when the respective error counter transitions from less than 96 to greater than or equal to 96. */

#define CAN_MCR_WRNEN(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_MCR_WRNEN_SHIFT)) & CAN_MCR_WRNEN_MASK)
#define CAN_MCR_SLFWAK_MASK                      (0x400000U)
#define CAN_MCR_SLFWAK_SHIFT                     (22U)
/* SLFWAK - Self Wake Up */
/* 0b0..FlexCAN Self Wake Up feature is disabled. */
/* 0b1..FlexCAN Self Wake Up feature is enabled. */

#define CAN_MCR_SLFWAK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SLFWAK_SHIFT)) & CAN_MCR_SLFWAK_MASK)
#define CAN_MCR_SUPV_MASK                        (0x800000U)
#define CAN_MCR_SUPV_SHIFT                       (23U)
/* SUPV - Supervisor Mode */
/* 0b0..FlexCAN is in User mode. Affected registers allow both Supervisor and Unrestricted accesses. */
/* 0b1..FlexCAN is in Supervisor mode. Affected registers allow only Supervisor access. Unrestricted access behaves as though the access was done to an unimplemented register location. */

#define CAN_MCR_SUPV(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SUPV_SHIFT)) & CAN_MCR_SUPV_MASK)
#define CAN_MCR_FRZACK_MASK                      (0x1000000U)
#define CAN_MCR_FRZACK_SHIFT                     (24U)
/* FRZACK - Freeze Mode Acknowledge */
/* 0b0..FlexCAN not in Freeze mode, prescaler running. */
/* 0b1..FlexCAN in Freeze mode, prescaler stopped. */

#define CAN_MCR_FRZACK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FRZACK_SHIFT)) & CAN_MCR_FRZACK_MASK)
#define CAN_MCR_SOFTRST_MASK                     (0x2000000U)
#define CAN_MCR_SOFTRST_SHIFT                    (25U)
/* SOFTRST - Soft Reset */
/* 0b0..No reset request. */
/* 0b1..Resets the registers affected by soft reset. */

#define CAN_MCR_SOFTRST(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SOFTRST_SHIFT)) & CAN_MCR_SOFTRST_MASK)
#define CAN_MCR_WAKMSK_MASK                      (0x4000000U)
#define CAN_MCR_WAKMSK_SHIFT                     (26U)
/* WAKMSK - Wake Up Interrupt Mask */
/* 0b0..Wake Up Interrupt is disabled. */
/* 0b1..Wake Up Interrupt is enabled. */

#define CAN_MCR_WAKMSK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_WAKMSK_SHIFT)) & CAN_MCR_WAKMSK_MASK)
#define CAN_MCR_NOTRDY_MASK                      (0x8000000U)
#define CAN_MCR_NOTRDY_SHIFT                     (27U)
/* NOTRDY - FlexCAN Not Ready */
/* 0b0..FlexCAN module is either in Normal mode, Listen-Only mode or Loop-Back mode. */
/* 0b1..FlexCAN module is either in Disable mode, Stop mode or Freeze mode. */

#define CAN_MCR_NOTRDY(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_NOTRDY_SHIFT)) & CAN_MCR_NOTRDY_MASK)
#define CAN_MCR_HALT_MASK                        (0x10000000U)
#define CAN_MCR_HALT_SHIFT                       (28U)
/* HALT - Halt FlexCAN */
/* 0b0..No Freeze mode request. */
/* 0b1..Enters Freeze mode if the FRZ bit is asserted. */

#define CAN_MCR_HALT(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_HALT_SHIFT)) & CAN_MCR_HALT_MASK)
#define CAN_MCR_RFEN_MASK                        (0x20000000U)
#define CAN_MCR_RFEN_SHIFT                       (29U)
/* RFEN - Rx FIFO Enable */
/* 0b0..Rx FIFO not enabled. */
/* 0b1..Rx FIFO enabled. */

#define CAN_MCR_RFEN(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_RFEN_SHIFT)) & CAN_MCR_RFEN_MASK)
#define CAN_MCR_FRZ_MASK                         (0x40000000U)
#define CAN_MCR_FRZ_SHIFT                        (30U)
/* FRZ - Freeze Enable */
/* 0b0..Not enabled to enter Freeze mode. */
/* 0b1..Enabled to enter Freeze mode. */

#define CAN_MCR_FRZ(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FRZ_SHIFT)) & CAN_MCR_FRZ_MASK)
#define CAN_MCR_MDIS_MASK                        (0x80000000U)
#define CAN_MCR_MDIS_SHIFT                       (31U)
/* MDIS - Module Disable */
/* 0b0..Enable the FlexCAN module. */
/* 0b1..Disable the FlexCAN module. */

#define CAN_MCR_MDIS(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_MDIS_SHIFT)) & CAN_MCR_MDIS_MASK)
/**
  * @}
  */

/** @name CTRL1 - Control 1 register
  * @{
  */
#define CAN_CTRL1_PROPSEG_MASK                   (0x7U)
#define CAN_CTRL1_PROPSEG_SHIFT                  (0U)
#define CAN_CTRL1_PROPSEG(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PROPSEG_SHIFT)) & CAN_CTRL1_PROPSEG_MASK)
#define CAN_CTRL1_LOM_MASK                       (0x8U)
#define CAN_CTRL1_LOM_SHIFT                      (3U)
/* LOM - Listen-Only Mode */
/* 0b0..Listen-Only mode is deactivated. */
/* 0b1..FlexCAN module operates in Listen-Only mode. */

#define CAN_CTRL1_LOM(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LOM_SHIFT)) & CAN_CTRL1_LOM_MASK)
#define CAN_CTRL1_LBUF_MASK                      (0x10U)
#define CAN_CTRL1_LBUF_SHIFT                     (4U)
/* LBUF - Lowest Buffer Transmitted First */
/* 0b0..Buffer with highest priority is transmitted first. */
/* 0b1..Lowest number buffer is transmitted first. */

#define CAN_CTRL1_LBUF(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LBUF_SHIFT)) & CAN_CTRL1_LBUF_MASK)
#define CAN_CTRL1_TSYN_MASK                      (0x20U)
#define CAN_CTRL1_TSYN_SHIFT                     (5U)
/* TSYN - Timer Sync */
/* 0b0..Timer Sync feature disabled */
/* 0b1..Timer Sync feature enabled */

#define CAN_CTRL1_TSYN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_TSYN_SHIFT)) & CAN_CTRL1_TSYN_MASK)
#define CAN_CTRL1_BOFFREC_MASK                   (0x40U)
#define CAN_CTRL1_BOFFREC_SHIFT                  (6U)
/* BOFFREC - Bus Off Recovery */
/* 0b0..Automatic recovering from Bus Off state enabled. */
/* 0b1..Automatic recovering from Bus Off state disabled. */

#define CAN_CTRL1_BOFFREC(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_BOFFREC_SHIFT)) & CAN_CTRL1_BOFFREC_MASK)
#define CAN_CTRL1_SMP_MASK                       (0x80U)
#define CAN_CTRL1_SMP_SHIFT                      (7U)
/* SMP - CAN Bit Sampling */
/* 0b0..Just one sample is used to determine the bit value. */
/* 0b1..Three samples are used to determine the value of the received bit: the regular one (sample point) and 2 preceding samples; a majority rule is used. */

#define CAN_CTRL1_SMP(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_SMP_SHIFT)) & CAN_CTRL1_SMP_MASK)
#define CAN_CTRL1_RWRNMSK_MASK                   (0x400U)
#define CAN_CTRL1_RWRNMSK_SHIFT                  (10U)
/* RWRNMSK - Rx Warning Interrupt Mask */
/* 0b0..Rx Warning Interrupt disabled. */
/* 0b1..Rx Warning Interrupt enabled. */

#define CAN_CTRL1_RWRNMSK(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_RWRNMSK_SHIFT)) & CAN_CTRL1_RWRNMSK_MASK)
#define CAN_CTRL1_TWRNMSK_MASK                   (0x800U)
#define CAN_CTRL1_TWRNMSK_SHIFT                  (11U)
/* TWRNMSK - Tx Warning Interrupt Mask */
/* 0b0..Tx Warning Interrupt disabled. */
/* 0b1..Tx Warning Interrupt enabled. */

#define CAN_CTRL1_TWRNMSK(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_TWRNMSK_SHIFT)) & CAN_CTRL1_TWRNMSK_MASK)
#define CAN_CTRL1_LPB_MASK                       (0x1000U)
#define CAN_CTRL1_LPB_SHIFT                      (12U)
/* LPB - Loop Back Mode */
/* 0b0..Loop Back disabled. */
/* 0b1..Loop Back enabled. */

#define CAN_CTRL1_LPB(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LPB_SHIFT)) & CAN_CTRL1_LPB_MASK)
#define CAN_CTRL1_CLKSRC_MASK                    (0x2000U)
#define CAN_CTRL1_CLKSRC_SHIFT                   (13U)
/* CLKSRC - CAN Engine Clock Source */
/* 0b0..The CAN engine clock source is the oscillator clock. Under this condition, the oscillator clock frequency must be lower than the bus clock. */
/* 0b1..The CAN engine clock source is the peripheral clock. */

#define CAN_CTRL1_CLKSRC(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_CLKSRC_SHIFT)) & CAN_CTRL1_CLKSRC_MASK)
#define CAN_CTRL1_ERRMSK_MASK                    (0x4000U)
#define CAN_CTRL1_ERRMSK_SHIFT                   (14U)
/* ERRMSK - Error Interrupt Mask */
/* 0b0..Error interrupt disabled. */
/* 0b1..Error interrupt enabled. */

#define CAN_CTRL1_ERRMSK(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_ERRMSK_SHIFT)) & CAN_CTRL1_ERRMSK_MASK)
#define CAN_CTRL1_BOFFMSK_MASK                   (0x8000U)
#define CAN_CTRL1_BOFFMSK_SHIFT                  (15U)
/* BOFFMSK - Bus Off Interrupt Mask */
/* 0b0..Bus Off interrupt disabled. */
/* 0b1..Bus Off interrupt enabled. */

#define CAN_CTRL1_BOFFMSK(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_BOFFMSK_SHIFT)) & CAN_CTRL1_BOFFMSK_MASK)
#define CAN_CTRL1_PSEG2_MASK                     (0x70000U)
#define CAN_CTRL1_PSEG2_SHIFT                    (16U)
#define CAN_CTRL1_PSEG2(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PSEG2_SHIFT)) & CAN_CTRL1_PSEG2_MASK)
#define CAN_CTRL1_PSEG1_MASK                     (0x380000U)
#define CAN_CTRL1_PSEG1_SHIFT                    (19U)
#define CAN_CTRL1_PSEG1(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PSEG1_SHIFT)) & CAN_CTRL1_PSEG1_MASK)
#define CAN_CTRL1_RJW_MASK                       (0xC00000U)
#define CAN_CTRL1_RJW_SHIFT                      (22U)
#define CAN_CTRL1_RJW(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_RJW_SHIFT)) & CAN_CTRL1_RJW_MASK)
#define CAN_CTRL1_PRESDIV_MASK                   (0xFF000000U)
#define CAN_CTRL1_PRESDIV_SHIFT                  (24U)
#define CAN_CTRL1_PRESDIV(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PRESDIV_SHIFT)) & CAN_CTRL1_PRESDIV_MASK)
/**
  * @}
  */

/** @name TIMER - Free Running Timer
  * @{
  */
#define CAN_TIMER_TIMER_MASK                     (0xFFFFU)
#define CAN_TIMER_TIMER_SHIFT                    (0U)
#define CAN_TIMER_TIMER(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_TIMER_TIMER_SHIFT)) & CAN_TIMER_TIMER_MASK)
/**
  * @}
  */

/** @name RXMGMASK - Rx Mailboxes Global Mask Register
  * @{
  */
#define CAN_RXMGMASK_MG_MASK                     (0xFFFFFFFFU)
#define CAN_RXMGMASK_MG_SHIFT                    (0U)
/* MG - Rx Mailboxes Global Mask Bits */
/* 0b00000000000000000000000000000000..The corresponding bit in the filter is "don't care." */
/* 0b00000000000000000000000000000001..The corresponding bit in the filter is checked. */

#define CAN_RXMGMASK_MG(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_RXMGMASK_MG_SHIFT)) & CAN_RXMGMASK_MG_MASK)
/**
  * @}
  */

/** @name RX14MASK - Rx 14 Mask register
  * @{
  */
#define CAN_RX14MASK_RX14M_MASK                  (0xFFFFFFFFU)
#define CAN_RX14MASK_RX14M_SHIFT                 (0U)
/* RX14M - Rx Buffer 14 Mask Bits */
/* 0b00000000000000000000000000000000..The corresponding bit in the filter is "don't care." */
/* 0b00000000000000000000000000000001..The corresponding bit in the filter is checked. */

#define CAN_RX14MASK_RX14M(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_RX14MASK_RX14M_SHIFT)) & CAN_RX14MASK_RX14M_MASK)
/**
  * @}
  */

/** @name RX15MASK - Rx 15 Mask register
  * @{
  */
#define CAN_RX15MASK_RX15M_MASK                  (0xFFFFFFFFU)
#define CAN_RX15MASK_RX15M_SHIFT                 (0U)
/* RX15M - Rx Buffer 15 Mask Bits */
/* 0b00000000000000000000000000000000..The corresponding bit in the filter is "don't care." */
/* 0b00000000000000000000000000000001..The corresponding bit in the filter is checked. */

#define CAN_RX15MASK_RX15M(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_RX15MASK_RX15M_SHIFT)) & CAN_RX15MASK_RX15M_MASK)
/**
  * @}
  */

/** @name ECR - Error Counter
  * @{
  */
#define CAN_ECR_TXERRCNT_MASK                    (0xFFU)
#define CAN_ECR_TXERRCNT_SHIFT                   (0U)
#define CAN_ECR_TXERRCNT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ECR_TXERRCNT_SHIFT)) & CAN_ECR_TXERRCNT_MASK)
#define CAN_ECR_RXERRCNT_MASK                    (0xFF00U)
#define CAN_ECR_RXERRCNT_SHIFT                   (8U)
#define CAN_ECR_RXERRCNT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ECR_RXERRCNT_SHIFT)) & CAN_ECR_RXERRCNT_MASK)
/**
  * @}
  */

/** @name ESR1 - Error and Status 1 register
  * @{
  */
#define CAN_ESR1_WAKINT_MASK                     (0x1U)
#define CAN_ESR1_WAKINT_SHIFT                    (0U)
/* WAKINT - Wake-Up Interrupt */
/* 0b0..No such occurrence. */
/* 0b1..Indicates a recessive to dominant transition was received on the CAN bus. */

#define CAN_ESR1_WAKINT(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_WAKINT_SHIFT)) & CAN_ESR1_WAKINT_MASK)
#define CAN_ESR1_ERRINT_MASK                     (0x2U)
#define CAN_ESR1_ERRINT_SHIFT                    (1U)
/* ERRINT - Error Interrupt */
/* 0b0..No such occurrence. */
/* 0b1..Indicates setting of any Error Bit in the Error and Status Register. */

#define CAN_ESR1_ERRINT(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERRINT_SHIFT)) & CAN_ESR1_ERRINT_MASK)
#define CAN_ESR1_BOFFINT_MASK                    (0x4U)
#define CAN_ESR1_BOFFINT_SHIFT                   (2U)
/* BOFFINT - Bus Off Interrupt */
/* 0b0..No such occurrence. */
/* 0b1..FlexCAN module entered Bus Off state. */

#define CAN_ESR1_BOFFINT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BOFFINT_SHIFT)) & CAN_ESR1_BOFFINT_MASK)
#define CAN_ESR1_RX_MASK                         (0x8U)
#define CAN_ESR1_RX_SHIFT                        (3U)
/* RX - FlexCAN In Reception */
/* 0b0..FlexCAN is not receiving a message. */
/* 0b1..FlexCAN is receiving a message. */

#define CAN_ESR1_RX(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RX_SHIFT)) & CAN_ESR1_RX_MASK)
#define CAN_ESR1_FLTCONF_MASK                    (0x30U)
#define CAN_ESR1_FLTCONF_SHIFT                   (4U)
/* FLTCONF - Fault Confinement State */
/* 0b00..Error Active */
/* 0b01..Error Passive */
/* 0b1x..Bus Off */

#define CAN_ESR1_FLTCONF(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FLTCONF_SHIFT)) & CAN_ESR1_FLTCONF_MASK)
#define CAN_ESR1_TX_MASK                         (0x40U)
#define CAN_ESR1_TX_SHIFT                        (6U)
/* TX - FlexCAN In Transmission */
/* 0b0..FlexCAN is not transmitting a message. */
/* 0b1..FlexCAN is transmitting a message. */

#define CAN_ESR1_TX(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TX_SHIFT)) & CAN_ESR1_TX_MASK)
#define CAN_ESR1_IDLE_MASK                       (0x80U)
#define CAN_ESR1_IDLE_SHIFT                      (7U)
/* IDLE */
/* 0b0..No such occurrence. */
/* 0b1..CAN bus is now IDLE. */

#define CAN_ESR1_IDLE(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_IDLE_SHIFT)) & CAN_ESR1_IDLE_MASK)
#define CAN_ESR1_RXWRN_MASK                      (0x100U)
#define CAN_ESR1_RXWRN_SHIFT                     (8U)
/* RXWRN - Rx Error Warning */
/* 0b0..No such occurrence. */
/* 0b1..RXERRCNT is greater than or equal to 96. */

#define CAN_ESR1_RXWRN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RXWRN_SHIFT)) & CAN_ESR1_RXWRN_MASK)
#define CAN_ESR1_TXWRN_MASK                      (0x200U)
#define CAN_ESR1_TXWRN_SHIFT                     (9U)
/* TXWRN - TX Error Warning */
/* 0b0..No such occurrence. */
/* 0b1..TXERRCNT is greater than or equal to 96. */

#define CAN_ESR1_TXWRN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TXWRN_SHIFT)) & CAN_ESR1_TXWRN_MASK)
#define CAN_ESR1_STFERR_MASK                     (0x400U)
#define CAN_ESR1_STFERR_SHIFT                    (10U)
/* STFERR - Stuffing Error */
/* 0b0..No such occurrence. */
/* 0b1..A Stuffing Error occurred since last read of this register. */

#define CAN_ESR1_STFERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_STFERR_SHIFT)) & CAN_ESR1_STFERR_MASK)
#define CAN_ESR1_FRMERR_MASK                     (0x800U)
#define CAN_ESR1_FRMERR_SHIFT                    (11U)
/* FRMERR - Form Error */
/* 0b0..No such occurrence. */
/* 0b1..A Form Error occurred since last read of this register. */

#define CAN_ESR1_FRMERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FRMERR_SHIFT)) & CAN_ESR1_FRMERR_MASK)
#define CAN_ESR1_CRCERR_MASK                     (0x1000U)
#define CAN_ESR1_CRCERR_SHIFT                    (12U)
/* CRCERR - Cyclic Redundancy Check Error */
/* 0b0..No such occurrence. */
/* 0b1..A CRC error occurred since last read of this register. */

#define CAN_ESR1_CRCERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_CRCERR_SHIFT)) & CAN_ESR1_CRCERR_MASK)
#define CAN_ESR1_ACKERR_MASK                     (0x2000U)
#define CAN_ESR1_ACKERR_SHIFT                    (13U)
/* ACKERR - Acknowledge Error */
/* 0b0..No such occurrence. */
/* 0b1..An ACK error occurred since last read of this register. */

#define CAN_ESR1_ACKERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ACKERR_SHIFT)) & CAN_ESR1_ACKERR_MASK)
#define CAN_ESR1_BIT0ERR_MASK                    (0x4000U)
#define CAN_ESR1_BIT0ERR_SHIFT                   (14U)
/* BIT0ERR - Bit0 Error */
/* 0b0..No such occurrence. */
/* 0b1..At least one bit sent as dominant is received as recessive. */

#define CAN_ESR1_BIT0ERR(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT0ERR_SHIFT)) & CAN_ESR1_BIT0ERR_MASK)
#define CAN_ESR1_BIT1ERR_MASK                    (0x8000U)
#define CAN_ESR1_BIT1ERR_SHIFT                   (15U)
/* BIT1ERR - Bit1 Error */
/* 0b0..No such occurrence. */
/* 0b1..At least one bit sent as recessive is received as dominant. */

#define CAN_ESR1_BIT1ERR(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT1ERR_SHIFT)) & CAN_ESR1_BIT1ERR_MASK)
#define CAN_ESR1_RWRNINT_MASK                    (0x10000U)
#define CAN_ESR1_RWRNINT_SHIFT                   (16U)
/* RWRNINT - Rx Warning Interrupt Flag */
/* 0b0..No such occurrence. */
/* 0b1..The Rx error counter transitioned from less than 96 to greater than or equal to 96. */

#define CAN_ESR1_RWRNINT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RWRNINT_SHIFT)) & CAN_ESR1_RWRNINT_MASK)
#define CAN_ESR1_TWRNINT_MASK                    (0x20000U)
#define CAN_ESR1_TWRNINT_SHIFT                   (17U)
/* TWRNINT - Tx Warning Interrupt Flag */
/* 0b0..No such occurrence. */
/* 0b1..The Tx error counter transitioned from less than 96 to greater than or equal to 96. */

#define CAN_ESR1_TWRNINT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TWRNINT_SHIFT)) & CAN_ESR1_TWRNINT_MASK)
#define CAN_ESR1_SYNCH_MASK                      (0x40000U)
#define CAN_ESR1_SYNCH_SHIFT                     (18U)
/* SYNCH - CAN Synchronization Status */
/* 0b0..FlexCAN is not synchronized to the CAN bus. */
/* 0b1..FlexCAN is synchronized to the CAN bus. */

#define CAN_ESR1_SYNCH(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_SYNCH_SHIFT)) & CAN_ESR1_SYNCH_MASK)
#define CAN_ESR1_BOFFDONEINT_MASK                (0x80000U)
#define CAN_ESR1_BOFFDONEINT_SHIFT               (19U)
/* BOFFDONEINT - Bus Off Done Interrupt */
/* 0b0..No such occurrence. */
/* 0b1..FlexCAN module has completed Bus Off process. */

#define CAN_ESR1_BOFFDONEINT(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BOFFDONEINT_SHIFT)) & CAN_ESR1_BOFFDONEINT_MASK)
#define CAN_ESR1_ERROVR_MASK                     (0x200000U)
#define CAN_ESR1_ERROVR_SHIFT                    (21U)
/* ERROVR - Error Overrun bit */
/* 0b0..Overrun has not occurred. */
/* 0b1..Overrun has occurred. */

#define CAN_ESR1_ERROVR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERROVR_SHIFT)) & CAN_ESR1_ERROVR_MASK)
/**
  * @}
  */

/** @name IMASK1 - Interrupt Masks 1 register
  * @{
  */
#define CAN_IMASK1_BUF31TO0M_MASK                (0xFFFFFFFFU)
#define CAN_IMASK1_BUF31TO0M_SHIFT               (0U)
/* BUF31TO0M - Buffer MB i Mask */
/* 0b00000000000000000000000000000000..The corresponding buffer Interrupt is disabled. */
/* 0b00000000000000000000000000000001..The corresponding buffer Interrupt is enabled. */

#define CAN_IMASK1_BUF31TO0M(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_IMASK1_BUF31TO0M_SHIFT)) & CAN_IMASK1_BUF31TO0M_MASK)
/**
  * @}
  */

/** @name IFLAG1 - Interrupt Flags 1 register
  * @{
  */
#define CAN_IFLAG1_BUF0I_MASK                    (0x1U)
#define CAN_IFLAG1_BUF0I_SHIFT                   (0U)
/* BUF0I - Buffer MB0 Interrupt Or Clear FIFO bit */
/* 0b0..The corresponding buffer has no occurrence of successfully completed transmission or reception when MCR[RFEN]=0. */
/* 0b1..The corresponding buffer has successfully completed transmission or reception when MCR[RFEN]=0. */

#define CAN_IFLAG1_BUF0I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF0I_SHIFT)) & CAN_IFLAG1_BUF0I_MASK)
#define CAN_IFLAG1_BUF4TO1I_MASK                 (0x1EU)
#define CAN_IFLAG1_BUF4TO1I_SHIFT                (1U)
/* BUF4TO1I - Buffer MB i Interrupt Or "reserved" */
/* 0b0000..The corresponding buffer has no occurrence of successfully completed transmission or reception when MCR[RFEN]=0. */
/* 0b0001..The corresponding buffer has successfully completed transmission or reception when MCR[RFEN]=0. */

#define CAN_IFLAG1_BUF4TO1I(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF4TO1I_SHIFT)) & CAN_IFLAG1_BUF4TO1I_MASK)
#define CAN_IFLAG1_BUF5I_MASK                    (0x20U)
#define CAN_IFLAG1_BUF5I_SHIFT                   (5U)
/* BUF5I - Buffer MB5 Interrupt Or "Frames available in Rx FIFO" */
/* 0b0..No occurrence of MB5 completing transmission/reception when MCR[RFEN]=0, or of frame(s) available in the FIFO, when MCR[RFEN]=1 */
/* 0b1..MB5 completed transmission/reception when MCR[RFEN]=0, or frame(s) available in the Rx FIFO when MCR[RFEN]=1. It generates a DMA request in case of MCR[RFEN] and MCR[DMA] are enabled. */

#define CAN_IFLAG1_BUF5I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF5I_SHIFT)) & CAN_IFLAG1_BUF5I_MASK)
#define CAN_IFLAG1_BUF6I_MASK                    (0x40U)
#define CAN_IFLAG1_BUF6I_SHIFT                   (6U)
/* BUF6I - Buffer MB6 Interrupt Or "Rx FIFO Warning" */
/* 0b0..No occurrence of MB6 completing transmission/reception when MCR[RFEN]=0, or of Rx FIFO almost full when MCR[RFEN]=1 */
/* 0b1..MB6 completed transmission/reception when MCR[RFEN]=0, or Rx FIFO almost full when MCR[RFEN]=1 */

#define CAN_IFLAG1_BUF6I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF6I_SHIFT)) & CAN_IFLAG1_BUF6I_MASK)
#define CAN_IFLAG1_BUF7I_MASK                    (0x80U)
#define CAN_IFLAG1_BUF7I_SHIFT                   (7U)
/* BUF7I - Buffer MB7 Interrupt Or "Rx FIFO Overflow" */
/* 0b0..No occurrence of MB7 completing transmission/reception when MCR[RFEN]=0, or of Rx FIFO overflow when MCR[RFEN]=1 */
/* 0b1..MB7 completed transmission/reception when MCR[RFEN]=0, or Rx FIFO overflow when MCR[RFEN]=1 */

#define CAN_IFLAG1_BUF7I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF7I_SHIFT)) & CAN_IFLAG1_BUF7I_MASK)
#define CAN_IFLAG1_BUF31TO8I_MASK                (0xFFFFFF00U)
#define CAN_IFLAG1_BUF31TO8I_SHIFT               (8U)
/* BUF31TO8I - Buffer MBi Interrupt */
/* 0b000000000000000000000000..The corresponding buffer has no occurrence of successfully completed transmission or reception. */
/* 0b000000000000000000000001..The corresponding buffer has successfully completed transmission or reception. */

#define CAN_IFLAG1_BUF31TO8I(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF31TO8I_SHIFT)) & CAN_IFLAG1_BUF31TO8I_MASK)
/**
  * @}
  */

/** @name CTRL2 - Control 2 register
  * @{
  */
#define CAN_CTRL2_EDFLTDIS_MASK                  (0x800U)
#define CAN_CTRL2_EDFLTDIS_SHIFT                 (11U)
/* EDFLTDIS - Edge Filter Disable */
/* 0b0..Edge Filter is enabled. */
/* 0b1..Edge Filter is disabled. */

#define CAN_CTRL2_EDFLTDIS(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_EDFLTDIS_SHIFT)) & CAN_CTRL2_EDFLTDIS_MASK)
#define CAN_CTRL2_ISOCANFDEN_MASK                (0x1000U)
#define CAN_CTRL2_ISOCANFDEN_SHIFT               (12U)
/* ISOCANFDEN - ISO CAN FD Enable */
/* 0b0..Stuff Count feature is disabled. Stuff Count bit field is not inserted before the CRC Sequence field. */
/* 0b1..Stuff Count feature is enabled. Stuff Count bit field is inserted before the CRC Sequence field. */

#define CAN_CTRL2_ISOCANFDEN(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_ISOCANFDEN_SHIFT)) & CAN_CTRL2_ISOCANFDEN_MASK)
#define CAN_CTRL2_PREXCEN_MASK                   (0x4000U)
#define CAN_CTRL2_PREXCEN_SHIFT                  (14U)
/* PREXCEN - Protocol Exception Enable */
/* 0b0..Protocol Exception is disabled. */
/* 0b1..Protocol Exception is enabled. */

#define CAN_CTRL2_PREXCEN(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_PREXCEN_SHIFT)) & CAN_CTRL2_PREXCEN_MASK)
#define CAN_CTRL2_TIMER_SRC_MASK                 (0x8000U)
#define CAN_CTRL2_TIMER_SRC_SHIFT                (15U)
/* TIMER_SRC - Timer Source */
/* 0b0..The Free Running Timer is clocked by the CAN bit clock, which defines the baud rate on the CAN bus. */
/* 0b1..The Free Running Timer is clocked by an external time tick. The period can be either adjusted to be equal to the baud rate on the CAN bus, or a different value as required. See the device specific section for details about the external time tick. */

#define CAN_CTRL2_TIMER_SRC(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_TIMER_SRC_SHIFT)) & CAN_CTRL2_TIMER_SRC_MASK)
#define CAN_CTRL2_EACEN_MASK                     (0x10000U)
#define CAN_CTRL2_EACEN_SHIFT                    (16U)
/* EACEN - Entire Frame Arbitration Field Comparison Enable For Rx Mailboxes */
/* 0b0..Rx Mailbox filter's IDE bit is always compared and RTR is never compared despite mask bits. */
/* 0b1..Enables the comparison of both Rx Mailbox filter's IDE and RTR bit with their corresponding bits within the incoming frame. Mask bits do apply. */

#define CAN_CTRL2_EACEN(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_EACEN_SHIFT)) & CAN_CTRL2_EACEN_MASK)
#define CAN_CTRL2_RRS_MASK                       (0x20000U)
#define CAN_CTRL2_RRS_SHIFT                      (17U)
/* RRS - Remote Request Storing */
/* 0b0..Remote Response Frame is generated. */
/* 0b1..Remote Request Frame is stored. */

#define CAN_CTRL2_RRS(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_RRS_SHIFT)) & CAN_CTRL2_RRS_MASK)
#define CAN_CTRL2_MRP_MASK                       (0x40000U)
#define CAN_CTRL2_MRP_SHIFT                      (18U)
/* MRP - Mailboxes Reception Priority */
/* 0b0..Matching starts from Rx FIFO and continues on Mailboxes. */
/* 0b1..Matching starts from Mailboxes and continues on Rx FIFO. */

#define CAN_CTRL2_MRP(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_MRP_SHIFT)) & CAN_CTRL2_MRP_MASK)
#define CAN_CTRL2_TASD_MASK                      (0xF80000U)
#define CAN_CTRL2_TASD_SHIFT                     (19U)
#define CAN_CTRL2_TASD(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_TASD_SHIFT)) & CAN_CTRL2_TASD_MASK)
#define CAN_CTRL2_RFFN_MASK                      (0xF000000U)
#define CAN_CTRL2_RFFN_SHIFT                     (24U)
#define CAN_CTRL2_RFFN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_RFFN_SHIFT)) & CAN_CTRL2_RFFN_MASK)
#define CAN_CTRL2_BOFFDONEMSK_MASK               (0x40000000U)
#define CAN_CTRL2_BOFFDONEMSK_SHIFT              (30U)
/* BOFFDONEMSK - Bus Off Done Interrupt Mask */
/* 0b0..Bus Off Done interrupt disabled. */
/* 0b1..Bus Off Done interrupt enabled. */

#define CAN_CTRL2_BOFFDONEMSK(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_BOFFDONEMSK_SHIFT)) & CAN_CTRL2_BOFFDONEMSK_MASK)
/**
  * @}
  */

/** @name ESR2 - Error and Status 2 register
  * @{
  */
#define CAN_ESR2_IMB_MASK                        (0x2000U)
#define CAN_ESR2_IMB_SHIFT                       (13U)
/* IMB - Inactive Mailbox */
/* 0b0..If ESR2[VPS] is asserted, the ESR2[LPTM] is not an inactive Mailbox. */
/* 0b1..If ESR2[VPS] is asserted, there is at least one inactive Mailbox. LPTM content is the number of the first one. */

#define CAN_ESR2_IMB(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_IMB_SHIFT)) & CAN_ESR2_IMB_MASK)
#define CAN_ESR2_VPS_MASK                        (0x4000U)
#define CAN_ESR2_VPS_SHIFT                       (14U)
/* VPS - Valid Priority Status */
/* 0b0..Contents of IMB and LPTM are invalid. */
/* 0b1..Contents of IMB and LPTM are valid. */

#define CAN_ESR2_VPS(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_VPS_SHIFT)) & CAN_ESR2_VPS_MASK)
#define CAN_ESR2_LPTM_MASK                       (0x7F0000U)
#define CAN_ESR2_LPTM_SHIFT                      (16U)
#define CAN_ESR2_LPTM(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_LPTM_SHIFT)) & CAN_ESR2_LPTM_MASK)
/**
  * @}
  */

/** @name CRCR - CRC Register
  * @{
  */
#define CAN_CRCR_TXCRC_MASK                      (0x7FFFU)
#define CAN_CRCR_TXCRC_SHIFT                     (0U)
#define CAN_CRCR_TXCRC(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CRCR_TXCRC_SHIFT)) & CAN_CRCR_TXCRC_MASK)
#define CAN_CRCR_MBCRC_MASK                      (0x7F0000U)
#define CAN_CRCR_MBCRC_SHIFT                     (16U)
#define CAN_CRCR_MBCRC(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CRCR_MBCRC_SHIFT)) & CAN_CRCR_MBCRC_MASK)
/**
  * @}
  */

/** @name RXFGMASK - Rx FIFO Global Mask register
  * @{
  */
#define CAN_RXFGMASK_FGM_MASK                    (0xFFFFFFFFU)
#define CAN_RXFGMASK_FGM_SHIFT                   (0U)
/* FGM - Rx FIFO Global Mask Bits */
/* 0b00000000000000000000000000000000..The corresponding bit in the filter is "don't care." */
/* 0b00000000000000000000000000000001..The corresponding bit in the filter is checked. */

#define CAN_RXFGMASK_FGM(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_RXFGMASK_FGM_SHIFT)) & CAN_RXFGMASK_FGM_MASK)
/**
  * @}
  */

/** @name RXFIR - Rx FIFO Information Register
  * @{
  */
#define CAN_RXFIR_IDHIT_MASK                     (0x1FFU)
#define CAN_RXFIR_IDHIT_SHIFT                    (0U)
#define CAN_RXFIR_IDHIT(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_RXFIR_IDHIT_SHIFT)) & CAN_RXFIR_IDHIT_MASK)
/**
  * @}
  */

/** @name CBT - CAN Bit Timing Register
  * @{
  */
#define CAN_CBT_EPSEG2_MASK                      (0x1FU)
#define CAN_CBT_EPSEG2_SHIFT                     (0U)
#define CAN_CBT_EPSEG2(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPSEG2_SHIFT)) & CAN_CBT_EPSEG2_MASK)
#define CAN_CBT_EPSEG1_MASK                      (0x3E0U)
#define CAN_CBT_EPSEG1_SHIFT                     (5U)
#define CAN_CBT_EPSEG1(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPSEG1_SHIFT)) & CAN_CBT_EPSEG1_MASK)
#define CAN_CBT_EPROPSEG_MASK                    (0xFC00U)
#define CAN_CBT_EPROPSEG_SHIFT                   (10U)
#define CAN_CBT_EPROPSEG(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPROPSEG_SHIFT)) & CAN_CBT_EPROPSEG_MASK)
#define CAN_CBT_ERJW_MASK                        (0x1F0000U)
#define CAN_CBT_ERJW_SHIFT                       (16U)
#define CAN_CBT_ERJW(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_CBT_ERJW_SHIFT)) & CAN_CBT_ERJW_MASK)
#define CAN_CBT_EPRESDIV_MASK                    (0x7FE00000U)
#define CAN_CBT_EPRESDIV_SHIFT                   (21U)
#define CAN_CBT_EPRESDIV(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPRESDIV_SHIFT)) & CAN_CBT_EPRESDIV_MASK)
#define CAN_CBT_BTF_MASK                         (0x80000000U)
#define CAN_CBT_BTF_SHIFT                        (31U)
/* BTF - Bit Timing Format Enable */
/* 0b0..Extended bit time definitions disabled. */
/* 0b1..Extended bit time definitions enabled. */

#define CAN_CBT_BTF(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_CBT_BTF_SHIFT)) & CAN_CBT_BTF_MASK)
/**
  * @}
  */

/** @name CS - Message Buffer 0 CS Register..Message Buffer 63 CS Register
  * @{
  */
#define CAN_CS_TIME_STAMP_MASK                   (0xFFFFU)
#define CAN_CS_TIME_STAMP_SHIFT                  (0U)
#define CAN_CS_TIME_STAMP(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CS_TIME_STAMP_SHIFT)) & CAN_CS_TIME_STAMP_MASK)
#define CAN_CS_DLC_MASK                          (0xF0000U)
#define CAN_CS_DLC_SHIFT                         (16U)
#define CAN_CS_DLC(x)                            (((uint32_t)(((uint32_t)(x)) << CAN_CS_DLC_SHIFT)) & CAN_CS_DLC_MASK)
#define CAN_CS_RTR_MASK                          (0x100000U)
#define CAN_CS_RTR_SHIFT                         (20U)
#define CAN_CS_RTR(x)                            (((uint32_t)(((uint32_t)(x)) << CAN_CS_RTR_SHIFT)) & CAN_CS_RTR_MASK)
#define CAN_CS_IDE_MASK                          (0x200000U)
#define CAN_CS_IDE_SHIFT                         (21U)
#define CAN_CS_IDE(x)                            (((uint32_t)(((uint32_t)(x)) << CAN_CS_IDE_SHIFT)) & CAN_CS_IDE_MASK)
#define CAN_CS_SRR_MASK                          (0x400000U)
#define CAN_CS_SRR_SHIFT                         (22U)
#define CAN_CS_SRR(x)                            (((uint32_t)(((uint32_t)(x)) << CAN_CS_SRR_SHIFT)) & CAN_CS_SRR_MASK)
#define CAN_CS_CODE_MASK                         (0x0F000000U)
#define CAN_CS_CODE_SHIFT                        (24U)
#define CAN_CS_CODE(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_CS_CODE_SHIFT)) & CAN_CS_CODE_MASK)

#define CAN_CS_ESI_MASK                          (0x20000000U)
#define CAN_CS_ESI_SHIFT                         (29U)
#define CAN_CS_ESI(x)                            (((uint32_t)(((uint32_t)(x)) << CAN_CS_ESI_SHIFT)) & CAN_CS_ESI_MASK)
#define CAN_CS_BRS_MASK                          (0x40000000U)
#define CAN_CS_BRS_SHIFT                         (30U)
/* BRS - Bit Rate Switch. This bit defines whether the bit rate is switched inside a CAN FD format frame. */
#define CAN_CS_BRS(x)                            (((uint32_t)(((uint32_t)(x)) << CAN_CS_BRS_SHIFT)) & CAN_CS_BRS_MASK)
#define CAN_CS_EDL_MASK                          (0x80000000U)
#define CAN_CS_EDL_SHIFT                         (31U)
/* EDL - Extended Data Length. This bit distinguishes between CAN format and CAN FD format frames.
   The EDL bit must not be set for Message Buffers configured to RANSWER with code field 0b1010. */
#define CAN_CS_EDL(x)                            (((uint32_t)(((uint32_t)(x)) << CAN_CS_EDL_SHIFT)) & CAN_CS_EDL_MASK)

/**
  * @}
  */

/* The count of CAN_CS */
#define CAN_CS_COUNT                             (32U)

/** @name ID - Message Buffer 0 ID Register..Message Buffer 63 ID Register
  * @{
  */
#define CAN_ID_EXT_MASK                          (0x3FFFFU)
#define CAN_ID_EXT_SHIFT                         (0U)
#define CAN_ID_EXT(x)                            (((uint32_t)(((uint32_t)(x)) << CAN_ID_EXT_SHIFT)) & CAN_ID_EXT_MASK)
#define CAN_ID_STD_MASK                          (0x1FFC0000U)
#define CAN_ID_STD_SHIFT                         (18U)
#define CAN_ID_STD(x)                            (((uint32_t)(((uint32_t)(x)) << CAN_ID_STD_SHIFT)) & CAN_ID_STD_MASK)
#define CAN_ID_PRIO_MASK                         (0xE0000000U)
#define CAN_ID_PRIO_SHIFT                        (29U)
#define CAN_ID_PRIO(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_ID_PRIO_SHIFT)) & CAN_ID_PRIO_MASK)
/**
  * @}
  */

/* The count of CAN_ID */
#define CAN_ID_COUNT                             (32U)

/** @name WORD0 - Message Buffer 0 WORD0 Register..Message Buffer 31 WORD0 Register
  * @{
  */
#define CAN_WORD0_DATA_BYTE_3_MASK               (0xFFU)
#define CAN_WORD0_DATA_BYTE_3_SHIFT              (0U)
#define CAN_WORD0_DATA_BYTE_3(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_WORD0_DATA_BYTE_3_SHIFT)) & CAN_WORD0_DATA_BYTE_3_MASK)
#define CAN_WORD0_DATA_BYTE_2_MASK               (0xFF00U)
#define CAN_WORD0_DATA_BYTE_2_SHIFT              (8U)
#define CAN_WORD0_DATA_BYTE_2(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_WORD0_DATA_BYTE_2_SHIFT)) & CAN_WORD0_DATA_BYTE_2_MASK)
#define CAN_WORD0_DATA_BYTE_1_MASK               (0xFF0000U)
#define CAN_WORD0_DATA_BYTE_1_SHIFT              (16U)
#define CAN_WORD0_DATA_BYTE_1(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_WORD0_DATA_BYTE_1_SHIFT)) & CAN_WORD0_DATA_BYTE_1_MASK)
#define CAN_WORD0_DATA_BYTE_0_MASK               (0xFF000000U)
#define CAN_WORD0_DATA_BYTE_0_SHIFT              (24U)
#define CAN_WORD0_DATA_BYTE_0(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_WORD0_DATA_BYTE_0_SHIFT)) & CAN_WORD0_DATA_BYTE_0_MASK)
/**
  * @}
  */

/* The count of CAN_WORD0 */
#define CAN_WORD0_COUNT                          (32U)

/** @name WORD1 - Message Buffer 0 WORD1 Register..Message Buffer 31 WORD1 Register
  * @{
  */
#define CAN_WORD1_DATA_BYTE_7_MASK               (0xFFU)
#define CAN_WORD1_DATA_BYTE_7_SHIFT              (0U)
#define CAN_WORD1_DATA_BYTE_7(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_WORD1_DATA_BYTE_7_SHIFT)) & CAN_WORD1_DATA_BYTE_7_MASK)
#define CAN_WORD1_DATA_BYTE_6_MASK               (0xFF00U)
#define CAN_WORD1_DATA_BYTE_6_SHIFT              (8U)
#define CAN_WORD1_DATA_BYTE_6(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_WORD1_DATA_BYTE_6_SHIFT)) & CAN_WORD1_DATA_BYTE_6_MASK)
#define CAN_WORD1_DATA_BYTE_5_MASK               (0xFF0000U)
#define CAN_WORD1_DATA_BYTE_5_SHIFT              (16U)
#define CAN_WORD1_DATA_BYTE_5(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_WORD1_DATA_BYTE_5_SHIFT)) & CAN_WORD1_DATA_BYTE_5_MASK)
#define CAN_WORD1_DATA_BYTE_4_MASK               (0xFF000000U)
#define CAN_WORD1_DATA_BYTE_4_SHIFT              (24U)
#define CAN_WORD1_DATA_BYTE_4(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_WORD1_DATA_BYTE_4_SHIFT)) & CAN_WORD1_DATA_BYTE_4_MASK)
/**
  * @}
  */

/* The count of CAN_WORD1 */
#define CAN_WORD1_COUNT                          (32U)

/** @name RXIMR - Rx Individual Mask Registers
  * @{
  */
#define CAN_RXIMR_MI_MASK                        (0xFFFFFFFFU)
#define CAN_RXIMR_MI_SHIFT                       (0U)
/* MI - Individual Mask Bits */
/* 0b00000000000000000000000000000000..The corresponding bit in the filter is "don't care." */
/* 0b00000000000000000000000000000001..The corresponding bit in the filter is checked. */

#define CAN_RXIMR_MI(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_RXIMR_MI_SHIFT)) & CAN_RXIMR_MI_MASK)
/**
  * @}
  */

/* The count of CAN_RXIMR */
#define CAN_RXIMR_COUNT                          (32U)





/** @name FDCTRL - CAN FD Control Register
  * @{
  */
#define CAN_FDCTRL_TDCVAL_MASK                   (0x3FU)
#define CAN_FDCTRL_TDCVAL_SHIFT                  (0U)
#define CAN_FDCTRL_TDCVAL(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCVAL_SHIFT)) & CAN_FDCTRL_TDCVAL_MASK)

#define CAN_FDCTRL_TDCOFF_MASK                   (0x1F00U)
#define CAN_FDCTRL_TDCOFF_SHIFT                  (8U)
#define MAX_TDCOFF                               ((uint32_t)CAN_FDCTRL_TDCOFF_MASK >> CAN_FDCTRL_TDCOFF_SHIFT)
#define CAN_FDCTRL_TDCOFF(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCOFF_SHIFT)) & CAN_FDCTRL_TDCOFF_MASK)

#define CAN_FDCTRL_TDCFALL_MASK                  (0x4000U)
#define CAN_FDCTRL_TDCFALL_SHIFT                 (14U)
#define CAN_FDCTRL_TDCFALL(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCFALL_SHIFT)) & CAN_FDCTRL_TDCFALL_MASK)

#define CAN_FDCTRL_TDCEN_MASK                    (0x8000U)
#define CAN_FDCTRL_TDCEN_SHIFT                   (15U)
#define CAN_FDCTRL_TDCEN(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCEN_SHIFT)) & CAN_FDCTRL_TDCEN_MASK)

#define CAN_FDCTRL_MBDSR0_MASK                   (0x30000U)
#define CAN_FDCTRL_MBDSR0_SHIFT                  (16U)
#define CAN_FDCTRL_MBDSR0(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_MBDSR0_SHIFT)) & CAN_FDCTRL_MBDSR0_MASK)

#define CAN_FDCTRL_FDRATE_MASK                   (0x80000000U)
#define CAN_FDCTRL_FDRATE_SHIFT                  (31U)
#define CAN_FDCTRL_FDRATE(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_FDRATE_SHIFT)) & CAN_FDCTRL_FDRATE_MASK)

/**
  * @}
  */

/** @name FDCBT -  CAN FD Bit Timing Register
  * @{
  */
#define CAN_FDCBT_FPSEG2_MASK                    (0x7U)
#define CAN_FDCBT_FPSEG2_SHIFT                   (0U)
#define CAN_FDCBT_FPSEG2(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPSEG2_SHIFT)) & CAN_FDCBT_FPSEG2_MASK)

#define CAN_FDCBT_FPSEG1_MASK                    (0xE0U)
#define CAN_FDCBT_FPSEG1_SHIFT                   (5U)
#define CAN_FDCBT_FPSEG1(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPSEG1_SHIFT)) & CAN_FDCBT_FPSEG1_MASK)

#define CAN_FDCBT_FPROPSEG_MASK                  (0x7C00U) 
#define CAN_FDCBT_FPROPSEG_SHIFT                 (10U)
#define CAN_FDCBT_FPROPSEG(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPROPSEG_SHIFT)) & CAN_FDCBT_FPROPSEG_MASK)

#define CAN_FDCBT_FRJW_MASK                      (0x70000U) 
#define CAN_FDCBT_FRJW_SHIFT                     (16U)
#define CAN_FDCBT_FRJW(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FRJW_SHIFT)) & CAN_FDCBT_FRJW_MASK)

#define CAN_FDCBT_FPRESDIV_MASK                  (0x3FF00000U)
#define CAN_FDCBT_FPRESDIV_SHIFT                 (20U)
#define CAN_FDCBT_FPRESDIV(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPRESDIV_SHIFT)) & CAN_FDCBT_FPRESDIV_MASK)

/**
  * @}
  */

/** @name FDCRC -   CAN FD CRC Register
  * @{
  */
#define CAN_FDCRC_FDTXCRC_MASK                   (0x1FFFFFU)
#define CAN_FDCRC_FDTXCRC_SHIFT                  (0U)
#define CAN_FDCRC_FDTXCRC(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCRC_FDTXCRC_SHIFT)) & CAN_FDCRC_FDTXCRC_MASK)

#define CAN_FDCRC_FDMBCRC_MASK                   (0x7FU)
#define CAN_FDCRC_FDMBCRC_SHIFT                  (24U)
#define CAN_FDCRC_FDMBCRC(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCRC_FDMBCRC_SHIFT)) & CAN_FDCRC_FDMBCRC_MASK)

/**
  * @}
  */

/** @name ERFCR -     Enhanced Rx FIFO Control Register
  * @{
  */
#define CAN_ERFCR_ERFWM_MASK                     (0x7U)
#define CAN_ERFCR_ERFWM_SHIFT                    (0U)
#define CAN_ERFCR_ERFWM(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ERFCR_ERFWM_SHIFT)) & CAN_ERFCR_ERFWM_MASK)

#define CAN_ERFCR_NEXIF_MASK                     (0x10000U)
#define CAN_ERFCR_NEXIF_SHIFT                    (16U)
#define CAN_ERFCR_NEXIF(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ERFCR_NEXIF_SHIFT)) & CAN_ERFCR_NEXIF_MASK)

#define CAN_ERFCR_DMALW_MASK                     (0x7C000000U)   
#define CAN_ERFCR_DMALW_SHIFT                    (26U)
#define CAN_ERFCR_DMALW(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ERFCR_DMALW_SHIFT)) & CAN_ERFCR_DMALW_MASK)

#define CAN_ERFCR_ERFEN_MASK                     (0x80000000U)   
#define CAN_ERFCR_ERFEN_SHIFT                    (31U)
#define CAN_ERFCR_ERFEN(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ERFCR_ERFEN_SHIFT)) & CAN_ERFCR_ERFEN_MASK)

/**
  * @}
  */

/** @name ERFIER -    Enhanced Rx FIFO Interrupt Enable Register
  * @{
  */
#define CAN_ERFIER_ERFDAIE_MASK                  (0x10000000U)
#define CAN_ERFIER_ERFDAIE_SHIFT                 (28U)
#define CAN_ERFIER_ERFDAIE(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_ERFIER_ERFDAIE_SHIFT)) & CAN_ERFIER_ERFDAIE_MASK)

#define CAN_ERFIER_ERFWMIIE_MASK                 (0x20000000U)
#define CAN_ERFIER_ERFWMIIE_SHIFT                (29U)
#define CAN_ERFIER_ERFWMIIE(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_ERFIER_ERFWMIIE_SHIFT)) & CAN_ERFIER_ERFWMIIE_MASK)

#define CAN_ERFIER_ERFOVFIE_MASK                 (0x40000000U)
#define CAN_ERFIER_ERFOVFIE_SHIFT                (30U)
#define CAN_ERFIER_ERFOVFIE(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_ERFIER_ERFOVFIE_SHIFT)) & CAN_ERFIER_ERFOVFIE_MASK)

#define CAN_ERFIER_ERFUFWIE_MASK                 (0x80000000U)
#define CAN_ERFIER_ERFUFWIE_SHIFT                (31U)
#define CAN_ERFIER_ERFUFWIE(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_ERFIER_ERFUFWIE_SHIFT)) & CAN_ERFIER_ERFUFWIE_MASK)

/**
  * @}
  */

/** @name ERFSR -    Enhanced Rx FIFO Status Register
  * @{
  */
#define CAN_ERFSR_ERFEL_MASK                     (0x7U)
#define CAN_ERFSR_ERFEL_SHIFT                    (0U)
#define CAN_ERFSR_ERFEL(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ERFSR_ERFEL_SHIFT)) & CAN_ERFSR_ERFEL_MASK)

#define CAN_ERFSR_ERFF_MASK                      (0x10000U)
#define CAN_ERFSR_ERFF_SHIFT                     (16U)
#define CAN_ERFSR_ERFF(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ERFSR_ERFF_SHIFT)) & CAN_ERFSR_ERFF_MASK)
#define CAN_ERFSR_ERFE_MASK                      (0x20000U)
#define CAN_ERFSR_ERFE_SHIFT                     (17U)
#define CAN_ERFSR_ERFE(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ERFSR_ERFE_SHIFT)) & CAN_ERFSR_ERFE_MASK)

#define CAN_ERFSR_ERFCLR_MASK                    (0x8000000U)
#define CAN_ERFSR_ERFCLR_SHIFT                   (27U)
#define CAN_ERFSR_ERFCLR(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ERFSR_ERFCLR_SHIFT)) & CAN_ERFSR_ERFCLR_MASK)

#define CAN_ERFSR_ERFDA_MASK                     (0x10000000U)
#define CAN_ERFSR_ERFDA_SHIFT                    (28U)
#define CAN_ERFSR_ERFDA(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ERFSR_ERFDA_SHIFT)) & CAN_ERFSR_ERFDA_MASK)
#define CAN_ERFSR_ERFWMI_MASK                    (0x20000000U)
#define CAN_ERFSR_ERFWMI_SHIFT                   (29U)
#define CAN_ERFSR_ERFWMI(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ERFSR_ERFWMI_SHIFT)) & CAN_ERFSR_ERFWMI_MASK)
#define CAN_ERFSR_ERFOVF_MASK                    (0x40000000U)
#define CAN_ERFSR_ERFOVF_SHIFT                   (30U)
#define CAN_ERFSR_ERFOVF(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ERFSR_ERFOVF_SHIFT)) & CAN_ERFSR_ERFOVF_MASK)
#define CAN_ERFSR_ERFUFW_MASK                    (0x80000000U)
#define CAN_ERFSR_ERFUFW_SHIFT                   (31U)
#define CAN_ERFSR_ERFUFW(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ERFSR_ERFUFW_SHIFT)) & CAN_ERFSR_ERFUFW_MASK)

/**
  * @}
  */

/** @name ERFCS -  Enhanced Message Buffer 0 CS Register..Enhanced Message Buffer 5 CS Register
  * @{
  */
#define CAN_ERFCS_TIME_STAMP_MASK                (0xFFFFU)
#define CAN_ERFCS_TIME_STAMP_SHIFT               (0U)
#define CAN_ERFCS_TIME_STAMP(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ERFCS_TIME_STAMP_SHIFT)) & CAN_ERFCS_TIME_STAMP_MASK)
#define CAN_ERFCS_DLC_MASK                       (0xF0000U)
#define CAN_ERFCS_DLC_SHIFT                      (16U)
#define CAN_ERFCS_DLC(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ERFCS_DLC_SHIFT)) & CAN_ERFCS_DLC_MASK)
#define CAN_ERFCS_RTR_MASK                       (0x100000U)
#define CAN_ERFCS_RTR_SHIFT                      (20U)
#define CAN_ERFCS_RTR(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ERFCS_RTRSHIFT)) & CAN_ERFCS_RTR_MASK)
#define CAN_ERFCS_IDE_MASK                       (0x200000U)
#define CAN_ERFCS_IDE_SHIFT                      (21U)
#define CAN_ERFCS_IDE(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ERFCS_IDE_SHIFT)) & CAN_ERFCS_IDE_MASK)
#define CAN_ERFCS_SRR_MASK                       (0x400000U)
#define CAN_ERFCS_SRR_SHIFT                      (22U)
#define CAN_ERFCS_SRR(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ERFCS_SRR_SHIFT)) & CAN_ERFCS_SRR_MASK)

#define CAN_ERFCS_ESI_MASK                       (0x20000000U)
#define CAN_ERFCS_ESI_SHIFT                      (29U)
#define CAN_ERFCS_ESI(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ERFCS_ESI_SHIFT)) & CAN_ERFCS_ESI_MASK)
#define CAN_ERFCS_BRS_MASK                       (0x40000000U)
#define CAN_ERFCS_BRS_SHIFT                      (30U)
#define CAN_ERFCS_BRS(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ERFCS_BRS_SHIFT)) & CAN_ERFCS_BRS_MASK)
#define CAN_ERFCS_EDL_MASK                       (0x80000000U)
#define CAN_ERFCS_EDL_SHIFT                      (31U)
#define CAN_ERFCS_EDL(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ERFCS_EDL_SHIFT)) & CAN_ERFCS_EDL_MASK)

/**
  * @}
  */

/* The count of CAN_ERFCS */
#define CAN_ERFCS_COUNT                          (6U)

/** @name ERFDID - Enhanced Message Buffer 0 ID Register..Enhanced Message Buffer 5 ID Register
  * @{
  */
#define CAN_ERFDID_EXT_MASK                      (0x3FFFFU)
#define CAN_ERFDID_EXT_SHIFT                     (0U)
#define CAN_ERFDID_EXT(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ERFDID_EXT_SHIFT)) & CAN_ERFDID_EXT_MASK)
#define CAN_ERFDID_STD_MASK                      (0x1FFC0000U)
#define CAN_ERFDID_STD_SHIFT                     (18U)
#define CAN_ERFDID_STD(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ERFDID_STD_SHIFT)) & CAN_ERFDID_STD_MASK)

/**
  * @}
  */

/* The count of CAN_ERFDID */
#define CAN_ERFDID_COUNT                             (6U)

/** @name ERFDWORD0 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD0_DATA_BYTE_3_MASK               (0xFFU)
#define CAN_ERFDWORD0_DATA_BYTE_3_SHIFT              (0U)
#define CAN_ERFDWORD0_DATA_BYTE_3(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD0_DATA_BYTE_3_SHIFT)) & CAN_ERFDWORD0_DATA_BYTE_3_MASK)
#define CAN_ERFDWORD0_DATA_BYTE_2_MASK               (0xFF00U)
#define CAN_ERFDWORD0_DATA_BYTE_2_SHIFT              (8U)
#define CAN_ERFDWORD0_DATA_BYTE_2(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD0_DATA_BYTE_2_SHIFT)) & CAN_WORD0_DATA_BYTE_2_MASK)
#define CAN_ERFDWORD0_DATA_BYTE_1_MASK               (0xFF0000U)
#define CAN_ERFDWORD0_DATA_BYTE_1_SHIFT              (16U)
#define CAN_ERFDWORD0_DATA_BYTE_1(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD0_DATA_BYTE_1_SHIFT)) & CAN_ERFDWORD0_DATA_BYTE_1_MASK)
#define CAN_ERFDWORD0_DATA_BYTE_0_MASK               (0xFF000000U)
#define CAN_ERFDWORD0_DATA_BYTE_0_SHIFT              (24U)
#define CAN_ERFDWORD0_DATA_BYTE_0(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD0_DATA_BYTE_0_SHIFT)) & CAN_ERFDWORD0_DATA_BYTE_0_MASK)

/**
  * @}
  */

/** @name ERFDWORD1 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD1_DATA_BYTE_7_MASK               (0xFFU)
#define CAN_ERFDWORD1_DATA_BYTE_7_SHIFT              (0U)
#define CAN_ERFDWORD1_DATA_BYTE_7(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD1_DATA_BYTE_7_SHIFT)) & CAN_ERFDWORD1_DATA_BYTE_7_MASK)
#define CAN_ERFDWORD1_DATA_BYTE_6_MASK               (0xFF00U)
#define CAN_ERFDWORD1_DATA_BYTE_6_SHIFT              (8U)
#define CAN_ERFDWORD1_DATA_BYTE_6(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD1_DATA_BYTE_6_SHIFT)) & CAN_ERFDWORD1_DATA_BYTE_6_MASK)
#define CAN_ERFDWORD1_DATA_BYTE_5_MASK               (0xFF0000U)
#define CAN_ERFDWORD1_DATA_BYTE_5_SHIFT              (16U)
#define CAN_ERFDWORD1_DATA_BYTE_5(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD1_DATA_BYTE_5_SHIFT)) & CAN_ERFDWORD1_DATA_BYTE_5_MASK)
#define CAN_ERFDWORD1_DATA_BYTE_4_MASK               (0xFF000000U)
#define CAN_ERFDWORD1_DATA_BYTE_4_SHIFT              (24U)
#define CAN_ERFDWORD1_DATA_BYTE_4(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD1_DATA_BYTE_4_SHIFT)) & CAN_ERFDWORD1_DATA_BYTE_4_MASK)

/**
  * @}
  */

/** @name ERFDWORD2 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD2_DATA_BYTE_11_MASK               (0xFFU)
#define CAN_ERFDWORD2_DATA_BYTE_11_SHIFT              (0U)
#define CAN_ERFDWORD2_DATA_BYTE_11(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD2_DATA_BYTE_11_SHIFT)) & CAN_ERFDWORD2_DATA_BYTE_11_MASK)
#define CAN_ERFDWORD2_DATA_BYTE_10_MASK               (0xFF00U)                                                                 
#define CAN_ERFDWORD2_DATA_BYTE_10_SHIFT              (8U)                                                                      
#define CAN_ERFDWORD2_DATA_BYTE_10(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD2_DATA_BYTE_10_SHIFT)) & CAN_ERFDWORD2_DATA_BYTE_10_MASK)
#define CAN_ERFDWORD2_DATA_BYTE_9_MASK               (0xFF0000U)                                                           
#define CAN_ERFDWORD2_DATA_BYTE_9_SHIFT              (16U)                                                                 
#define CAN_ERFDWORD2_DATA_BYTE_9(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD2_DATA_BYTE_9_SHIFT)) & CAN_ERFDWORD2_DATA_BYTE_9_MASK)
#define CAN_ERFDWORD2_DATA_BYTE_8_MASK               (0xFF000000U)                              
#define CAN_ERFDWORD2_DATA_BYTE_8_SHIFT              (24U)                                      
#define CAN_ERFDWORD2_DATA_BYTE_8(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD2_DATA_BYTE_8_SHIFT)) & CAN_ERFDWORD2_DATA_BYTE_8_MASK)

/**
  * @}
  */

/** @name ERFDWORD3 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD3_DATA_BYTE_15_MASK               (0xFFU)
#define CAN_ERFDWORD3_DATA_BYTE_15_SHIFT              (0U)
#define CAN_ERFDWORD3_DATA_BYTE_15(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD3_DATA_BYTE_15_SHIFT)) & CAN_ERFDWORD3_DATA_BYTE_15_MASK)
#define CAN_ERFDWORD3_DATA_BYTE_14_MASK               (0xFF00U)                                                                 
#define CAN_ERFDWORD3_DATA_BYTE_14_SHIFT              (8U)                                                                      
#define CAN_ERFDWORD3_DATA_BYTE_14(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD3_DATA_BYTE_14_SHIFT)) & CAN_ERFDWORD3_DATA_BYTE_14_MASK)
#define CAN_ERFDWORD3_DATA_BYTE_13_MASK               (0xFF0000U)                                                           
#define CAN_ERFDWORD3_DATA_BYTE_13_SHIFT              (16U)                                                                 
#define CAN_ERFDWORD3_DATA_BYTE_13(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD3_DATA_BYTE_13_SHIFT)) & CAN_ERFDWORD3_DATA_BYTE_13_MASK)
#define CAN_ERFDWORD3_DATA_BYTE_12_MASK               (0xFF000000U)                              
#define CAN_ERFDWORD3_DATA_BYTE_12_SHIFT              (24U)                                      
#define CAN_ERFDWORD3_DATA_BYTE_12(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD3_DATA_BYTE_12_SHIFT)) & CAN_ERFDWORD3_DATA_BYTE_12_MASK)

/**
  * @}
  */

/** @name ERFDWORD4 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD4_DATA_BYTE_19_MASK               (0xFFU)
#define CAN_ERFDWORD4_DATA_BYTE_19_SHIFT              (0U)
#define CAN_ERFDWORD4_DATA_BYTE_19(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD4_DATA_BYTE_19_SHIFT)) & CAN_ERFDWORD4_DATA_BYTE_19_MASK)
#define CAN_ERFDWORD4_DATA_BYTE_18_MASK               (0xFF00U)                                  
#define CAN_ERFDWORD4_DATA_BYTE_18_SHIFT              (8U)                                       
#define CAN_ERFDWORD4_DATA_BYTE_18(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD4_DATA_BYTE_18_SHIFT)) & CAN_ERFDWORD4_DATA_BYTE_18_MASK)
#define CAN_ERFDWORD4_DATA_BYTE_17_MASK               (0xFF0000U)                               
#define CAN_ERFDWORD4_DATA_BYTE_17_SHIFT              (16U)                                     
#define CAN_ERFDWORD4_DATA_BYTE_17(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD4_DATA_BYTE_17_SHIFT)) & CAN_ERFDWORD4_DATA_BYTE_17_MASK)
#define CAN_ERFDWORD4_DATA_BYTE_16_MASK               (0xFF000000U)                            
#define CAN_ERFDWORD4_DATA_BYTE_16_SHIFT              (24U)                                    
#define CAN_ERFDWORD4_DATA_BYTE_16(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD4_DATA_BYTE_16_SHIFT)) & CAN_ERFDWORD4_DATA_BYTE_16_MASK)

/**
  * @}
  */

/** @name ERFDWORD5 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD5_DATA_BYTE_23_MASK               (0xFFU)
#define CAN_ERFDWORD5_DATA_BYTE_23_SHIFT              (0U)
#define CAN_ERFDWORD5_DATA_BYTE_23(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD5_DATA_BYTE_23_SHIFT)) & CAN_ERFDWORD5_DATA_BYTE_23_MASK)
#define CAN_ERFDWORD5_DATA_BYTE_22_MASK               (0xFF00U)                                 
#define CAN_ERFDWORD5_DATA_BYTE_22_SHIFT              (8U)                                      
#define CAN_ERFDWORD5_DATA_BYTE_22(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD5_DATA_BYTE_22_SHIFT)) & CAN_ERFDWORD5_DATA_BYTE_22_MASK)
#define CAN_ERFDWORD5_DATA_BYTE_21_MASK               (0xFF0000U)                                
#define CAN_ERFDWORD5_DATA_BYTE_21_SHIFT              (16U)                                      
#define CAN_ERFDWORD5_DATA_BYTE_21(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD5_DATA_BYTE_21_SHIFT)) & CAN_ERFDWORD5_DATA_BYTE_21_MASK)
#define CAN_ERFDWORD5_DATA_BYTE_20_MASK               (0xFF000000U)                             
#define CAN_ERFDWORD5_DATA_BYTE_20_SHIFT              (24U)                                     
#define CAN_ERFDWORD5_DATA_BYTE_20(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD5_DATA_BYTE_20_SHIFT)) & CAN_ERFDWORD5_DATA_BYTE_20_MASK)

/**
  * @}
  */

/** @name ERFDWORD6 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD6_DATA_BYTE_27_MASK               (0xFFU)
#define CAN_ERFDWORD6_DATA_BYTE_27_SHIFT              (0U)
#define CAN_ERFDWORD6_DATA_BYTE_27(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD6_DATA_BYTE_27_SHIFT)) & CAN_ERFDWORD6_DATA_BYTE_27_MASK)
#define CAN_ERFDWORD6_DATA_BYTE_26_MASK               (0xFF00U)                                  
#define CAN_ERFDWORD6_DATA_BYTE_26_SHIFT              (8U)                                       
#define CAN_ERFDWORD6_DATA_BYTE_26(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD6_DATA_BYTE_26_SHIFT)) & CAN_ERFDWORD6_DATA_BYTE_26_MASK)
#define CAN_ERFDWORD6_DATA_BYTE_25_MASK               (0xFF0000U)                               
#define CAN_ERFDWORD6_DATA_BYTE_25_SHIFT              (16U)                                     
#define CAN_ERFDWORD6_DATA_BYTE_25(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD6_DATA_BYTE_25_SHIFT)) & CAN_ERFDWORD6_DATA_BYTE_25_MASK)
#define CAN_ERFDWORD6_DATA_BYTE_24_MASK               (0xFF000000U)                            
#define CAN_ERFDWORD6_DATA_BYTE_24_SHIFT              (24U)                                    
#define CAN_ERFDWORD6_DATA_BYTE_24(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD6_DATA_BYTE_24_SHIFT)) & CAN_ERFDWORD6_DATA_BYTE_24_MASK)

/**
  * @}
  */

/** @name ERFDWORD7 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD7_DATA_BYTE_31_MASK               (0xFFU)
#define CAN_ERFDWORD7_DATA_BYTE_31_SHIFT              (0U)
#define CAN_ERFDWORD7_DATA_BYTE_31(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD7_DATA_BYTE_31_SHIFT)) & CAN_ERFDWORD7_DATA_BYTE_31_MASK)
#define CAN_ERFDWORD7_DATA_BYTE_30_MASK               (0xFF00U)                                   
#define CAN_ERFDWORD7_DATA_BYTE_30_SHIFT              (8U)                                        
#define CAN_ERFDWORD7_DATA_BYTE_30(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD7_DATA_BYTE_30_SHIFT)) & CAN_ERFDWORD7_DATA_BYTE_30_MASK)
#define CAN_ERFDWORD7_DATA_BYTE_29_MASK               (0xFF0000U)                               
#define CAN_ERFDWORD7_DATA_BYTE_29_SHIFT              (16U)                                     
#define CAN_ERFDWORD7_DATA_BYTE_29(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD7_DATA_BYTE_29_SHIFT)) & CAN_ERFDWORD7_DATA_BYTE_29_MASK)
#define CAN_ERFDWORD7_DATA_BYTE_28_MASK               (0xFF000000U)                               
#define CAN_ERFDWORD7_DATA_BYTE_28_SHIFT              (24U)                                       
#define CAN_ERFDWORD7_DATA_BYTE_28(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD7_DATA_BYTE_28_SHIFT)) & CAN_ERFDWORD7_DATA_BYTE_28_MASK)

/**
  * @}
  */

/** @name ERFDWORD8 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD8_DATA_BYTE_35_MASK               (0xFFU)
#define CAN_ERFDWORD8_DATA_BYTE_35_SHIFT              (0U)
#define CAN_ERFDWORD8_DATA_BYTE_35(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD9_DATA_BYTE_35_SHIFT)) & CAN_ERFDWORD9_DATA_BYTE_35_MASK)
#define CAN_ERFDWORD8_DATA_BYTE_34_MASK               (0xFF00U)                                  
#define CAN_ERFDWORD8_DATA_BYTE_34_SHIFT              (8U)                                       
#define CAN_ERFDWORD8_DATA_BYTE_34(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD9_DATA_BYTE_34_SHIFT)) & CAN_ERFDWORD9_DATA_BYTE_34_MASK)
#define CAN_ERFDWORD8_DATA_BYTE_33_MASK               (0xFF0000U)                                
#define CAN_ERFDWORD8_DATA_BYTE_33_SHIFT              (16U)                                      
#define CAN_ERFDWORD8_DATA_BYTE_33(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD9_DATA_BYTE_33_SHIFT)) & CAN_ERFDWORD9_DATA_BYTE_33_MASK)
#define CAN_ERFDWORD8_DATA_BYTE_32_MASK               (0xFF000000U)                              
#define CAN_ERFDWORD8_DATA_BYTE_32_SHIFT              (24U)                                      
#define CAN_ERFDWORD8_DATA_BYTE_32(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD9_DATA_BYTE_32_SHIFT)) & CAN_ERFDWORD9_DATA_BYTE_32_MASK)

/**
  * @}
  */

/** @name ERFDWORD9 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD9_DATA_BYTE_39_MASK               (0xFFU)
#define CAN_ERFDWORD9_DATA_BYTE_39_SHIFT              (0U)
#define CAN_ERFDWORD9_DATA_BYTE_39(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD9_DATA_BYTE_39_SHIFT)) & CAN_ERFDWORD9_DATA_BYTE_39_MASK)
#define CAN_ERFDWORD9_DATA_BYTE_38_MASK               (0xFF00U)                            
#define CAN_ERFDWORD9_DATA_BYTE_38_SHIFT              (8U)                                 
#define CAN_ERFDWORD9_DATA_BYTE_38(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD9_DATA_BYTE_38_SHIFT)) & CAN_ERFDWORD9_DATA_BYTE_38_MASK)
#define CAN_ERFDWORD9_DATA_BYTE_37_MASK               (0xFF0000U)                               
#define CAN_ERFDWORD9_DATA_BYTE_37_SHIFT              (16U)                                     
#define CAN_ERFDWORD9_DATA_BYTE_37(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD9_DATA_BYTE_37_SHIFT)) & CAN_ERFDWORD9_DATA_BYTE_37_MASK)
#define CAN_ERFDWORD9_DATA_BYTE_36_MASK               (0xFF000000U)                             
#define CAN_ERFDWORD9_DATA_BYTE_36_SHIFT              (24U)                                     
#define CAN_ERFDWORD9_DATA_BYTE_36(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD9_DATA_BYTE_36_SHIFT)) & CAN_ERFDWORD9_DATA_BYTE_36_MASK)

/**
  * @}
  */

/** @name ERFDWORD10 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD10_DATA_BYTE_43_MASK               (0xFFU)
#define CAN_ERFDWORD10_DATA_BYTE_43_SHIFT              (0U)
#define CAN_ERFDWORD10_DATA_BYTE_43(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD10_DATA_BYTE_43_SHIFT)) & CAN_ERFDWORD10_DATA_BYTE_43_MASK)
#define CAN_ERFDWORD10_DATA_BYTE_42_MASK               (0xFF00U)                                 
#define CAN_ERFDWORD10_DATA_BYTE_42_SHIFT              (8U)                                      
#define CAN_ERFDWORD10_DATA_BYTE_42(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD10_DATA_BYTE_42_SHIFT)) & CAN_ERFDWORD10_DATA_BYTE_42_MASK)
#define CAN_ERFDWORD10_DATA_BYTE_41_MASK               (0xFF0000U)                                 
#define CAN_ERFDWORD10_DATA_BYTE_41_SHIFT              (16U)                                       
#define CAN_ERFDWORD10_DATA_BYTE_41(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD10_DATA_BYTE_41_SHIFT)) & CAN_ERFDWORD10_DATA_BYTE_41_MASK)
#define CAN_ERFDWORD10_DATA_BYTE_40_MASK               (0xFF000000U)                             
#define CAN_ERFDWORD10_DATA_BYTE_40_SHIFT              (24U)                                     
#define CAN_ERFDWORD10_DATA_BYTE_40(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD10_DATA_BYTE_40_SHIFT)) & CAN_ERFDWORD10_DATA_BYTE_40_MASK)

/**
  * @}
  */

/** @name ERFDWORD11 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD11_DATA_BYTE_47_MASK               (0xFFU)
#define CAN_ERFDWORD11_DATA_BYTE_47_SHIFT              (0U)
#define CAN_ERFDWORD11_DATA_BYTE_47(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD11_DATA_BYTE_47_SHIFT)) & CAN_ERFDWORD11_DATA_BYTE_47_MASK)
#define CAN_ERFDWORD11_DATA_BYTE_46_MASK               (0xFF00U)                                  
#define CAN_ERFDWORD11_DATA_BYTE_46_SHIFT              (8U)                                       
#define CAN_ERFDWORD11_DATA_BYTE_46(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD11_DATA_BYTE_46_SHIFT)) & CAN_ERFDWORD11_DATA_BYTE_46_MASK)
#define CAN_ERFDWORD11_DATA_BYTE_45_MASK               (0xFF0000U)                               
#define CAN_ERFDWORD11_DATA_BYTE_45_SHIFT              (16U)                                     
#define CAN_ERFDWORD11_DATA_BYTE_45(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD11_DATA_BYTE_45_SHIFT)) & CAN_ERFDWORD11_DATA_BYTE_45_MASK)
#define CAN_ERFDWORD11_DATA_BYTE_44_MASK               (0xFF000000U)                           
#define CAN_ERFDWORD11_DATA_BYTE_44_SHIFT              (24U)                                   
#define CAN_ERFDWORD11_DATA_BYTE_44(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD11_DATA_BYTE_44_SHIFT)) & CAN_ERFDWORD11_DATA_BYTE_44_MASK)

/**
  * @}
  */

/** @name ERFDWORD12 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD12_DATA_BYTE_51_MASK               (0xFFU)
#define CAN_ERFDWORD12_DATA_BYTE_51_SHIFT              (0U)
#define CAN_ERFDWORD12_DATA_BYTE_51(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD12_DATA_BYTE_51_SHIFT)) & CAN_ERFDWORD12_DATA_BYTE_51_MASK)
#define CAN_ERFDWORD12_DATA_BYTE_50_MASK               (0xFF00U)                                   
#define CAN_ERFDWORD12_DATA_BYTE_50_SHIFT              (8U)                                        
#define CAN_ERFDWORD12_DATA_BYTE_50(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD12_DATA_BYTE_50_SHIFT)) & CAN_ERFDWORD12_DATA_BYTE_50_MASK)
#define CAN_ERFDWORD12_DATA_BYTE_49_MASK               (0xFF0000U)                                  
#define CAN_ERFDWORD12_DATA_BYTE_49_SHIFT              (16U)                                        
#define CAN_ERFDWORD12_DATA_BYTE_49(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD12_DATA_BYTE_49_SHIFT)) & CAN_ERFDWORD12_DATA_BYTE_49_MASK)
#define CAN_ERFDWORD12_DATA_BYTE_48_MASK               (0xFF000000U)                                
#define CAN_ERFDWORD12_DATA_BYTE_48_SHIFT              (24U)                                        
#define CAN_ERFDWORD12_DATA_BYTE_48(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD12_DATA_BYTE_48_SHIFT)) & CAN_ERFDWORD12_DATA_BYTE_48_MASK)

/**
  * @}
  */

/** @name ERFDWORD13 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD13_DATA_BYTE_55_MASK               (0xFFU)
#define CAN_ERFDWORD13_DATA_BYTE_55_SHIFT              (0U)
#define CAN_ERFDWORD13_DATA_BYTE_55(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD13_DATA_BYTE_55_SHIFT)) & CAN_ERFDWORD13_DATA_BYTE_55_MASK)
#define CAN_ERFDWORD13_DATA_BYTE_54_MASK               (0xFF00U)                              
#define CAN_ERFDWORD13_DATA_BYTE_54_SHIFT              (8U)                                      
#define CAN_ERFDWORD13_DATA_BYTE_54(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD13_DATA_BYTE_54_SHIFT)) & CAN_ERFDWORD13_DATA_BYTE_54_MASK)
#define CAN_ERFDWORD13_DATA_BYTE_53_MASK               (0xFF0000U)                                 
#define CAN_ERFDWORD13_DATA_BYTE_53_SHIFT              (16U)                                       
#define CAN_ERFDWORD13_DATA_BYTE_53(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD13_DATA_BYTE_53_SHIFT)) & CAN_ERFDWORD13_DATA_BYTE_53_MASK)
#define CAN_ERFDWORD13_DATA_BYTE_52_MASK               (0xFF000000U)                            
#define CAN_ERFDWORD13_DATA_BYTE_52_SHIFT              (24U)                                    
#define CAN_ERFDWORD13_DATA_BYTE_52(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD13_DATA_BYTE_52_SHIFT)) & CAN_ERFDWORD13_DATA_BYTE_52_MASK)

/**
  * @}
  */

/** @name ERFDWORD14 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD14_DATA_BYTE_59_MASK               (0xFFU)
#define CAN_ERFDWORD14_DATA_BYTE_59_SHIFT              (0U)
#define CAN_ERFDWORD14_DATA_BYTE_59(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD14_DATA_BYTE_59_SHIFT)) & CAN_ERFDWORD14_DATA_BYTE_59_MASK)
#define CAN_ERFDWORD14_DATA_BYTE_58_MASK               (0xFF00U)                                 
#define CAN_ERFDWORD14_DATA_BYTE_58_SHIFT              (8U)                                      
#define CAN_ERFDWORD14_DATA_BYTE_58(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD14_DATA_BYTE_58_SHIFT)) & CAN_ERFDWORD14_DATA_BYTE_58_MASK)
#define CAN_ERFDWORD14_DATA_BYTE_57_MASK               (0xFF0000U)                                
#define CAN_ERFDWORD14_DATA_BYTE_57_SHIFT              (16U)                                      
#define CAN_ERFDWORD14_DATA_BYTE_57(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD14_DATA_BYTE_57_SHIFT)) & CAN_ERFDWORD14_DATA_BYTE_57_MASK)
#define CAN_ERFDWORD14_DATA_BYTE_56_MASK               (0xFF000000U)                             
#define CAN_ERFDWORD14_DATA_BYTE_56_SHIFT              (24U)                                     
#define CAN_ERFDWORD14_DATA_BYTE_56(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD14_DATA_BYTE_56_SHIFT)) & CAN_ERFDWORD14_DATA_BYTE_56_MASK)
                                                                                                   
/**
  * @}
  */


/** @name ERFDWORD15 - Enhanced Message Buffer 0 WORD0 Register..Enhanced Message Buffer 5 WORD0 Register
  * @{
  */
#define CAN_ERFDWORD15_DATA_BYTE_63_MASK               (0xFFU)
#define CAN_ERFDWORD15_DATA_BYTE_63_SHIFT              (0U)
#define CAN_ERFDWORD15_DATA_BYTE_63(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD15_DATA_BYTE_63_SHIFT)) & CAN_ERFDWORD15_DATA_BYTE_63_MASK)
#define CAN_ERFDWORD15_DATA_BYTE_62_MASK               (0xFF00U)                                 
#define CAN_ERFDWORD15_DATA_BYTE_62_SHIFT              (8U)                                      
#define CAN_ERFDWORD15_DATA_BYTE_62(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD15_DATA_BYTE_62_SHIFT)) & CAN_ERFDWORD15_DATA_BYTE_62_MASK)
#define CAN_ERFDWORD15_DATA_BYTE_61_MASK               (0xFF0000U)                                 
#define CAN_ERFDWORD15_DATA_BYTE_61_SHIFT              (16U)                                       
#define CAN_ERFDWORD15_DATA_BYTE_61(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD15_DATA_BYTE_61_SHIFT)) & CAN_ERFDWORD15_DATA_BYTE_61_MASK)
#define CAN_ERFDWORD15_DATA_BYTE_60_MASK               (0xFF000000U)                               
#define CAN_ERFDWORD15_DATA_BYTE_60_SHIFT              (24U)                                       
#define CAN_ERFDWORD15_DATA_BYTE_60(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ERFDWORD15_DATA_BYTE_60_SHIFT)) & CAN_ERFDWORD15_DATA_BYTE_60_MASK)

/**
  * @}
  */


/** @name IHOFF -  Identifier receive filter hit indication
  * @{
  */
#define CAN_IHOFF_IDHIT_MASK                     (0x7FU)
#define CAN_IHOFF_IDHIT_SHIFT                    (0U)
#define CAN_IHOFF_IDHIT(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_IHOFF_IDHIT_SHIFT)) & CAN_IHOFF_IDHIT_MASK)

/**
  * @}
  */

/* The count of CAN_IHOFF */
#define CAN_IHOFF_COUNT                          (6U)


/** @name ERFFEL0~1 -     Enhanced Rx FIFO Filter Element Registers 
  * @{
  */
#define CAN_ERFFEL_FEL_MASK                      (0xFFFFFFFFU)
#define CAN_ERFFEL_FEL_SHIFT                     (0U)
#define CAN_ERFFEL_FEL(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ERFFEL_FEL_SHIFT)) & CAN_ERFFEL_FEL_MASK)

/* The count of CAN_ERFFEL */
#define CAN_ERFFEL_COUNT                         (2U)

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/*----------------------------------------------------------------------------*/
#endif
/*----------------------------------------------------------------------------*/

