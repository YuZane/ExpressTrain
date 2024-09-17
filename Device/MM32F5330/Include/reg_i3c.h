/***********************************************************************************************************************
    @file     reg_i3c.h
    @author   VV TEAM
    @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE SERIES OF
              MM32 FIRMWARE LIBRARY.
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

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __REG_I3C_H
#define __REG_I3C_H

/* Files includes ------------------------------------------------------------*/
#include "core_starmc1.h"

#if 0
/* IP_DesignSpec_I3C_V0.2 */
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__CC_ARM)
#pragma anon_unions
#elif defined(__ICCARM__)
#pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic ignored "-Wc11-extensions"
#pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif

/**
  * @brief I3C Base Address Definition
  */
#define I3C1_BASE                       (APB1PERIPH_BASE + 0xA000) /*!< Base Address: 0x4000A000 */

/**
  * @brief I3C Register Structure Definition
  */
typedef struct
{
    __IO uint32_t RESERVED0x00;        /*!< Reserved Register                  offset: 0x00 */
    __IO uint32_t CONFIG;              /*!< Configuration register             offset: 0x04 */
    __IO uint32_t STATUS;              /*!< Status register                    offset: 0x08 */
    __IO uint32_t CTRL;                /*!< Control register                   offset: 0x0C */
    __IO uint32_t INTSET;              /*!< Interrupt enable register          offset: 0x10 */
    __IO uint32_t INTCLR;              /*!< Clear interrupt-enable register    offset: 0x14 */
    __IO uint32_t INTMASKED;           /*!< Interrupt masked status register   offset: 0x18 */
    __IO uint32_t ERWAR;               /*!< Error and warning register         offset: 0x1C */
    __IO uint32_t DMACTRL;             /*!< DMA control register               offset: 0x20 */
    __IO uint32_t RESERVED0x24[2];     /*!< Reserved Register                  offset: 0x24~0x28 */
    __IO uint32_t DATACTRL;            /*!< Data control register              offset: 0x2C */
    __IO uint32_t WDATAB;              /*!< Write bytes register               offset: 0x30 */
    __IO uint32_t WDATABE;             /*!< Write the final byte register      offset: 0x34 */
    __IO uint32_t WDATAH;              /*!< Write a half-word register         offset: 0x38 */
    __IO uint32_t WDATAHE;             /*!< Write the final half-word register offset: 0x3C */
    __IO uint32_t RDATAB;              /*!< Read bytes register                offset: 0x40 */
    __IO uint32_t RESERVED0x44;        /*!< Reserved Register                  offset: 0x44 */
    __IO uint32_t RDATAH;              /*!< Read half-word register            offset: 0x48 */
    __IO uint32_t RESERVED0x4C[2];     /*!< Reserved Register                  offset: 0x4C~0x50 */
    __IO uint32_t WDATAB1;             /*!< Only write bytes register          offset: 0x54 */
    __IO uint32_t RESERVED0x58;        /*!< Reserved Register                  offset: 0x58 */
    __IO uint32_t CAPABILITIES2;       /*!< Functional information register2   offset: 0x5C */
    __IO uint32_t CAPABILITIES;        /*!< Functional information register    offset: 0x60 */
    __IO uint32_t DYNADDR;             /*!< Dynamic address register           offset: 0x64 */
    __IO uint32_t MAXLIMITS;           /*!< Max. limit register                offset: 0x68 */
    __IO uint32_t PARTNO;              /*!< Product model number id register   offset: 0x6C */
    __IO uint32_t IDEXT;               /*!< Id component register              offset: 0x70 */
    __IO uint32_t VENDORID;            /*!< Supplier id register               offset: 0x74 */
    __IO uint32_t RESERVED0x78[98];    /*!< Reserved Register                  offset: 0x78~0x1FC */
    __IO uint32_t DIV;                 /*!< Frequency division Register        offset: 0x200 */
    __IO uint32_t SPKLEN;              /*!< Filter register                    offset: 0x204 */
} I3C_TypeDef;

/**
  * @brief I3C type pointer Definition
  */
#define I3C1                                ((I3C_TypeDef *)I3C1_BASE)

/**
  * @brief I3C_CONFIG Register Bit Definition
  */
#define I3C_CONFIG_SLVENA_Pos               (0)
#define I3C_CONFIG_SLVENA_Msk               (0x01U << I3C_CONFIG_SLVENA_Pos)   /*!< I3C enable */
#define I3C_CONFIG_NACK_Pos                 (1)
#define I3C_CONFIG_NACK_Msk                 (0x01U << I3C_CONFIG_NACK_Pos)     /*!< Slave does not respond to all requests except the CCC broadcast */
#define I3C_CONFIG_MATSS_Pos                (2)
#define I3C_CONFIG_MATSS_Msk                (0x01U << I3C_CONFIG_MATSS_Pos)    /*!< The START and STOP state bits are setted only if the STATUS.MATCHED is setted */
#define I3C_CONFIG_S0IGNORE_Pos             (3)
#define I3C_CONFIG_S0IGNORE_Msk             (0x01U << I3C_CONFIG_S0IGNORE_Pos) /*!< The s0 or s1 errors are not detected */

#define I3C_CONFIG_IDRAND_Pos               (8)
#define I3C_CONFIG_IDRAND_Msk               (0x01U << I3C_CONFIG_IDRAND_Pos)   /*!< The bit 32 of the 48-bit PID indicates that the value of the partno register is random */
#define I3C_CONFIG_OFFLINE_Pos              (9)
#define I3C_CONFIG_OFFLINE_Msk              (0x01U << I3C_CONFIG_OFFLINE_Pos)  /*!< Rejoin the i3c bus with the existing dynamic address */

#define I3C_CONFIG_BAMAT_Pos                (16)
#define I3C_CONFIG_BAMAT_Msk                (0xFFU << I3C_CONFIG_BAMAT_Pos)    /*!< Configure count value */

#define I3C_CONFIG_SADDR_Pos                (25)
#define I3C_CONFIG_SADDR_Msk                (0x7FU << I3C_CONFIG_SADDR_Pos)    /*!< Set the i2c 7-bit static address */

/**
  * @brief I3C_STATUS Register Bit Definition
  */
#define I3C_STATUS_STNOTSTOP_Pos            (0)
#define I3C_STATUS_STNOTSTOP                (0x01U << I3C_STATUS_STNOTSTOP_Pos) /*!< I3C Bus busy (active), or this device detected s0 / s1 error waiting for HDR exit mode. */
#define I3C_STATUS_STMSG_Pos                (1)
#define I3C_STATUS_STMSG                    (0x01U << I3C_STATUS_STMSG_Pos)     /*!< Listening to or responding to the bus */
#define I3C_STATUS_STCCCH_Pos               (2)
#define I3C_STATUS_STCCCH                   (0x01U << I3C_STATUS_STCCCH_Pos)    /*!< Automating the ccc */
#define I3C_STATUS_STREQRD_Pos              (3)
#define I3C_STATUS_STREQRD                  (0x01U << I3C_STATUS_STREQRD_Pos)   /*!< The current bus request is an SDR mode read (bus reads the data from this device), or if the device is sending an IBI. */
#define I3C_STATUS_STREQWR_Pos              (4)
#define I3C_STATUS_STREQWR                  (0x01U << I3C_STATUS_STREQWR_Pos)   /*!< The bus is in ENTDAA mode (whether I3C has dynamic address or not) */
#define I3C_STATUS_STDAA_Pos                (5)
#define I3C_STATUS_STDAA                    (0x01U << I3C_STATUS_STDAA_Pos)     /*!< Target address for master mode */
#define I3C_STATUS_STHDR_Pos                (6)
#define I3C_STATUS_STHDR                    (0x01U << I3C_STATUS_STHDR_Pos)     /*!< The bus is in HDR mode */

#define I3C_STATUS_START_Pos                (8)
#define I3C_STATUS_START                    (0x01U << I3C_STATUS_START_Pos)     /*!< The bus is detected to have a START or RESTART condition write 1 reset since the last time. */
#define I3C_STATUS_MATED_Pos                (9)
#define I3C_STATUS_MATED                    (0x01U << I3C_STATUS_MATED_Pos)     /*!< Since the last time the bit was cleared, the incoming address matches the I3C dynamic or I2C static address of this device. */
#define I3C_STATUS_STOP_Pos                 (10)
#define I3C_STATUS_STOP                     (0x01U << I3C_STATUS_STOP_Pos)      /*!< A STOP has been detected on the bus since the last time the bit was cleared (the STNOTSTOP bit indicates whether the bus is currently stopped). */
#define I3C_STATUS_RXPEND_Pos               (11)
#define I3C_STATUS_RXPEND                   (0x01U << I3C_STATUS_RXPEND_Pos)    /*!< Received data available */
#define I3C_STATUS_TXNF_Pos                 (12)
#define I3C_STATUS_TXNF                     (0x01U << I3C_STATUS_TXNF_Pos)      /*!< The TX FIFO can accept more data */
#define I3C_STATUS_DACHG_Pos                (13)
#define I3C_STATUS_DACHG                    (0x01U << I3C_STATUS_DACHG_Pos)     /*!< Dynamic addresses change when they are assigned, reassigned, or reset (lost). */
#define I3C_STATUS_CCC_Pos                  (14)
#define I3C_STATUS_CCC                      (0x01U << I3C_STATUS_CCC_Pos)       /*!< The I3C received from the device could not automatically process the CCC. */
#define I3C_STATUS_ERWAR_Pos                (15)
#define I3C_STATUS_ERWAR                    (0x01U << I3C_STATUS_ERWAR_Pos)     /*!< An error or warning has occurred; see the ERRWARN register for details */

#define I3C_STATUS_CHANDLED_Pos             (17)
#define I3C_STATUS_CHANDLED                 (0x01U << I3C_STATUS_CHANDLED_Pos)  /*!< The I3C slave device has handled the CCC */
#define I3C_STATUS_EVENT_Pos                (18)
#define I3C_STATUS_EVENT                    (0x01U << I3C_STATUS_EVENT_Pos)     /*!< An IBI, MR, or Hot-Join has been requested */

#define I3C_STATUS_EVDET_Pos                (20)
#define I3C_STATUS_EVDET_Msk                (0x03U << I3C_STATUS_EVDET_Pos)     /*!< Status of the current (EVENT is 1) or pending event */

#define I3C_STATUS_IBIDIS_Pos               (24)
#define I3C_STATUS_IBIDIS                   (0x01U << I3C_STATUS_IBIDIS_Pos)    /*!< The IBI is forbidden by the I3C bus controller via DISEC CCC */
#define I3C_STATUS_MRDIS_Pos                (25)
#define I3C_STATUS_MRDIS                    (0x01U << I3C_STATUS_MRDIS_Pos)     /*!< The bus controller request is forbidden by the I3C bus controller via DISEC CCC */

#define I3C_STATUS_HJDIS_Pos                (27)
#define I3C_STATUS_HJDIS                    (0x01U << I3C_STATUS_HJDIS_Pos)     /*!< Hot-Join is forbidden by the I3C bus controller via DISEC CCC */

#define I3C_STATUS_ACTSTATE_Pos             (28)
#define I3C_STATUS_ACTSTATE_Msk             (0x03U << I3C_STATUS_ACTSTATE_Pos)  /*!< The active state set by the I3C bus controller via ENTASn CCC */

/**
  * @brief I3C_CTRL Register Bit Definition
  */
#define I3C_CTRL_EVENT_Pos                  (0)
#define I3C_CTRL_EVENT_Msk                  (0x03U << I3C_CTRL_EVENT_Pos)    /*!< Setting it to a value other than 0 will request an event: */

#define I3C_CTRL_EXTDATA_Pos                (3)
#define I3C_CTRL_EXTDATA                    (0x01U << I3C_CTRL_EXTDATA_Pos)  /*!< After the IBI sends the IBIDATA, it continues to send extended data from the Tx FIFO. */

#define I3C_CTRL_IBIDATA_Pos                (8)
#define I3C_CTRL_IBIDATA_Msk                (0xFFU << I3C_CTRL_IBIDATA_Pos)  /*!< The IBI payload data bytes. */

#define I3C_CTRL_PENDINT_Pos                (16)
#define I3C_CTRL_PENDINT_Msk                (0x0FU << I3C_CTRL_PENDINT_Pos)  /*!< Configure the return value of the Pending Interrupt field of the GETSTATUS CCC command. */
#define I3C_CTRL_ACTSTATE_Pos               (20)
#define I3C_CTRL_ACTSTATE_Msk               (0x03U << I3C_CTRL_ACTSTATE_Pos) /*!< Configure the Activity state of the device as the return value of the Activity Mode field of the GETSTATUS CCC command. */

#define I3C_CTRL_VENDINFO_Pos               (24)
#define I3C_CTRL_VENDINFO_Msk               (0xFFU << I3C_CTRL_VENDINFO_Pos) /*!< The returned value of the Vendor Reserved field of the GETSTATUS CCC command is specified. */

/**
  * @brief I3C_INTSET Register Bit Definition
  */
#define I3C_INTSET_STIE_Pos                 (8)
#define I3C_INTSET_STIE                     (0x01U << I3C_INTSET_STIE_Pos)      /*!< Bus START/RESTART condition detected interrupt enabled */
#define I3C_INTSET_MATEDIE_Pos              (9)
#define I3C_INTSET_MATEDIE                  (0x01U << I3C_INTSET_MATEDIE_Pos)   /*!< Dynamic/static address matched interrupt enabled */
#define I3C_INTSET_SPIE_Pos                 (10)
#define I3C_INTSET_SPIE                     (0x01U << I3C_INTSET_SPIE_Pos)      /*!< Bus STOP condition detected interrupt enabled */
#define I3C_INTSET_RXPIE_Pos                (11)
#define I3C_INTSET_RXPIE                    (0x01U << I3C_INTSET_RXPIE_Pos)     /*!< Receive interrupt enabled */
#define I3C_INTSET_TXSIE_Pos                (12)
#define I3C_INTSET_TXSIE                    (0x01U << I3C_INTSET_TXSIE_Pos)     /*!< Transmit data (read) interrupt enabled */
#define I3C_INTSET_DACHGIE_Pos              (13)
#define I3C_INTSET_DACHGIE                  (0x01U << I3C_INTSET_DACHGIE_Pos)   /*!< Dynamic address changed interrupt enabled */
#define I3C_INTSET_CCCIE_Pos                (14)
#define I3C_INTSET_CCCIE                    (0x01U << I3C_INTSET_CCCIE_Pos)     /*!< CCC received interrupt enabled */
#define I3C_INTSET_ERWARIE_Pos              (15)
#define I3C_INTSET_ERWARIE                  (0x01U << I3C_INTSET_ERWARIE_Pos)   /*!< Error/Warning interrupt enabled */

#define I3C_INTSET_HANDLEDIE_Pos            (17)
#define I3C_INTSET_HANDLEDIE                (0x01U << I3C_INTSET_HANDLEDIE_Pos) /*!< CCC handled interrupt enabled */
#define I3C_INTSET_EVTIE_Pos                (18)
#define I3C_INTSET_EVTIE                    (0x01U << I3C_INTSET_EVTIE_Pos)     /*!< Event request (IBI or Hot-Join) interrupt enabled */

/**
  * @brief I3C_INTCLR Register Bit Definition
  */
#define I3C_INTCLR_STCLR_Pos                (8)
#define I3C_INTCLR_STCLR                    (0x01U << I3C_INTCLR_STCLR_Pos)      /*!< Clear START bit and disable the corresponding interrupt source */
#define I3C_INTCLR_MATEDCLR_Pos             (9)
#define I3C_INTCLR_MATEDCLR                 (0x01U << I3C_INTCLR_MATEDCLR_Pos)   /*!< Clear MATCHED bit and disable the corresponding interrupt source */
#define I3C_INTCLR_SPCLR_Pos                (10)
#define I3C_INTCLR_SPCLR                    (0x01U << I3C_INTCLR_SPCLR_Pos)      /*!< Clear STOP bit and disable the corresponding interrupt source */
#define I3C_INTCLR_RXPCLR_Pos               (11)
#define I3C_INTCLR_RXPCLR                   (0x01U << I3C_INTCLR_RXPCLR_Pos)     /*!< Clear RXPEND bit and disable the corresponding interrupt source */
#define I3C_INTCLR_TXSCLR_Pos               (12)
#define I3C_INTCLR_TXSCLR                   (0x01U << I3C_INTCLR_TXSCLR_Pos)     /*!< Clear TXSEND bit and disable the corresponding interrupt source */
#define I3C_INTCLR_DACHGCLR_Pos             (13)
#define I3C_INTCLR_DACHGCLR                 (0x01U << I3C_INTCLR_DACHGCLR_Pos)   /*!< Clear DACHG bit and disable the corresponding interrupt source */
#define I3C_INTCLR_CCCCLR_Pos               (14)
#define I3C_INTCLR_CCCCLR                   (0x01U << I3C_INTCLR_CCCCLR_Pos)     /*!< Clear CCC bit and disable the corresponding interrupt source */
#define I3C_INTCLR_ERWARCLR_Pos             (15)
#define I3C_INTCLR_ERWARCLR                 (0x01U << I3C_INTCLR_ERWARCLR_Pos)   /*!< Clear ERRWARN bit and disable the corresponding interrupt source */

#define I3C_INTCLR_HANDLEDCLR_Pos           (17)
#define I3C_INTCLR_HANDLEDCLR               (0x01U << I3C_INTCLR_HANDLEDCLR_Pos) /*!< Clear CHANDLED bit and disable the corresponding interrupt source */
#define I3C_INTCLR_EVTCLR_Pos               (18)
#define I3C_INTCLR_EVTCLR                   (0x01U << I3C_INTCLR_EVTCLR_Pos)     /*!< Clear EVENT bit and disable the corresponding interrupt source */

/**
  * @brief I3C_INTMASKED Register Bit Definition
  */
#define I3C_INTMASKED_STMSK_Pos             (8)
#define I3C_INTMASKED_STMSK                 (0x01U << I3C_INTMASKED_STMSK_Pos)      /*!< Bus START/RESTART condition detected flag */
#define I3C_INTMASKED_MATEDMSK_Pos          (9)
#define I3C_INTMASKED_MATEDMSK              (0x01U << I3C_INTMASKED_MATEDMSK_Pos)   /*!< Dynamic/static address matched interrupt flag */
#define I3C_INTMASKED_SPMSK_Pos             (10)
#define I3C_INTMASKED_SPMSK                 (0x01U << I3C_INTMASKED_SPMSK_Pos)      /*!< Bus STOP condition detected interrupt flag */
#define I3C_INTMASKED_RXPMSK_Pos            (11)
#define I3C_INTMASKED_RXPMSK                (0x01U << I3C_INTMASKED_RXPMSK_Pos)     /*!< Receive interrupt flag */
#define I3C_INTMASKED_TXSMSK_Pos            (12)
#define I3C_INTMASKED_TXSMSK                (0x01U << I3C_INTMASKED_TXSMSK_Pos)     /*!< Transmit data interrupt flag */
#define I3C_INTMASKED_DACHGMSK_Pos          (13)
#define I3C_INTMASKED_DACHGMSK              (0x01U << I3C_INTMASKED_DACHGMSK_Pos)   /*!< Dynamic address changed interrupt flag */
#define I3C_INTMASKED_CCCMSK_Pos            (14)
#define I3C_INTMASKED_CCCMSK                (0x01U << I3C_INTMASKED_CCCMSK_Pos)     /*!< CCC received interrupt flag */
#define I3C_INTMASKED_ERWARMSK_Pos          (15)
#define I3C_INTMASKED_ERWARMSK              (0x01U << I3C_INTMASKED_ERWARMSK_Pos)   /*!< Error/Warning interrupt flag */

#define I3C_INTMASKED_HANDLEDMSK_Pos        (17)
#define I3C_INTMASKED_HANDLEDMSK            (0x01U << I3C_INTMASKED_HANDLEDMSK_Pos) /*!< CHANDLED interrupt flag */
#define I3C_INTMASKED_EVTMSK_Pos            (18)
#define I3C_INTMASKED_EVTMSK                (0x01U << I3C_INTMASKED_EVTMSK_Pos)     /*!< Event request interrupt flag */

/**
  * @brief I3C_ERWAR Register Bit Definition
  */
#define I3C_ERWAR_ORUN_Pos                  (0)
#define I3C_ERWAR_ORUN                      (0x01U << I3C_ERWAR_ORUN_Pos)     /*!< The application failed to read the send data in time, resulting in a FIFO overflow */

#define I3C_ERWAR_URUN_Pos                  (1)
#define I3C_ERWAR_URUN                      (0x01U << I3C_ERWAR_URUN_Pos)     /*!< The application failed to provide send data in a timely manner, resulting in an underload of the FIFO */
#define I3C_ERWAR_URUNNACK_Pos              (2)
#define I3C_ERWAR_URUNNACK                  (0x01U << I3C_ERWAR_URUNNACK_Pos) /*!< The FIFO is underloaded when receiving the address of the read command, so the I3C slave device does not respond (NACK) to the address */
#define I3C_ERWAR_TERM_Pos                  (3)
#define I3C_ERWAR_TERM                      (0x01U << I3C_ERWAR_TERM_Pos)     /*!< The I3C bus controller terminates the read before the I3C sends data from the device to END */
#define I3C_ERWAR_INVST_Pos                 (4)
#define I3C_ERWAR_INVST                     (0x01U << I3C_ERWAR_INVST_Pos)    /*!< Invalid START condition occurs (SCL drops when SDA is high under STOP condition) */

#define I3C_ERWAR_SPAR_Pos                  (8)
#define I3C_ERWAR_SPAR                      (0x01U << I3C_ERWAR_SPAR_Pos)     /*!< The SDR message sent by the I3C controller has a parity error. Or the bus controller fails to drive the clock over 100us during SDR Read, resulting in read abort (timeout) */
#define I3C_ERWAR_S0S1_Pos                  (11)
#define I3C_ERWAR_S0S1                      (0x01U << I3C_ERWAR_S0S1_Pos)     /*!< An S0 or S1 error occurs, and the I3C is locked from the device, waiting for the HDR to exit the mode */

#define I3C_ERWAR_OREAD_Pos                 (16)
#define I3C_ERWAR_OREAD                     (0x01U << I3C_ERWAR_OREAD_Pos)    /*!< RDATAB and RDATAH registers are read in empty time */
#define I3C_ERWAR_OWRITE_Pos                (17)
#define I3C_ERWAR_OWRITE                    (0x01U << I3C_ERWAR_OWRITE_Pos)   /*!< Write data overflow: When full, the WDATAB/WDATABE and WDATAH/WDATAHE registers are written */

/**
  * @brief I3C_DMACTRL Register Bit Definition
  */
#define I3C_DMACTRL_DMAFB_Pos               (0)
#define I3C_DMACTRL_DMAFB_Msk               (0x03U << I3C_DMACTRL_DMAFB_Pos)  /*!< DMA read enable: */
#define I3C_DMACTRL_DMAFB_0                 (0x00U << I3C_DMACTRL_DMAFB_Pos)  /*!< DMA disable */
#define I3C_DMACTRL_DMAFB_1                 (0x01U << I3C_DMACTRL_DMAFB_Pos)  /*!< DMA is enabled for 1 frame messages and then automatically zeroed out when a STOP or START condition is detected */
#define I3C_DMACTRL_DMAFB_2                 (0x02U << I3C_DMACTRL_DMAFB_Pos)  /*!< DMA is enabled until this field is set to 0 */

#define I3C_DMACTRL_DMATB_Pos               (2)
#define I3C_DMACTRL_DMATB_Msk               (0x03U << I3C_DMACTRL_DMATB_Pos)  /*!< DMA write enable: */
#define I3C_DMACTRL_DMATB_0                 (0x00U << I3C_DMACTRL_DMATB_Pos)  /*!< DMA disable */
#define I3C_DMACTRL_DMATB_1                 (0x01U << I3C_DMACTRL_DMATB_Pos)  /*!< DMA is enabled for 1 frame messages and then automatically zeroed out when a STOP or START condition is detected */
#define I3C_DMACTRL_DMATB_2                 (0x02U << I3C_DMACTRL_DMATB_Pos)  /*!< DMA is enabled until this field is set to 0 */

#define I3C_DMACTRL_DMAWTH_Pos              (4)
#define I3C_DMACTRL_DMAWTH_Msk              (0x03U << I3C_DMACTRL_DMAWTH_Pos) /*!< Bit width of data transmitted by DMA: */
#define I3C_DMACTRL_DMAWTH_0                (0x00U << I3C_DMACTRL_DMAWTH_Pos) /*!< Byte (default) */
#define I3C_DMACTRL_DMAWTH_1                (0x01U << I3C_DMACTRL_DMAWTH_Pos) /*!< Byte (default) */
#define I3C_DMACTRL_DMAWTH_2                (0x02U << I3C_DMACTRL_DMAWTH_Pos) /*!< Half word (16 bits) */

/**
  * @brief I3C_DATACTRL Register Bit Definition
  */
#define I3C_DATACTRL_FLUSHTB_Pos            (0)
#define I3C_DATACTRL_FLUSHTB_Msk            (0x01U << I3C_DATACTRL_FLUSHTB_Pos) /*!< Refresh TX FIFO */
#define I3C_DATACTRL_FLUSHFB_Pos            (1)
#define I3C_DATACTRL_FLUSHFB_Msk            (0x01U << I3C_DATACTRL_FLUSHFB_Pos) /*!< Refresh RX FIFO */

#define I3C_DATACTRL_UNLOCK_Pos             (3)
#define I3C_DATACTRL_UNLOCK_Msk             (0x01U << I3C_DATACTRL_UNLOCK_Pos)  /*!< Write bits 7 to 4 are valid */
#define I3C_DATACTRL_TXTRIG_Pos             (4)
#define I3C_DATACTRL_TXTRIG_Msk             (0x03U << I3C_DATACTRL_TXTRIG_Pos)  /*!< TX FIFO trigger level: */
#define I3C_DATACTRL_TXTRIG_0               (0x00U << I3C_DATACTRL_TXTRIG_Pos)  /*!< Triggered when empty */
#define I3C_DATACTRL_TXTRIG_1               (0x01U << I3C_DATACTRL_TXTRIG_Pos)  /*!< Triggered when <= 1/4 full */
#define I3C_DATACTRL_TXTRIG_2               (0x02U << I3C_DATACTRL_TXTRIG_Pos)  /*!< Triggered when <= 1/2 full */
#define I3C_DATACTRL_TXTRIG_3               (0x03U << I3C_DATACTRL_TXTRIG_Pos)  /*!< Triggered when 1 or less than full */
#define I3C_DATACTRL_RXTRIG_Pos             (6)
#define I3C_DATACTRL_RXTRIG_Msk             (0x03U << I3C_DATACTRL_RXTRIG_Pos)  /*!< RX FIFO trigger level: */
#define I3C_DATACTRL_RXTRIG_0               (0x00U << I3C_DATACTRL_RXTRIG_Pos)  /*!< Triggered when empty */
#define I3C_DATACTRL_RXTRIG_1               (0x01U << I3C_DATACTRL_RXTRIG_Pos)  /*!< Triggered when >= 1/4 full */
#define I3C_DATACTRL_RXTRIG_2               (0x02U << I3C_DATACTRL_RXTRIG_Pos)  /*!< Triggered when >= 1/2 full */
#define I3C_DATACTRL_RXTRIG_3               (0x03U << I3C_DATACTRL_RXTRIG_Pos)  /*!< Triggered when >= 3/4 full */

#define I3C_DATACTRL_TXCNT_Pos              (16)
#define I3C_DATACTRL_TXCNT_Msk              (0x1FU << I3C_DATACTRL_TXCNT_Pos)   /*!< The number of bytes sent in the FIFO */

#define I3C_DATACTRL_RXCNT_Pos              (24)
#define I3C_DATACTRL_RXCNT_Msk              (0x1FU << I3C_DATACTRL_RXCNT_Pos)   /*!< The number of bytes received in the FIFO */

#define I3C_DATACTRL_TXFULL_Pos             (30)
#define I3C_DATACTRL_TXFULL                 (0x01U << I3C_DATACTRL_TXFULL_Pos)  /*!< TX FIFO full/not full */

#define I3C_DATACTRL_RXEMPTY_Pos            (31)
#define I3C_DATACTRL_RXEMPTY                (0x01U << I3C_DATACTRL_RXEMPTY_Pos) /*!< RX FIFO empty/not empty */

/**
  * @brief I3C_WDATAB Register Bit Definition
  */
#define I3C_WDATAB_DATA_Pos                 (0)
#define I3C_WDATAB_DATA_Msk                 (0xFFU << I3C_WDATAB_DATA_Pos) /*!< Bytes of data sent to the bus controller */
#define I3C_WDATAB_END_Pos                  (8)
#define I3C_WDATAB_END                      (0x01U << I3C_WDATAB_END_Pos)  /*!< DATA is the final byte */

#define I3C_WDATAB_END2_Pos                 (16)
#define I3C_WDATAB_END2                     (0x01U << I3C_WDATAB_END2_Pos) /*!< DATA is the final byte 8 */

/**
  * @brief I3C_WDATABE Register Bit Definition
  */
#define I3C_WDATABE_DATA_Pos                (0)
#define I3C_WDATABE_DATA                    (0xFFU << I3C_WDATABE_DATA_Pos) /*!< The final data byte sent to the bus controller */

/**
  * @brief I3C_WDATAH Register Bit Definition
  */
#define I3C_WDATAH_DATA0_Pos                (0)
#define I3C_WDATAH_DATA0_Msk                (0xFFU << I3C_WDATAH_DATA0_Pos) /*!< The first byte of data sent to the bus controller */

#define I3C_WDATAH_DATA1_Pos                (8)
#define I3C_WDATAH_DATA1_Msk                (0xFFU << I3C_WDATAH_DATA1_Pos) /*!< The second byte of data sent to the bus controller */

#define I3C_WDATAH_END_Pos                  (16)
#define I3C_WDATAH_END                      (0x01U << I3C_WDATAH_END_Pos)   /*!< DATA1 is the final byte */

/**
  * @brief I3C_WDATAHE Register Bit Definition
  */
#define I3C_WDATAHE_DATA0_Pos               (0)
#define I3C_WDATAHE_DATA0_Msk               (0xFFU << I3C_WDATAHE_DATA0_Pos) /*!< The first byte of data sent to the bus controller */
#define I3C_WDATAHE_DATA1_Pos               (8)
#define I3C_WDATAHE_DATA1_Msk               (0xFFU << I3C_WDATAHE_DATA1_Pos) /*!< The second (and final) data byte sent to the bus controller */

/**
  * @brief I3C_RDATAB Register Bit Definition
  */
#define I3C_RDATAB_DATA0_Pos                (0)
#define I3C_RDATAB_DATA0_Msk                (0xFFU << I3C_RDATAB_DATA0_Pos) /*!< Read data bytes from the RX FIFO */

/**
  * @brief I3C_RDATAH Register Bit Definition
  */
#define I3C_RDATAH_LSB_Pos                  (0)
#define I3C_RDATAH_LSB_Msk                  (0xFFU << I3C_RDATAH_LSB_Pos) /*!< Read the first byte from the RX FIFO */
#define I3C_RDATAH_MSB_Pos                  (8)
#define I3C_RDATAH_MSB_Msk                  (0xFFU << I3C_RDATAH_MSB_Pos) /*!< Read the second byte from the RX FIFO */

/**
  * @brief I3C_WDATAB1 Register Bit Definition
  */
#define I3C_WDATAB1_DATA_Pos                (0)
#define I3C_WDATAB1_DATA                    (0xFFU << I3C_WDATAB1_DATA_Pos) /*!< Bytes sent to the bus controller */

/**
  * @brief I3C_CAPABILITIES2 Register Bit Definition
  */
#define I3C_CAPABILITIES2_MAPCNT_Pos        (0)
#define I3C_CAPABILITIES2_MAPCNT            (0x0FU << I3C_CAPABILITIES2_MAPCNT_Pos)   /*!< Indicates the number of supported address mappings */
#define I3C_CAPABILITIES2_I2C10B_Pos        (4)
#define I3C_CAPABILITIES2_I2C10B            (0x01U << I3C_CAPABILITIES2_I2C10B_Pos)   /*!< Supports I2C 10-bit addresses */
#define I3C_CAPABILITIES2_I2CRST_Pos        (5)
#define I3C_CAPABILITIES2_I2CRST            (0x01U << I3C_CAPABILITIES2_I2CRST_Pos)   /*!< Supports the reset of I2C applications */
#define I3C_CAPABILITIES2_I2CDEVID_Pos      (6)
#define I3C_CAPABILITIES2_I2CDEVID          (0x01U << I3C_CAPABILITIES2_I2CDEVID_Pos) /*!< Supports the I2C device ID */
#define I3C_CAPABILITIES2_IBIEXT_Pos        (8)
#define I3C_CAPABILITIES2_IBIEXT            (0x01U << I3C_CAPABILITIES2_IBIEXT_Pos)   /*!< supports IBI extended data */
#define I3C_CAPABILITIES2_IBIFIFO_Pos       (9)
#define I3C_CAPABILITIES2_IBIFIFO           (0x01U << I3C_CAPABILITIES2_IBIFIFO_Pos)  /*!< Support level 2 FIFO for IBI extended data */

#define I3C_CAPABILITIES2_V1P1_Pos          (16)
#define I3C_CAPABILITIES2_V1P1              (0x01U << I3C_CAPABILITIES2_V1P1_Pos)     /*!< Support GETCAPS CCC (I3C protocol v1.1) */
#define I3C_CAPABILITIES2_SLVRST_Pos        (17)
#define I3C_CAPABILITIES2_SLVRST            (0x01U << I3C_CAPABILITIES2_SLVRST_Pos)   /*!< Support reset from device (I3C protocol v1.1) */
#define I3C_CAPABILITIES2_GROUP_Pos         (18)
#define I3C_CAPABILITIES2_GROUP             (0x03U << I3C_CAPABILITIES2_GROUP_Pos)    /*!< Number of GROUP addresses supported */

#define I3C_CAPABILITIES2_AASA_Pos          (21)
#define I3C_CAPABILITIES2_AASA              (0x01U << I3C_CAPABILITIES2_AASA_Pos)     /*!< SETAASA is supported */
#define I3C_CAPABILITIES2_SSTSUB_Pos        (22)
#define I3C_CAPABILITIES2_SSTSUB            (0x01U << I3C_CAPABILITIES2_SSTSUB_Pos)   /*!< Support subscriber from device to device */
#define I3C_CAPABILITIES2_SSTWR_Pos         (23)
#define I3C_CAPABILITIES2_SSTWR             (0x01U << I3C_CAPABILITIES2_SSTWR_Pos)    /*!< Supports write from device to device */

/**
  * @brief I3C_CAPABILITIES Register Bit Definition
  */
#define I3C_CAPABILITIES_IDENA_Pos          (0)
#define I3C_CAPABILITIES_IDENA              (0x03U << I3C_CAPABILITIES_IDENA_Pos)     /*!< Processing method of 48-bit ID */
#define I3C_CAPABILITIES_IDREG_Pos          (2)
#define I3C_CAPABILITIES_IDREG              (0x0FU << I3C_CAPABILITIES_IDREG_Pos)     /*!< Select the register configuration ID information field */
#define I3C_CAPABILITIES_HDRSUPP_Pos        (6)
#define I3C_CAPABILITIES_HDRSUPP            (0x07U << I3C_CAPABILITIES_HDRSUPP_Pos)   /*!< Whether to support HDR and its type */
#define I3C_CAPABILITIES_MASTER_Pos         (9)
#define I3C_CAPABILITIES_MASTER             (0x01U << I3C_CAPABILITIES_MASTER_Pos)    /*!< Support the master function */
#define I3C_CAPABILITIES_SADDR_Pos          (10)
#define I3C_CAPABILITIES_SADDR              (0x03U << I3C_CAPABILITIES_SADDR_Pos)     /*!< Static address */

#define I3C_CAPABILITIES_CCCHANDLE_Pos      (12)
#define I3C_CAPABILITIES_CCCHANDLE          (0x0FU << I3C_CAPABILITIES_CCCHANDLE_Pos) /*!< Indicates that the CCC is handled by I3C from a device or by an application */

#define I3C_CAPABILITIES_IBIMRHJ_Pos        (16)
#define I3C_CAPABILITIES_IBIMRHJ            (0x1FU << I3C_CAPABILITIES_IBIMRHJ_Pos)   /*!< Supported event types: */
#define I3C_CAPABILITIES_TIMECTRL_Pos       (21)
#define I3C_CAPABILITIES_TIMECTRL           (0x01U << I3C_CAPABILITIES_TIMECTRL_Pos)  /*!< At least one time control mode is supported */
#define I3C_CAPABILITIES_EXTFIFO_Pos        (23)
#define I3C_CAPABILITIES_EXTFIFO            (0x07U << I3C_CAPABILITIES_EXTFIFO_Pos)   /*!< Supports external FIFO */
#define I3C_CAPABILITIES_FIFOTX_Pos         (26)
#define I3C_CAPABILITIES_FIFOTX             (0x03U << I3C_CAPABILITIES_FIFOTX_Pos)    /*!< Supports TX FIFO and its size */
#define I3C_CAPABILITIES_FIFORX_Pos         (28)
#define I3C_CAPABILITIES_FIFORX             (0x03U << I3C_CAPABILITIES_FIFORX_Pos)    /*!< Whether RX FIFO is supported and its size */

#define I3C_CAPABILITIES_INT_Pos            (30)
#define I3C_CAPABILITIES_INT                (0x01U << I3C_CAPABILITIES_INT_Pos)       /*!< Support interrupt */
#define I3C_CAPABILITIES_DMA_Pos            (31)
#define I3C_CAPABILITIES_DMA                (0x01U << I3C_CAPABILITIES_DMA_Pos)       /*!< Support DMA */

/**
  * @brief I3C_DYNADDR Register Bit Definition
  */
#define I3C_DYNADDR_DAVLD_Pos               (0)
#define I3C_DYNADDR_DAVLD                   (0x01U << I3C_DYNADDR_DAVLD_Pos)  /*!< A dynamic address was assigned when DADDR was read or set */
#define I3C_DYNADDR_DADDR_Pos               (1)
#define I3C_DYNADDR_DADDR_Msk               (0x7FU << I3C_DYNADDR_DADDR_Pos)  /*!< If DAVALID is 1, the field is the assigned dynamic address */
#define I3C_DYNADDR_DCAUSE_Pos              (8)
#define I3C_DYNADDR_DCAUSE                  (0x07U << I3C_DYNADDR_DCAUSE_Pos) /*!< Indicates the cause of the dynamic address change */

#define I3C_DYNADDR_KEY_Pos                 (16)
#define I3C_DYNADDR_KEY                     (0xFFFFU << I3C_DYNADDR_KEY_Pos)  /*!< Mapping Address Setting */

/**
  * @brief I3C_MAXLIMITS Register Bit Definition
  */
#define I3C_MAXLIMITS_MAXRD_Pos             (0)
#define I3C_MAXLIMITS_MAXRD_Msk             (0xFFFU << I3C_MAXLIMITS_MAXRD_Pos) /*!< Maximum number of bytes read. The value ranges from 16 to 4095 */
#define I3C_MAXLIMITS_MAXWR_Pos             (16)
#define I3C_MAXLIMITS_MAXWR_Msk             (0xFFFU << I3C_MAXLIMITS_MAXWR_Pos) /*!< Maximum number of bytes written. The value ranges from 8 to 4095 */

/**
  * @brief I3C_PARTNO Register Bit Definition
  */
#define I3C_PARTNO_Pos                      (0)
#define I3C_PARTNO_Msk                      (0xFFFFFFFFU << I3C_PARTNO_Pos) /*!< Set Part Number. 0 is usually invalid */

/**
  * @brief I3C_IDEXT Register Bit Definition
  */
#define I3C_IDEXT_DCR_Pos                   (8)
#define I3C_IDEXT_DCR_Msk                   (0xFFU << I3C_IDEXT_DCR_Pos) /*!< Configure DCR (Device type) */
#define I3C_IDEXT_BCR_Pos                   (16)
#define I3C_IDEXT_BCR_Msk                   (0xFFU << I3C_IDEXT_BCR_Pos) /*!< Configure BCR (bus capability) */

/**
  * @brief I3C_VENDORID Register Bit Definition
  */
#define I3C_VENDORID_VID_Pos                (0)
#define I3C_VENDORID_VID_Msk                (0x7FFFU << I3C_VENDORID_VID_Pos) /*!< Configure 15 bit MIPI Vendor ID */

/**
  * @brief I3C_DIV Register Bit Definition
  */
#define I3C_DIV_Pos                         (0)
#define I3C_DIV_Msk                         (0xFFU << I3C_DIV_Pos) /*!< CLK_SLOW Frequency division */

/**
  * @brief I3C_SPKLEN Register Bit Definition
  */
#define I3C_SPKLEN_SPKLEN_Pos               (0)
#define I3C_SPKLEN_SPKLEN_Msk               (0xFFU << I3C_SPKLEN_SPKLEN_Pos) /*!< Configure digital filter at the SCL and SDA inputs */

#define I3C_SPKLEN_FLT_EN_Pos               (16)
#define I3C_SPKLEN_FLT_EN                   (0x01U << I3C_SPKLEN_FLT_EN_Pos) /*!< Filter enable */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif
/*----------------------------------------------------------------------------*/
