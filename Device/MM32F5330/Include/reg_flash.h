/***********************************************************************************************************************
    @file     reg_flash.h
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

#ifndef __REG_FLASH_H
#define __REG_FLASH_H

/* Files includes ------------------------------------------------------------*/
#include "core_starmc1.h"

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
/* anonymous unions are enabled by default -----------------------------------*/
#else
#warning Not supported compiler type
#endif

/**
  * @brief  MM32 MCU Memory/Peripherals mapping
  */
#define FLASH_BASE                      (0x08000000U)              /*!< FLASH base address in the alias region */
#define SRAM_BASE                       (0x20000000U)              /*!< SRAM base address in the alias region */

/**
  * @brief FLASH Base Address Definition
  */
#define FLASH_REG_BASE                  (AHB1PERIPH_BASE + 0x2000) /*!< Base Address: 0x40022000 */

/**
  * @brief OPTB Base Address Definition
  */
#define OB_BASE                         (0x1FFFF800U)              /*!< Flash Option Bytes base address */

/**
  * @brief FLASH Registers Structure Definition
  */
typedef struct
{
    __IO uint32_t ACR;                 /*!< Access control Register                        offset: 0x00 */
    __IO uint32_t KEYR;                /*!< Key Register                                   offset: 0x04 */
    __IO uint32_t OPTKEYR;             /*!< Option byte key Register                       offset: 0x08 */
    __IO uint32_t SR;                  /*!< State Register                                 offset: 0x0C */
    __IO uint32_t CR;                  /*!< Control Register                               offset: 0x10 */
    __IO uint32_t AR;                  /*!< Address Register                               offset: 0x14 */
    __IO uint32_t RESERVED0x18;        /*!< RESERVED                                       offset: 0x18 */
    __IO uint32_t OBR;                 /*!< Option bytes Register                          offset: 0x1C */
    __IO uint32_t WRPR;                /*!< Write protect Register                         offset: 0x20 */
    __IO uint32_t RESERVED0x24[71];    /*!< RESERVED                                       offset: 0x24-0x13C */
    __IO uint32_t ECC_CR;              /*!< Option bytes Register                          offset: 0x140 */
    __IO uint32_t ECC_PR;              /*!< Write protect Register                         offset: 0x144 */
    __IO uint32_t ECC_EER;             /*!< Option bytes Register                          offset: 0x148 */
    __IO uint32_t ECC_SR;              /*!< Write protect Register                         offset: 0x14C */
    __IO uint32_t ECC_ADDRR;           /*!< Option bytes Register                          offset: 0x150 */
    __IO uint32_t ECC_SYNR;            /*!< Write protect Register                         offset: 0x154 */
    __IO uint32_t ECC_DLR;             /*!< Option bytes Register                          offset: 0x158 */
    __IO uint32_t ECC_DHR;             /*!< Write protect Register                         offset: 0x15C */
} FLASH_TypeDef;

/**
  * @brief  OPT Structure Definition
  */
typedef struct
{
    __IO uint16_t RDP;                 /*!< Read Protect,                                  offset: 0x00 */
    __IO uint16_t USER;                /*!< User option byte,                              offset: 0x02 */
    __IO uint16_t Data0;               /*!< User data 0,                                   offset: 0x04 */
    __IO uint16_t Data1;               /*!< User data 1,                                   offset: 0x06 */
    __IO uint16_t WRP0;                /*!< Flash write protection option byte 0,          offset: 0x08 */
    __IO uint16_t WRP1;                /*!< Flash write protection option byte 1,          offset: 0x0A */
    __IO uint16_t WRP2;                /*!< Flash write protection option byte 2,          offset: 0x0C */
    __IO uint16_t WRP3;                /*!< Flash write protection option byte 3,          offset: 0x0E */
    __IO uint16_t RESERVED0x10[20];    /*!< RESERVED                                       offset: 0x10-0x36 */
    __IO uint16_t Boot_addr1;          /*!< Boot_addr[7:0]                                 offset: 0x38 */
    __IO uint16_t Boot_addr2;          /*!< Boot_addr[15:8]                                offset: 0x3A */
    __IO uint16_t Boot_addr3;          /*!< Boot_addr[23:16]                               offset: 0x3C */
    __IO uint16_t Boot_addr4;          /*!< Boot_addr[31:24]                               offset: 0x3E */
} OB_TypeDef;

/**
  * @brief  CACHE BYTES Structure Definition
  */

/**
  * @brief FLASH type pointer Definition
  */
#define FLASH                           ((FLASH_TypeDef *)FLASH_REG_BASE)

/**
  * @brief OPTB type pointer Definition
  */
#define OB                              ((OB_TypeDef *)OB_BASE)

/**
  * @brief FLASH_ACR Register Bit Definition
  */
#define FLASH_ACR_LATENCY_Pos           (0)
#define FLASH_ACR_LATENCY_Msk           (0x07U << FLASH_ACR_LATENCY_Pos) /*!< LATENCY[2:0] bits (Latency) */
#define FLASH_ACR_LATENCY_0             (0x00U << FLASH_ACR_LATENCY_Pos) /*!< 0 waiting state */
#define FLASH_ACR_LATENCY_1             (0x01U << FLASH_ACR_LATENCY_Pos) /*!< 1 waiting state */
#define FLASH_ACR_LATENCY_2             (0x02U << FLASH_ACR_LATENCY_Pos) /*!< 2 waiting state */
#define FLASH_ACR_LATENCY_3             (0x03U << FLASH_ACR_LATENCY_Pos) /*!< 3 waiting state */
#define FLASH_ACR_LATENCY_4             (0x04U << FLASH_ACR_LATENCY_Pos) /*!< 4 waiting state */
#define FLASH_ACR_LATENCY_5             (0x05U << FLASH_ACR_LATENCY_Pos) /*!< 5 waiting state */
#define FLASH_ACR_LATENCY_6             (0x06U << FLASH_ACR_LATENCY_Pos) /*!< 6 waiting state */
#define FLASH_ACR_LATENCY_7             (0x07U << FLASH_ACR_LATENCY_Pos) /*!< 7 waiting state */

/**
  * @brief FLASH_KEYR Register Bit Definition
  */
#define FLASH_KEYR_FKEYR_Pos            (0)
#define FLASH_KEYR_FKEYR_Msk            (0xFFFFFFFFU << FLASH_KEYR_FKEYR_Pos) /*!< FLASH Key */

/**
  * @brief FLASH_OPTKEYR Register Bit Definition
  */
#define FLASH_OPTKEYR_Pos               (0)
#define FLASH_OPTKEYR_Msk               (0xFFFFFFFFU << FLASH_OPTKEYR_Pos) /*!< Option Byte Key */

/**
  * @brief FLASH_SR Register Bit Definition
  */
#define FLASH_SR_BSY_Pos                (0)
#define FLASH_SR_BSY_Msk                (0x01U << FLASH_SR_BSY_Pos)      /*!< Busy */
#define FLASH_SR_PGERR_Pos              (2)
#define FLASH_SR_PGERR_Msk              (0x01U << FLASH_SR_PGERR_Pos)    /*!< Programming Error */
#define FLASH_SR_WRPRTERR_Pos           (4)
#define FLASH_SR_WRPRTERR_Msk           (0x01U << FLASH_SR_WRPRTERR_Pos) /*!< Write Protection Error */
#define FLASH_SR_EOP_Pos                (5)
#define FLASH_SR_EOP_Msk                (0x01U << FLASH_SR_EOP_Pos)      /*!< End of operation */
#define FLASH_SR_ECCERR_Pos             (6)
#define FLASH_SR_ECCERR_Msk             (0x01U << FLASH_SR_ECCERR_Pos)   /*!< Ecc Programming error */

/**
  * @brief FLASH_CR Register Bit Definition
  */
#define FLASH_CR_PG_Pos                 (0)
#define FLASH_CR_PG_Msk                 (0x01U << FLASH_CR_PG_Pos)     /*!< Programming */
#define FLASH_CR_PER_Pos                (1)
#define FLASH_CR_PER_Msk                (0x01U << FLASH_CR_PER_Pos)    /*!< Page Erase */
#define FLASH_CR_MER_Pos                (2)
#define FLASH_CR_MER_Msk                (0x01U << FLASH_CR_MER_Pos)    /*!< Mass Erase */
#define FLASH_CR_OPTPG_Pos              (4)
#define FLASH_CR_OPTPG_Msk              (0x01U << FLASH_CR_OPTPG_Pos)  /*!< Option Byte Programming */
#define FLASH_CR_OPTER_Pos              (5)
#define FLASH_CR_OPTER_Msk              (0x01U << FLASH_CR_OPTER_Pos)  /*!< Option Byte Erase */
#define FLASH_CR_STRT_Pos               (6)
#define FLASH_CR_STRT_Msk               (0x01U << FLASH_CR_STRT_Pos)   /*!< Start */
#define FLASH_CR_LOCK_Pos               (7)
#define FLASH_CR_LOCK_Msk               (0x01U << FLASH_CR_LOCK_Pos)   /*!< Lock */
#define FLASH_CR_OPTWRE_Pos             (9)
#define FLASH_CR_OPTWRE_Msk             (0x01U << FLASH_CR_OPTWRE_Pos) /*!< Option Bytes Write Enable */

/**
  * @brief FLASH_AR Register Bit Definition
  */
#define FLASH_AR_FAR_Pos                (0)
#define FLASH_AR_FAR_Msk                (0xFFFFFFFFU << FLASH_AR_FAR_Pos) /*!< Flash Address */

/**
  * @brief FLASH_OBR Register Bit Definition
  */
#define FLASH_OBR_OPTERR_Pos            (0)
#define FLASH_OBR_OPTERR_Msk            (0x01U << FLASH_OBR_OPTERR_Pos)      /*!< Option Byte Error */
#define FLASH_OBR_RDPRT_Pos             (1)
#define FLASH_OBR_RDPRT_Msk             (0x01U << FLASH_OBR_RDPRT_Pos)       /*!<  Read protection level status */

#define FLASH_OBR_USER_Pos              (2)
#define FLASH_OBR_USER_Msk              (0xFFU << FLASH_OBR_USER_Pos)        /*!< User Option Bytes */

#define FLASH_OBR_WDG_SW                (0x01U << FLASH_OBR_USER_Pos)        /*!< WDG_SW */
#define FLASH_OBR_RST_STOP              (0x02U << FLASH_OBR_USER_Pos)        /*!< nRST_STOP */
#define FLASH_OBR_RST_STDBY             (0x04U << FLASH_OBR_USER_Pos)        /*!< nRST_STDBY */

#define FLASH_OBR_FLASH_ECCEN           (0x20U << FLASH_OBR_FLASH_ECCEN_Pos) /*!< FLASH ECC Enable */
#define FLASH_OBR_SRAM_ECCEN            (0x40U << FLASH_OBR_SRAM_ECCEN_Pos)  /*!< SRAM ECC Enable */

#define FLASH_OBR_Data0_Pos             (10)
#define FLASH_OBR_Data0_Msk             (0xFFU << FLASH_OBR_Data0_Pos)       /*!< User data storage option byte */
#define FLASH_OBR_Data1_Pos             (18)
#define FLASH_OBR_Data1_Msk             (0xFFU << FLASH_OBR_Data1_Pos)       /*!< User data storage option byte */

/**
  * @brief FLASH_WRPR Register Bit Definition
  */
#define FLASH_WRPR_WRP_Pos              (0)
#define FLASH_WRPR_WRP_Msk              (0xFFFFFFFFU << FLASH_WRPR_WRP_Pos) /*!< Write Protect */

/**
  * @brief FLASH_ECC_CR Register Bit Definition
  */
#define FLASH_ECC_CR_RERR_Pos           (1)
#define FLASH_ECC_CR_RERR_Msk           (0x01U << FLASH_ECC_CR_RERR_Pos) /*!< Check whether the error information for the ECC_ADDRR, ECC_DR, ECC_SYNR register is updated */

/**
  * @brief FLASH_ECC_PR Register Bit Definition
  */
#define FLASH_ECC_PR_PRC_Pos            (0)
#define FLASH_ECC_PR_PRC_Msk            (0x01U << FLASH_ECC_PR_PRC_Pos) /*!< ECC_CR write is enabled */

/**
  * @brief FLASH_ECC_EER Register Bit Definition
  */
#define FLASH_ECC_EER_1BITIE_Pos        (0)
#define FLASH_ECC_EER_1BITIE_Msk        (0x01U << FLASH_ECC_EER_1BITIE_Pos) /*!< An event request is generated when a 1-bit ECC error occurs */
#define FLASH_ECC_EER_2BITIE_Pos        (1)
#define FLASH_ECC_EER_2BITIE_Msk        (0x01U << FLASH_ECC_EER_2BITIE_Pos) /*!< An event request is generated when a 2-bit ECC error occurs */

/**
  * @brief FLASH_ECC_SR Register Bit Definition
  */
#define FLASH_ECC_SR_1BITF_Pos          (0)
#define FLASH_ECC_SR_1BITF_Msk          (0x01U << FLASH_ECC_SR_1BITF_Pos) /*!< Set when a 1-bit ECC error occurs */
#define FLASH_ECC_SR_2BITF_Pos          (1)
#define FLASH_ECC_SR_2BITF_Msk          (0x01U << FLASH_ECC_SR_2BITF_Pos) /*!< Set when a 2-bit ECC error occurs */

/**
  * @brief FLASH_ECC_ADDRR Register Bit Definition
  */
#define FLASH_ECC_ADDRR_ADDR_Pos        (0)
#define FLASH_ECC_ADDRR_ADDR_Msk        (0xFFFFFFFFU << FLASH_ECC_ADDRR_ADDR_Pos) /*!< ECC error data address generated */

/**
  * @brief FLASH_ECC_SYNR Register Bit Definition
  */
#define FLASH_ECC_SYNR_SYN_Pos          (0)
#define FLASH_ECC_SYNR_SYN_Msk          (0xFFU << FLASH_ECC_SYNR_SYN_Pos) /*!< An ECC error checkmark was generated */

/**
  * @brief FLASH_ECC_DLR Register Bit Definition
  */
#define FLASH_ECC_DLR_DATAL_Pos         (0)
#define FLASH_ECC_DLR_DATAL_Msk         (0xFFFFFFFFU << FLASH_ECC_DLR_DATAL_Pos) /*!< Generate ECC errors in the raw data LSB portion */

/**
  * @brief FLASH_ECC_DHR Register Bit Definition
  */
#define FLASH_ECC_DHR_DATAH_Pos         (0)
#define FLASH_ECC_DHR_DATAH_Msk         (0xFFFFFFFFU << FLASH_ECC_DHR_DATAH_Pos) /*!< Generate ECC errors in the raw data MSB portion */

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

/** --------------------------------------------------------------------------*/
#endif
/** --------------------------------------------------------------------------*/

