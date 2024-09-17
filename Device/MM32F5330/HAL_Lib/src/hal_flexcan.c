/***********************************************************************************************************************
    @file     hal_flexcan.c
    @author   VV TEAM
    @brief    THIS FILE PROVIDES ALL THE FLEXCAN FIRMWARE FUNCTIONS.
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

#define __HAL_FLEXCAN_C

/* Files includes ----------------------------------------------------------------------------------------------------*/
#include "hal_flexcan.h"
#include "string.h"

/** @addtogroup MM32_StdPeriph_Driver
  * @{
  */

/** @addtogroup FLEXCAN
  * @{
  */

/** @defgroup FLEXCAN_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup FLEXCAN_Private_Functions
  * @{
  */

/* Typedef for interrupt handler. */
typedef void (*flexcan_isr_t)(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle);

/**
  * @brief  Gets the FlexCAN module interrupt flags.
  *
  * This function gets all FlexCAN status flags. The flags are returned as the logical
  * OR value of the enumerators @ref _flexcan_flags. To check the specific status,
  * compare the return value with enumerators in @ref _flexcan_flags.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @return FlexCAN status flags which are ORed by the enumerators in the _flexcan_flags.
  */
static uint32_t FLEXCAN_GetStatusFlags(FLEXCAN_TypeDef *flex_can)
{
    return (flex_can->ESR1);
}

#if (defined (FLEXCAN_HAS_ERRATA_9595) && FLEXCAN_HAS_ERRATA_9595)
void FLEXCAN_EnterFreezeMode(FLEXCAN_TypeDef *flex_can)
{
    uint32_t uint32_tTimeoutCount = 0U;
    uint32_t uint32_tTempMCR      = 0U;
    uint32_t uint32_tTempIMASK1   = 0U;

#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint32_t uint32_tTempIMASK2 = 0U;
#endif

     /* Step1: set FRZ enable in MCR. */
    flex_can->MCR |= CAN_MCR_FRZ_MASK;

     /* Step2: to check if MDIS bit set in MCR. if yes, clear it. */
    if (0U != (flex_can->MCR & CAN_MCR_MDIS_MASK))
    {
        flex_can->MCR &= ~CAN_MCR_MDIS_MASK;
    }

     /* Step3: polling LPMACK. */
    uint32_tTimeoutCount = (uint32_t)FLEXCAN_WAIT_TIMEOUT;

    while ((0U == (flex_can->MCR & CAN_MCR_LPMACK_MASK)) && (uint32_tTimeoutCount > 0U))
    {
        uint32_tTimeoutCount--;
    }

     /* Step4: to check FLTCONF in ESR1 register */
    if (0U == (flex_can->ESR1 & CAN_ESR1_FLTCONF_BUSOFF))
    {
         /* Step5B: Set Halt bits. */
        flex_can->MCR |= CAN_MCR_HALT_MASK;

         /* Step6B: Poll the MCR register until the Freeze Acknowledge (FRZACK) bit is set, timeout need more than 178 */
         /* CAN bit length, so 20 multiply timeout is enough. */
        uint32_tTimeoutCount = (uint32_t)FLEXCAN_WAIT_TIMEOUT * 20U;

        while ((0U == (flex_can->MCR & CAN_MCR_FRZACK_MASK)) && (uint32_tTimeoutCount > 0U))
        {
            uint32_tTimeoutCount--;
        }
    }
    else
    {
         /* backup MCR and IMASK register. Errata document not descript it, but we need backup for step 8A and 9A. */
        uint32_tTempMCR    = flex_can->MCR;
        uint32_tTempIMASK1 = flex_can->IMASK1;
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        uint32_tTempIMASK2 = flex_can->IMASK2;
#endif

         /* Step5A: Set the Soft Reset bit ((SOFTRST) in the MCR. */
        flex_can->MCR |= CAN_MCR_SOFTRST_MASK;

         /* Step6A: Poll the MCR register until the Soft Reset (SOFTRST) bit is cleared. */
        uint32_tTimeoutCount = (uint32_t)FLEXCAN_WAIT_TIMEOUT;

        while ((CAN_MCR_SOFTRST_MASK == (flex_can->MCR & CAN_MCR_SOFTRST_MASK)) && (uint32_tTimeoutCount > 0U))
        {
            uint32_tTimeoutCount--;
        }

         /* Step7A: Poll the MCR register until the Freeze Acknowledge (FRZACK) bit is set. */
        uint32_tTimeoutCount = (uint32_t)FLEXCAN_WAIT_TIMEOUT;

        while ((0U == (flex_can->MCR & CAN_MCR_FRZACK_MASK)) && (uint32_tTimeoutCount > 0U))
        {
            uint32_tTimeoutCount--;
        }

         /* Step8A: reconfig MCR. */
        flex_can->MCR = uint32_tTempMCR;

         /* Step9A: reconfig IMASK. */
        flex_can->IMASK1 = uint32_tTempIMASK1;
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        flex_can->IMASK2 = uint32_tTempIMASK2;
#endif
    }
}

#elif (defined (FLEXCAN_HAS_ERRATA_8341) && FLEXCAN_HAS_ERRATA_8341)
void FLEXCAN_EnterFreezeMode(FLEXCAN_TypeDef *flex_can)
{
    uint32_t uint32_tTimeoutCount = 0U;
    uint32_t uint32_tTempMCR      = 0U;
    uint32_t uint32_tTempIMASK1   = 0U;

#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint32_t uint32_tTempIMASK2   = 0U;
#endif

     /* Step1: set FRZ and HALT bit enable in MCR. */
    flex_can->MCR |= CAN_MCR_FRZ_MASK;
    flex_can->MCR |= CAN_MCR_HALT_MASK;

     /* Step2: to check if MDIS bit set in MCR. if yes, clear it. */
    if (0U != (flex_can->MCR & CAN_MCR_MDIS_MASK))
    {
        flex_can->MCR &= ~CAN_MCR_MDIS_MASK;
    }

     /* Step3: Poll the MCR register until the Freeze Acknowledge (FRZACK) bit is set. */
    uint32_tTimeoutCount = (uint32_t)FLEXCAN_WAIT_TIMEOUT * 100U;

    while ((0U == (flex_can->MCR & CAN_MCR_FRZACK_MASK)) && (uint32_tTimeoutCount > 0U))
    {
        uint32_tTimeoutCount--;
    }

     /* Step4: check whether the timeout reached. if no skip step5 to step8. */
    if (0U == uint32_tTimeoutCount)
    {
         /* backup MCR and IMASK register. Errata document not descript it, but we need backup for step 8A and 9A. */
        uint32_tTempMCR    = flex_can->MCR;
        uint32_tTempIMASK1 = flex_can->IMASK1;
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        uint32_tTempIMASK2 = flex_can->IMASK2;
#endif
         /* Step5: Set the Soft Reset bit ((SOFTRST) in the MCR. */
        flex_can->MCR |= CAN_MCR_SOFTRST_MASK;

         /* Step6: Poll the MCR register until the Soft Reset (SOFTRST) bit is cleared. */
        while (CAN_MCR_SOFTRST_MASK == (flex_can->MCR & CAN_MCR_SOFTRST_MASK))
        {
        }

         /* Step7: reconfig MCR. */
        flex_can->MCR = uint32_tTempMCR;

         /* Step8: reconfig IMASK. */
        flex_can->IMASK1 = uint32_tTempIMASK1;
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        flex_can->IMASK2 = uint32_tTempIMASK2;
#endif
    }
}

#else
void FLEXCAN_EnterFreezeMode(FLEXCAN_TypeDef *flex_can)
{
     /* Set Freeze, Halt bits. */
    flex_can->MCR |= CAN_MCR_FRZ_MASK;
    flex_can->MCR |= CAN_MCR_HALT_MASK;

    while (0U == (flex_can->MCR & CAN_MCR_FRZACK_MASK))
    {
    }
}

#endif

void FLEXCAN_ExitFreezeMode(FLEXCAN_TypeDef *flex_can)
{
     /* Clear Freeze, Halt bits. */
    flex_can->MCR &= ~CAN_MCR_HALT_MASK;
    flex_can->MCR &= ~CAN_MCR_FRZ_MASK;

     /* Wait until the FlexCAN Module exit freeze mode. */
    while (0U != (flex_can->MCR & CAN_MCR_FRZACK_MASK))
    {
    }
}

#if !defined (NDEBUG)
bool FLEXCAN_IsMbOccupied(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx)
{
    uint8_t lastOccupiedMb;
    bool fgRet;

     /* Is Rx FIFO enabled? */
    if (0U != (flex_can->MCR & CAN_MCR_RFEN_MASK))
    {
         /* Get RFFN value. */
        lastOccupiedMb = (uint8_t)((flex_can->CTRL2 & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
         /* Calculate the number of last Message Buffer occupied by Rx FIFO. */
        lastOccupiedMb = ((lastOccupiedMb + 1U) * 2U) + 5U;

#if ((defined (FLEXCAN_HAS_ERRATA_5641) && FLEXCAN_HAS_ERRATA_5641) || \
        (defined (FLEXCAN_HAS_ERRATA_5829) && FLEXCAN_HAS_ERRATA_5829))
         /* the first valid MB should be occupied by ERRATA 5461 or 5829. */
        lastOccupiedMb += 1U;
#endif
        fgRet = (mbIdx <= lastOccupiedMb);
    }
    else
    {
#if ((defined (FLEXCAN_HAS_ERRATA_5641) && FLEXCAN_HAS_ERRATA_5641) || \
        (defined (FLEXCAN_HAS_ERRATA_5829) && FLEXCAN_HAS_ERRATA_5829))
        if (0U == mbIdx)
        {
            fgRet = true;
        }
        else
#endif
        {
            fgRet = false;
        }
    }

    return (fgRet);
}

#endif

#if ((defined (FLEXCAN_HAS_ERRATA_5641) && FLEXCAN_HAS_ERRATA_5641) || \
    (defined (FLEXCAN_HAS_ERRATA_5829) && FLEXCAN_HAS_ERRATA_5829))
static uint8_t FLEXCAN_GetFirstValidMb(FLEXCAN_TypeDef *flex_can)
{
    uint8_t firstValidMbNum;

    if (0U != (flex_can->MCR & CAN_MCR_RFEN_MASK))
    {
        firstValidMbNum = (uint8_t)((flex_can->CTRL2 & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        firstValidMbNum = ((firstValidMbNum + 1U) * 2U) + 6U;
    }
    else
    {
        firstValidMbNum = 0U;
    }

    return (firstValidMbNum);
}

#endif

static ErrorStatus FLEXCAN_IsMbIntEnabled(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx)
{
    uint32_t flag = 1U;
    ErrorStatus fgRet    = ERROR;

#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    if (mbIdx >= 32U)
    {
        fgRet = (0U != (flex_can->IMASK2 & (flag << (mbIdx - 32U))));
    }
    else
#endif
    {
        fgRet = (0U != (flex_can->IMASK1 & (flag << mbIdx)));
    }

    return (fgRet);
}

static void FLEXCAN_Reset(FLEXCAN_TypeDef *flex_can)
{
    /* The module must should be first exit from low power */
    /* mode, and then soft reset can be applied. */

    uint8_t i;

#if (defined (FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    if (0 != (FLEXCAN_INSTANCE_HAS_DOZE_MODE_SUPPORTn(flex_can)))
    {
         /* De-/assert DOZE Enable Bit. */
        flex_can->MCR &= ~CAN_MCR_DOZE_MASK;
    }
#endif

     /* Wait until FlexCAN exit from any Low Power Mode. */
    while (0U != (flex_can->MCR & CAN_MCR_LPMACK_MASK))
    {
    }

     /* assert Soft Reset Signal. */
    flex_can->MCR |= CAN_MCR_SOFTRST_MASK;

     /* Wait until FlexCAN reset completes. */
    while (0U != (flex_can->MCR & CAN_MCR_SOFTRST_MASK))
    {
    }

     /* Reset MCR register. */
#if (defined (FLEXCAN_HAS_GLITCH_FILTER) && FLEXCAN_HAS_GLITCH_FILTER)
    flex_can->MCR |= CAN_MCR_WRNEN_MASK | CAN_MCR_WAKSRC_MASK |
                     CAN_MCR_MAXMB((uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can) - 1U);
#else
    flex_can->MCR |=
        CAN_MCR_WRNEN_MASK | CAN_MCR_MAXMB((uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can) - 1U);
#endif

    /* Reset CTRL1 and CTRL2 register. */

    flex_can->CTRL1 = CAN_CTRL1_SMP_MASK;
    flex_can->CTRL2 = CAN_CTRL2_TASD(0x16) | CAN_CTRL2_RRS_MASK | CAN_CTRL2_EACEN_MASK;

     /* Clean all individual Rx Mask of Message Buffers. */
    for (i = 0; i < (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can); i++)
    {
        flex_can->RXIMR[i] = 0x3FFFFFFF;
    }

     /* Clean Global Mask of Message Buffers. */
    flex_can->RXMGMASK = 0x3FFFFFFF;
     /* Clean Global Mask of Message Buffer 14. */
    flex_can->RX14MASK = 0x3FFFFFFF;
     /* Clean Global Mask of Message Buffer 15. */
    flex_can->RX15MASK = 0x3FFFFFFF;
     /* Clean Global Mask of Rx FIFO. */
    flex_can->RXFGMASK = 0x3FFFFFFF;

     /* Clean all Message Buffer CS fields. */
    for (i = 0; i < (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can); i++)
    {
        flex_can->MB[i].CS = 0x0;
    }
}

/* Type definitions */
/* brief Structure type for grouping CAN bus timing related information. */
typedef struct t_can_bus_timing
{
    uint8_t timeQuanta;                /* Total number of time quanta */
    uint8_t propSeg;                   /* CAN propagation segment */
    uint8_t phaseSeg1;                 /* CAN phase segment 1 */
    uint8_t phaseSeg2;                 /* CAN phase segment 2 */
} tCanBusTiming;

/**
  * @brief Sets the FlexCAN protocol timing characteristic.
  *
  * This function gives user settings to CAN bus timing characteristic.
  * The function is for an experienced user. For less experienced users, call
  * the FLEXCAN_Init() and fill the baud rate field with a desired value.
  * This provides the default timing characteristics to the module.
  *
  * Note that calling FLEXCAN_SetTimingConfig() overrides the baud rate set
  * in FLEXCAN_Init().
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param pConfig Pointer to the timing configuration structure.
  */
void FLEXCAN_SetTimingConfig(FLEXCAN_TypeDef *flex_can, const flexcan_timing_config_t *pConfig)
{
     /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

     /* Cleaning previous Timing Setting. */
    flex_can->CTRL1 &= ~(CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_RJW_MASK | CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
                         CAN_CTRL1_PROPSEG_MASK);

     /* Updating Timing Setting according to configuration structure. */
    flex_can->CTRL1 |= (CAN_CTRL1_PRESDIV(pConfig->preDivider) | CAN_CTRL1_RJW(pConfig->rJumpwidth) |
                        CAN_CTRL1_PSEG1(pConfig->phaseSeg1) | CAN_CTRL1_PSEG2(pConfig->phaseSeg2) |
                        CAN_CTRL1_PROPSEG(pConfig->propSeg));

     /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

/**
  * brief Sets the FlexCAN FD protocol timing characteristic.
  *
  * This function gives user settings to CAN bus timing characteristic.
  * The function is for an experienced user. For less experienced users, call
  * the FLEXCAN_FDInit() and fill the baud rate field with a desired value.
  * This provides the default timing characteristics to the module.
  *
  * Note that calling FLEXCAN_SetFDTimingConfig() overrides the baud rate set
  * in FLEXCAN_FDInit().
  *
  * param flex_can FlexCAN peripheral base address.
  * param pConfig Pointer to the timing configuration structure.
  */
void FLEXCAN_SetFDTimingConfig(FLEXCAN_TypeDef *flex_can, const flexcan_timing_config_t *pConfig)
{
     /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

    flex_can->CBT |= CAN_CBT_BTF(1);

     /* Cleaning previous Timing Setting. */
    flex_can->CBT &= ~(CAN_CBT_EPRESDIV_MASK | CAN_CBT_ERJW_MASK | CAN_CBT_EPSEG1_MASK | CAN_CBT_EPSEG2_MASK |
                       CAN_CBT_EPROPSEG_MASK);

     /* Updating Timing Setting according to configuration structure. */
    flex_can->CBT |= (CAN_CBT_EPRESDIV(pConfig->preDivider) | CAN_CBT_ERJW(pConfig->rJumpwidth) |
                      CAN_CBT_EPSEG1(pConfig->phaseSeg1) | CAN_CBT_EPSEG2(pConfig->phaseSeg2) |
                      CAN_CBT_EPROPSEG(pConfig->propSeg));

     /* Cleaning previous Timing Setting. */
    flex_can->FDCBT &= ~(CAN_FDCBT_FPRESDIV_MASK | CAN_FDCBT_FRJW_MASK | CAN_FDCBT_FPSEG1_MASK | CAN_FDCBT_FPSEG2_MASK |
                         CAN_FDCBT_FPROPSEG_MASK);

     /* Updating Timing Setting according to configuration structure. */
    flex_can->FDCBT |= (CAN_FDCBT_FPRESDIV(pConfig->fpreDivider) | CAN_FDCBT_FRJW(pConfig->frJumpwidth) |
                        CAN_FDCBT_FPSEG1(pConfig->fphaseSeg1) | CAN_FDCBT_FPSEG2(pConfig->fphaseSeg2) |
                        CAN_FDCBT_FPROPSEG(pConfig->fpropSeg));

     /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

void FLEXCAN_SetBaudRate(FLEXCAN_TypeDef *flex_can, flexcan_timing_config_t timingConfig)
{
     /* Update actual timing characteristic. */
    FLEXCAN_SetTimingConfig(flex_can, (const flexcan_timing_config_t *)(uint32_t)&timingConfig);
}

void FLEXCAN_SetFDBaudRate(FLEXCAN_TypeDef *flex_can, flexcan_timing_config_t timingConfig)
{
     /* Update actual timing characteristic. */
    FLEXCAN_SetFDTimingConfig(flex_can, (const flexcan_timing_config_t *)(uint32_t)&timingConfig);
}

/**
  * @brief Initializes a FlexCAN instance.
  *
  * This function initializes the FlexCAN module with user-defined settings.
  * This example shows how to set up the flexcan_config_t parameters and how
  * to call the FLEXCAN_Init function by passing in these parameters.
  *  code
  *   flexcan_config_t flexcanConfig;
  *   flexcanConfig.clkSrc               = Enum_Flexcan_ClkSrc0;
  *   flexcanConfig.baudRate             = 1000000U;
  *   flexcanConfig.maxMbNum             = 16;
  *   flexcanConfig.enableLoopBack       = false;
  *   flexcanConfig.enableSelfWakeup     = false;
  *   flexcanConfig.enableIndividMask    = false;
  *   flexcanConfig.disableSelfReception = false;
  *   flexcanConfig.enableListenOnlyMode = false;
  *   flexcanConfig.enableDoze           = false;
  *   flexcanConfig.timingConfig         = timingConfig;
  *   FLEXCAN_Init(CAN0, &flexcanConfig, 8000000UL);
  *   endcode
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param pConfig Pointer to the user-defined configuration structure.
  */
void FLEXCAN_Init(FLEXCAN_TypeDef *flex_can, const flexcan_config_t *pConfig)
{
    uint32_t mcrTemp;
    uint32_t ctrl1Temp;

#if defined (CAN_CTRL1_CLKSRC_MASK)
    {
         /* Disable FlexCAN Module. */
        FLEXCAN_Cmd(flex_can, DISABLE);
        /* Protocol-Engine clock source selection, This bit must be set */
        /* when FlexCAN Module in Disable Mode. */

        flex_can->CTRL1 = (Enum_Flexcan_ClkSrc0 == pConfig->clkSrc) ? (flex_can->CTRL1 & ~CAN_CTRL1_CLKSRC_MASK) :
                          (flex_can->CTRL1 | CAN_CTRL1_CLKSRC_MASK);
    }
#endif /* CAN_CTRL1_CLKSRC_MASK */

     /* Enable FlexCAN Module for configuration. */
    FLEXCAN_Cmd(flex_can, ENABLE);

     /* Reset to known status. */
    FLEXCAN_Reset(flex_can);

     /* Save current CTRL1 value and enable to enter Freeze mode(enabled by default). */
    ctrl1Temp = flex_can->CTRL1;

     /* Save current MCR value and enable to enter Freeze mode(enabled by default). */
    mcrTemp = flex_can->MCR;

     /* Enable Loop Back Mode? */
    ctrl1Temp = (pConfig->enableLoopBack) ? (ctrl1Temp | CAN_CTRL1_LPB_MASK) : (ctrl1Temp & ~CAN_CTRL1_LPB_MASK);

     /* Enable Timer Sync? */
    ctrl1Temp = (pConfig->enableTimerSync) ? (ctrl1Temp | CAN_CTRL1_TSYN_MASK) : (ctrl1Temp & ~CAN_CTRL1_TSYN_MASK);

     /* Enable Listen Only Mode? */
    ctrl1Temp = (pConfig->enableListenOnlyMode) ? ctrl1Temp | CAN_CTRL1_LOM_MASK : ctrl1Temp & ~CAN_CTRL1_LOM_MASK;

     /* Set the maximum number of Message Buffers */
    mcrTemp = (mcrTemp & ~CAN_MCR_MAXMB_MASK) | CAN_MCR_MAXMB((uint32_t)pConfig->maxMbNum - 1U);

     /* Enable Self Wake Up Mode and configure the wake up source. */
    mcrTemp = (pConfig->enableSelfWakeup) ? (mcrTemp | CAN_MCR_SLFWAK_MASK) : (mcrTemp & ~CAN_MCR_SLFWAK_MASK);
    mcrTemp = (Enum_Flexcan_WakeupSrcFiltered == pConfig->wakeupSrc) ? (mcrTemp | CAN_MCR_WAKSRC_MASK) :
              (mcrTemp & ~CAN_MCR_WAKSRC_MASK);

     /* Enable Individual Rx Masking? */
    mcrTemp = (pConfig->enableIndividMask) ? (mcrTemp | CAN_MCR_IRMQ_MASK) : (mcrTemp & ~CAN_MCR_IRMQ_MASK);

     /* Disable Self Reception? */
    mcrTemp = (pConfig->disableSelfReception) ? mcrTemp | CAN_MCR_SRXDIS_MASK : mcrTemp & ~CAN_MCR_SRXDIS_MASK;

#if (defined (FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    if (0 != FLEXCAN_INSTANCE_HAS_DOZE_MODE_SUPPORTn(flex_can))
    {
         /* Enable Doze Mode? */
        mcrTemp = (pConfig->enableDoze) ? (mcrTemp | CAN_MCR_DOZE_MASK) : (mcrTemp & ~CAN_MCR_DOZE_MASK);
    }
#endif

     /* Write back CTRL1 Configuration to register. */
    flex_can->CTRL1 = ctrl1Temp;

#if (defined (FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
     /* Enable to update in MCER. */
    flex_can->CTRL2 |= CAN_CTRL2_ECRWRE_MASK;
    flex_can->MECR  &= ~CAN_MECR_ECRWRDIS_MASK;
#endif

     /* Write back MCR Configuration to register. */
    flex_can->MCR = mcrTemp;

     /* Baud Rate Configuration. */
    FLEXCAN_SetBaudRate(flex_can, pConfig->timingConfig);
}

/**
  * @brief Initializes a FlexCAN FD instance.
  *
  * This function initializes the FlexCAN module with user-defined settings.
  * This example shows how to set up the flexcan_config_t parameters and how
  * to call the FLEXCAN_FDInit function by passing in these parameters.
  *  code
  *   flexcan_config_t flexcanConfig;
  *   flexcanConfig.clkSrc               = kFLEXCAN_ClkSrc0;
  *   flexcanConfig.baudRate             = 1000000U;
  *   flexcanConfig.baudRateFD           = 2000000U;
  *   flexcanConfig.maxMbNum             = 16;
  *   flexcanConfig.enableLoopBack       = false;
  *   flexcanConfig.enableSelfWakeup     = false;
  *   flexcanConfig.enableIndividMask    = false;
  *   flexcanConfig.disableSelfReception = false;
  *   flexcanConfig.enableListenOnlyMode = false;
  *   flexcanConfig.enableDoze           = false;
  *   flexcanConfig.timingConfig         = timingConfig;
  *   FLEXCAN_FDInit(CAN0, &flexcanConfig, 8000000UL, kFLEXCAN_16BperMB, false);
  *   endcode
  *
  * @param flex_can FlexCAN peripheral base address.
  * @param pConfig Pointer to the user-defined configuration structure.
  * @param dataSize FlexCAN FD frame payload size.
  * @param brs If bitrate switch is enabled in FD mode.
  */
void FLEXCAN_FDInit(FLEXCAN_TypeDef *flex_can, const flexcan_config_t *pConfig, flexcan_mb_size_t dataSize, FunctionalState brs)
{
    uint32_t fdctrl = 0U;

     /* Extra bitrate setting for CANFD. */
    FLEXCAN_SetFDBaudRate(flex_can, pConfig->timingConfig);

     /* read FDCTRL register. */
    fdctrl = flex_can->FDCTRL;

     /* Enable FD operation and set bitrate switch. */
    if (brs)
    {
        fdctrl |= CAN_FDCTRL_FDRATE_MASK;
    }
    else
    {
        fdctrl &= ~CAN_FDCTRL_FDRATE_MASK;
    }

    if (brs && !(pConfig->enableLoopBack))
    {
         /* The TDC offset should be configured as shown in this equation : offset = PSEG1 + PROPSEG + 2 */
        if (((uint32_t)pConfig->timingConfig.fphaseSeg1 + pConfig->timingConfig.fpropSeg + 2U) < MAX_TDCOFF)
        {
            fdctrl = (fdctrl & ~CAN_FDCTRL_TDCOFF_MASK) | CAN_FDCTRL_TDCOFF((uint32_t)pConfig->timingConfig.fphaseSeg1 +
                                                                            pConfig->timingConfig.fpropSeg + 2U);
        }
        else
        {
            fdctrl = (fdctrl & ~CAN_FDCTRL_TDCOFF_MASK) | CAN_FDCTRL_TDCOFF(MAX_TDCOFF);
        }

         /* Enable the Transceiver Delay Compensation */
        fdctrl = (fdctrl & ~CAN_FDCTRL_TDCEN_MASK) | CAN_FDCTRL_TDCEN(1);
    }

     /* Before use "|=" operation for multi-bits field, CPU should clean previous Setting. */
    fdctrl = (fdctrl & ~CAN_FDCTRL_MBDSR0_MASK) | CAN_FDCTRL_MBDSR0(dataSize);

     /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

    flex_can->MCR |= CAN_MCR_FDEN_MASK;

     /* update the FDCTL register. */
    flex_can->FDCTRL = fdctrl;

     /* Enable CAN FD ISO mode by default. */
    flex_can->CTRL2 |= CAN_CTRL2_ISOCANFDEN_MASK;

     /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

/**
  * @brief De-initializes a FlexCAN instance.
  *
  * This function disables the FlexCAN module clock and sets all register values
  * to the reset value.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  */
void FLEXCAN_Deinit(FLEXCAN_TypeDef *flex_can)
{
     /* Reset all Register Contents. */
    FLEXCAN_Reset(flex_can);

     /* Disable FlexCAN module. */
    FLEXCAN_Cmd(flex_can, DISABLE);
}

/**
  * @brief Gets the default configuration structure.
  *
  * This function initializes the FlexCAN configuration structure to default values. The default
  * values are as follows.
  *   flexcanConfig->clkSrc                  = Enum_Flexcan_ClkSrc0;
  *   flexcanConfig->baudRate                = 1000000U;
  *   flexcanConfig->baudRateFD              = 2000000U;
  *   flexcanConfig->maxMbNum                = 16;
  *   flexcanConfig->enableLoopBack          = false;
  *   flexcanConfig->enableSelfWakeup        = false;
  *   flexcanConfig->enableIndividMask       = false;
  *   flexcanConfig->disableSelfReception    = false;
  *   flexcanConfig->enableListenOnlyMode    = false;
  *   flexcanConfig->enableDoze              = false;
  *   flexcanConfig.timingConfig             = timingConfig;
  *
  * @param pConfig Pointer to the FlexCAN configuration structure.
  */
void FLEXCAN_GetDefaultConfig(flexcan_config_t *pConfig)
{
     /* Initializes the configure structure to zero. */
    (void)memset(pConfig, 0, sizeof(*pConfig));

     /* Initialize FlexCAN Module config struct with default value. */
    pConfig->clkSrc   = Enum_Flexcan_ClkSrc0;
    pConfig->baudRate = 1000000U;      /* FlexCAN baud rate. */

    pConfig->baudRateFD = 2000000U;    /* FlexCAN FD baud rate. */

    pConfig->maxMbNum             = 32;
    pConfig->enableLoopBack       = false;
    pConfig->enableTimerSync      = true;
    pConfig->enableSelfWakeup     = false;
    pConfig->wakeupSrc            = Enum_Flexcan_WakeupSrcFiltered;
    pConfig->enableIndividMask    = true;
    pConfig->disableSelfReception = false;
    pConfig->enableListenOnlyMode = false;
#if (defined (FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    pConfig->enableDoze = false;
#endif
     /* Default protocol timing configuration, time quantum is 10. */
    pConfig->timingConfig.phaseSeg1  = 3;
    pConfig->timingConfig.phaseSeg2  = 2;
    pConfig->timingConfig.propSeg    = 1;
    pConfig->timingConfig.rJumpwidth = 1;

    pConfig->timingConfig.fphaseSeg1  = 3;
    pConfig->timingConfig.fphaseSeg2  = 3;
    pConfig->timingConfig.fpropSeg    = 1;
    pConfig->timingConfig.frJumpwidth = 1;
}

/**
  * @brief Sets the FlexCAN receive message buffer global mask.
  *
  * This function sets the global mask for the FlexCAN message buffer in a matching process.
  * The configuration is only effective when the Rx individual mask is disabled in the FLEXCAN_Init().
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mask Rx Message Buffer Global Mask value.
  */
void FLEXCAN_SetRxMbGlobalMask(FLEXCAN_TypeDef *flex_can, uint32_t mask)
{
     /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

     /* Setting Rx Message Buffer Global Mask value. */
    flex_can->RXMGMASK = mask;
    flex_can->RX14MASK = mask;
    flex_can->RX15MASK = mask;

     /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

/**
  * @brief Sets the FlexCAN receive FIFO global mask.
  *
  * This function sets the global mask for FlexCAN FIFO in a matching process.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mask Rx Fifo Global Mask value.
  */
void FLEXCAN_SetRxFifoGlobalMask(FLEXCAN_TypeDef *flex_can, uint32_t mask)
{
     /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

     /* Setting Rx FIFO Global Mask value. */
    flex_can->RXFGMASK = mask;

     /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

/**
  * @brief Sets the FlexCAN receive individual mask.
  *
  * This function sets the individual mask for the FlexCAN matching process.
  * The configuration is only effective when the Rx individual mask is enabled in the FLEXCAN_Init().
  * If the Rx FIFO is disabled, the individual mask is applied to the corresponding Message Buffer.
  * If the Rx FIFO is enabled, the individual mask for Rx FIFO occupied Message Buffer is applied to
  * the Rx Filter with the same index. Note that only the first 32
  * individual masks can be used as the Rx FIFO filter mask.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param maskIdx The Index of individual Mask.
  * @param mask Rx Individual Mask value.
  */
void FLEXCAN_SetRxIndividualMask(FLEXCAN_TypeDef *flex_can, uint8_t maskIdx, uint32_t mask)
{
     /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

     /* Setting Rx Individual Mask value. */
    flex_can->RXIMR[maskIdx] = mask;

     /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

/**
  * @brief Configures a FlexCAN transmit message buffer.
  *
  * This function aborts the previous transmission, cleans the Message Buffer, and
  * configures it as a Transmit Message Buffer.
  *
  * @param flexcan: FlexCAN peripheral Struct Point.
  * @param mbIdx: The Message Buffer index.
  * @param state:  This parameter can be: ENABLE or DISABLE.
  */
void FLEXCAN_TxMbConfig(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, FunctionalState state)
{
     /* Inactivate Message Buffer. */
    if (state)
    {
        flex_can->MB[mbIdx].CS = CAN_CS_CODE(Enum_Flexcan_TxMbInactive);
    }
    else
    {
        flex_can->MB[mbIdx].CS = 0;
    }

     /* Clean Message Buffer content. */
    flex_can->MB[mbIdx].ID    = 0x0;
    flex_can->MB[mbIdx].WORD0 = 0x0;
    flex_can->MB[mbIdx].WORD1 = 0x0;
}

/**
  * @brief Calculates the segment values for a single bit time for classical CAN
  *
  * @param baudRate The data speed in bps
  * @param tqNum Number of time quantas per bit
  * @param pTimingConfig Pointer to the FlexCAN timing configuration structure.
  *
  * @return SUCCESS if Calculates the segment success, ERROR if Calculates the segment success
  */
static ErrorStatus FLEXCAN_GetSegments(uint32_t baudRate, uint32_t tqNum, flexcan_timing_config_t *pTimingConfig)
{
    uint32_t ideal_sp;
    uint32_t p1;
    ErrorStatus fgRet = ERROR;

     /* Get ideal sample point. For the Bit field in CTRL1 register can't calculate higher ideal SP, we set it as the */
     /* lowest one(75%). */
    ideal_sp = IDEAL_SP_LOW;

     /* distribute time quanta. */
    p1 = tqNum * (uint32_t)ideal_sp;
    pTimingConfig->propSeg = (uint8_t)(p1 / (uint32_t)IDEAL_SP_FACTOR - 3U);

    if (pTimingConfig->propSeg <= (MAX_PSEG1 + MAX_PROPSEG))
    {
        if (pTimingConfig->propSeg > MAX_PROPSEG)
        {
            pTimingConfig->phaseSeg1 = pTimingConfig->propSeg - MAX_PROPSEG;
            pTimingConfig->propSeg   = MAX_PROPSEG;
        }
        else
        {
            pTimingConfig->phaseSeg1 = 0;
        }

        if (pTimingConfig->phaseSeg1 <= MAX_PSEG1)
        {
             /* The value of prog Seg should be not larger than tqNum -4U. */
            if ((pTimingConfig->propSeg + pTimingConfig->phaseSeg1) < ((uint8_t)tqNum - 4U))
            {
                pTimingConfig->phaseSeg2 = (uint8_t)tqNum - (pTimingConfig->phaseSeg1 + pTimingConfig->propSeg + 4U);

                if (pTimingConfig->phaseSeg2 <= MAX_PSEG2)
                {
                    if ((pTimingConfig->phaseSeg1 < pTimingConfig->phaseSeg2) && (pTimingConfig->propSeg > (pTimingConfig->phaseSeg2 - pTimingConfig->phaseSeg1)))
                    {
                        pTimingConfig->propSeg  -= (pTimingConfig->phaseSeg2 - pTimingConfig->phaseSeg1);
                        pTimingConfig->phaseSeg1 = pTimingConfig->phaseSeg2;
                    }

                     /* subtract one TQ for sync seg. */
                     /* sjw is 20% of total TQ, rounded to nearest int. */
                    pTimingConfig->rJumpwidth = ((uint8_t)tqNum + 4U) / 5U - 1U;

                     /* The max tqNum for CBT will reach to 129, ERJW would not be larger than 26. */
                     /* Considering that max ERJW is 31, rJumpwidth will always be smaller than MAX_ERJW. */
                    if (pTimingConfig->rJumpwidth > MAX_RJW)
                    {
                        pTimingConfig->rJumpwidth = MAX_RJW;
                    }

                    fgRet = SUCCESS;
                }
            }
        }
    }

    return (fgRet);
}

/**
  * @brief Calculates the improved timing values by specific baudrates for classical CAN
  *
  * @param baudRate  The classical CAN speed in bps defined by user
  * @param sourceClock_Hz The Source clock data speed in bps. Zero to disable baudrate switching
  * @param pTimingConfig Pointer to the FlexCAN timing configuration structure.
  *
  * @return SUCCESS if timing configuration found, ERROR if failed to find configuration
  */
ErrorStatus FLEXCAN_CalculateImprovedTimingValues(uint32_t baudRate, uint32_t sourceClock_Hz, flexcan_timing_config_t *pTimingConfig)
{
    uint32_t clk;                      /* the clock is tqNumb x baudRateFD. */
    uint32_t tqNum;                    /* Numbers of TQ. */
    ErrorStatus fgRet = ERROR;

     /*  Auto Improved Protocal timing for CTRL1. */
    tqNum = CTRL1_MAX_TIME_QUANTA;

    do
    {
        clk = baudRate * tqNum;

        if (clk > sourceClock_Hz)
        {
            continue;                  /* tqNum too large, clk has been exceed sourceClock_Hz. */
        }

        if ((sourceClock_Hz / clk * clk) != sourceClock_Hz)
        {
            continue;                  /* Non-supporting: the frequency of clock source is not divisible by target baud rate, the user */
            /* should change a divisible baud rate. */
        }

        pTimingConfig->preDivider = (uint16_t)(sourceClock_Hz / clk) - 1U;

        if (pTimingConfig->preDivider > MAX_PRESDIV)
        {
            break;                     /* The frequency of source clock is too large or the baud rate is too small, the pre-divider could */
            /* not handle it. */
        }

         /* Try to get the best timing configuration. */
        if (FLEXCAN_GetSegments(baudRate, tqNum, pTimingConfig))
        {
            fgRet = SUCCESS;
            break;
        }
    }
    while(--tqNum >= CTRL1_MIN_TIME_QUANTA);

    return (fgRet);
}

uint32_t FLEXCAN_GetFDMailboxOffset(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx)
{
    uint32_t offset   = 0;
    uint32_t dataSize = (flex_can->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT;

    switch (dataSize)
    {
        case (uint32_t)FLEXCAN_8BperMB:
            offset = (((uint32_t)mbIdx / 32U) * 512U + ((uint32_t)mbIdx % 32U) * 16U);
            break;

        case (uint32_t)FLEXCAN_16BperMB:
            offset = (((uint32_t)mbIdx / 21U) * 512U + ((uint32_t)mbIdx % 21U) * 24U);
            break;

        case (uint32_t)FLEXCAN_32BperMB:
            offset = (((uint32_t)mbIdx / 12U) * 512U + ((uint32_t)mbIdx % 12U) * 40U);
            break;

        case (uint32_t)FLEXCAN_64BperMB:
            offset = (((uint32_t)mbIdx / 7U) * 512U + ((uint32_t)mbIdx % 7U) * 72U);
            break;

        default:
             /* All the cases have been listed above, the default clause should not be reached. */
            break;
    }

     /* To get the dword aligned offset, need to divide by 4. */
    offset = offset / 4U;
    return (offset);
}

/**
  * @brief Calculates the segment values for a single bit time for CANFD bus control baud Rate
  *
  * @param baudRate The canfd bus control speed in bps
  * @param tqNum Number of time quanta per bit
  * @param pTimingConfig Pointer to the FlexCAN timing configuration structure.
  *
  * @return TRUE if Calculates the segment success, FALSE if Calculates the segment success
  */
static ErrorStatus FLEXCAN_FDGetSegments(uint32_t baudRate, uint32_t tqNum, flexcan_timing_config_t *pTimingConfig)
{
    uint32_t ideal_sp;
    uint32_t p1;
    ErrorStatus fgRet = ERROR;

     /* Get ideal sample point. */
    if (baudRate >= 1000000U)
    {
        ideal_sp = IDEAL_SP_LOW;
    }
    else if (baudRate >= 800000U)
    {
        ideal_sp = IDEAL_SP_MID;
    }
    else
    {
        ideal_sp = IDEAL_SP_HIGH;
    }

     /* distribute time quanta. */
    p1 = tqNum * (uint32_t)ideal_sp;
    pTimingConfig->propSeg = (uint8_t)(p1 / (uint32_t)IDEAL_SP_FACTOR - 3U);

    if (pTimingConfig->propSeg <= (MAX_EPSEG1 + MAX_EPROPSEG))
    {
        if (pTimingConfig->propSeg > MAX_EPROPSEG)
        {
            pTimingConfig->phaseSeg1 = pTimingConfig->propSeg - MAX_EPROPSEG;
            pTimingConfig->propSeg   = MAX_EPROPSEG;
        }
        else
        {
            pTimingConfig->phaseSeg1 = 0;
        }

        if (pTimingConfig->phaseSeg1 <= MAX_EPSEG1)
        {
             /* The value of prog Seg should be not larger than tqNum -4U. */
            if ((pTimingConfig->propSeg + pTimingConfig->phaseSeg1) < ((uint8_t)tqNum - 4U))
            {
                pTimingConfig->phaseSeg2 = (uint8_t)tqNum - (pTimingConfig->phaseSeg1 + pTimingConfig->propSeg + 4U);

                if (pTimingConfig->phaseSeg2 <= MAX_EPSEG2)
                {
                    if ((pTimingConfig->phaseSeg1 < pTimingConfig->phaseSeg2) && (pTimingConfig->propSeg > (pTimingConfig->phaseSeg2 - pTimingConfig->phaseSeg1)))
                    {
                        pTimingConfig->propSeg  -= (pTimingConfig->phaseSeg2 - pTimingConfig->phaseSeg1);
                        pTimingConfig->phaseSeg1 = pTimingConfig->phaseSeg2;
                    }

                     /* subtract one TQ for sync seg. */
                     /* sjw is 20% of total TQ, rounded to nearest int. */
                    pTimingConfig->rJumpwidth = ((uint8_t)tqNum + 4U) / 5U - 1U;

                     /* The max tqNum for CBT will reach to 129, ERJW would not be larger than 26. */
                     /* Considering that max ERJW is 31, rJumpwidth will always be smaller than MAX_ERJW. */
                    if (pTimingConfig->rJumpwidth > MAX_ERJW)
                    {
                        pTimingConfig->rJumpwidth = MAX_ERJW;
                    }

                    fgRet = SUCCESS;
                }
            }
        }
    }

    return (fgRet);
}

/**
  * @brief Calculates the segment values for a single bit time for CANFD bus data baud Rate
  *
  * @param baudRatebrs The canfd bus data speed in bps
  * @param tqNum Number of time quanta per bit
  * @param pTimingConfig Pointer to the FlexCAN timing configuration structure.
  *
  * @return SUCCESS if Calculates the segment success, ERROR if Calculates the segment success
  */
static ErrorStatus FLEXCAN_FDGetSegmentswithBRS(uint32_t baudRatebrs, uint32_t tqNum, flexcan_timing_config_t *pTimingConfig)
{
    uint32_t ideal_sp;
    uint32_t p1;
    ErrorStatus fgRet = ERROR;

     /* get ideal sample point. */
    if (baudRatebrs >= 1000000U)
    {
        ideal_sp = IDEAL_SP_LOW;
    }
    else if (baudRatebrs >= 800000U)
    {
        ideal_sp = IDEAL_SP_MID;
    }
    else
    {
        ideal_sp = IDEAL_SP_HIGH;
    }

     /* distribute time quanta. */
    p1 = tqNum * (uint32_t)ideal_sp;
    pTimingConfig->fpropSeg = (uint8_t)(p1 / (uint32_t)IDEAL_SP_FACTOR - 2U);

    if (pTimingConfig->fpropSeg <= (MAX_FPSEG1 + MAX_FPROPSEG))
    {
        if (pTimingConfig->fpropSeg > MAX_FPROPSEG)
        {
            pTimingConfig->fphaseSeg1 = pTimingConfig->fpropSeg - MAX_FPROPSEG;
            pTimingConfig->fpropSeg   = MAX_FPROPSEG;
        }
        else
        {
            pTimingConfig->fphaseSeg1 = 0;
        }

        if (pTimingConfig->fphaseSeg1 <= MAX_PSEG1)
        {
            if ((pTimingConfig->fpropSeg + pTimingConfig->fphaseSeg1) < ((uint8_t)tqNum - 3U))
            {
                pTimingConfig->fphaseSeg2 = (uint8_t)tqNum - (pTimingConfig->fphaseSeg1 + pTimingConfig->fpropSeg + 3U);

                if (pTimingConfig->fphaseSeg2 <= MAX_PSEG2)
                {
                    if ((pTimingConfig->fphaseSeg1 < pTimingConfig->fphaseSeg2) && (pTimingConfig->fpropSeg > (pTimingConfig->fphaseSeg2 - pTimingConfig->fphaseSeg1)))
                    {
                        pTimingConfig->fpropSeg  -= (pTimingConfig->fphaseSeg2 - pTimingConfig->fphaseSeg1);
                        pTimingConfig->fphaseSeg1 = pTimingConfig->fphaseSeg2;
                    }

                     /* subtract one TQ for sync seg. */
                     /* sjw is 20% of total TQ, rounded to nearest int. */
                    pTimingConfig->frJumpwidth = ((uint8_t)tqNum + 4U) / 5U - 1U;

                    if (pTimingConfig->frJumpwidth > MAX_FRJW)
                    {
                        pTimingConfig->frJumpwidth = MAX_FRJW;
                    }

                    fgRet = SUCCESS;
                }
            }
        }
    }

    return (fgRet);
}

/**
  * @brief Calculates the improved timing values by specific baudrates for CAN by CBT register
  *
  * @param baudRate  The classical CAN speed in bps defined by user
  * @param sourceClock_Hz The Source clock data speed in bps. Zero to disable baudrate switching
  * @param pTimingConfig Pointer to the FlexCAN timing configuration structure.
  *
  * @return SUCCESS if timing configuration found, ERROR if failed to find configuration
  */
static ErrorStatus FLEXCAN_CalculateImprovedTimingValuesByCBT(uint32_t baudRate, uint32_t sourceClock_Hz, flexcan_timing_config_t *pTimingConfig)
{
    uint32_t clk;                      /* the clock is tqNumb x baudRateFD. */
    uint32_t tqNum;                    /* Numbers of TQ. */
    ErrorStatus fgRet = ERROR;

    tqNum = CBT_MAX_TIME_QUANTA;

     /*  Auto Improved Protocal timing. */
    do
    {
        clk = baudRate * tqNum;

        if (clk > sourceClock_Hz)
        {
            continue;                  /* tqNum too large, clk has been exceed sourceClock_Hz. */
        }

        if ((sourceClock_Hz / clk * clk) != sourceClock_Hz)
        {
            continue;                  /* Non-supporting: the frequency of clock source is not divisible by target baud rate, the user
                                          should change a divisible baud rate. */
        }

         /* Make sure the new calculated divider value is greater than the previous one. */
        if (pTimingConfig->preDivider > ((uint16_t)(sourceClock_Hz / clk) - 1U))
        {
            continue;
        }
        else
        {
            pTimingConfig->preDivider = (uint16_t)(sourceClock_Hz / clk) - 1U;
        }

         /* To minimize errors when processing FD frames, try to calculate the same value for FPRESDIV and PRESDIV (in CBT). */
        if (pTimingConfig->preDivider != pTimingConfig->fpreDivider)
        {
            continue;
        }

        if (pTimingConfig->preDivider > MAX_EPRESDIV)
        {
            break;                     /* The frequency of source clock is too large or the baud rate is too small, the pre-divider could
                                          not handle it. */
        }

         /* Try to get the best timing configuration. */
        if (FLEXCAN_FDGetSegments(baudRate, tqNum, pTimingConfig))
        {
            fgRet = SUCCESS;
            break;
        }
    }
    while(--tqNum >= CBT_MIN_TIME_QUANTA);

    return (fgRet);
}

/**
  * @brief Calculates the improved timing values by specific baudrates for CAN by FDCBT register
  *
  * @param baudRate  The CANFD speed in bps defined by user
  * @param sourceClock_Hz The Source clock data speed in bps. Zero to disable baudrate switching
  * @param pTimingConfig Pointer to the FlexCAN timing configuration structure.
  *
  * @return SUCCESS if timing configuration found, ERROR if failed to find configuration
  */
static ErrorStatus FLEXCAN_CalculateImprovedTimingValuesByFDCBT(uint32_t baudRate, uint32_t sourceClock_Hz, flexcan_timing_config_t *pTimingConfig)
{
    uint32_t clk;                      /* the clock is tqNumb x baudRateFD. */
    uint32_t tqNum;                    /* Numbers of TQ. */
    ErrorStatus fgRet = ERROR;

    tqNum = FDCBT_MAX_TIME_QUANTA;

     /*  Auto Improved Protocal timing. */
    do
    {
        clk = baudRate * tqNum;

        if (clk > sourceClock_Hz)
        {
            continue;                  /* tqNum too large, clk has been exceed sourceClock_Hz. */
        }

        if ((sourceClock_Hz / clk * clk) != sourceClock_Hz)
        {
            continue;                  /* Non-supporting: the frequency of clock source is not divisible by target baud rate, the user
                                          should change a divisible baud rate. */
        }

         /* Make sure the new calculated divider value is greater than the previous one. */
        if (pTimingConfig->fpreDivider > ((uint16_t)(sourceClock_Hz / clk) - 1U))
        {
            continue;
        }
        else
        {
            pTimingConfig->fpreDivider = (uint16_t)(sourceClock_Hz / clk) - 1U;
        }

        if (pTimingConfig->fpreDivider > MAX_FPRESDIV)
        {
            break;                     /* The frequency of source clock is too large or the baud rate is too small, the pre-divider could
                                          not handle it. */
        }

         /* Try to get the best timing configuration. */
        if (FLEXCAN_FDGetSegmentswithBRS(baudRate, tqNum, pTimingConfig))
        {
            fgRet = SUCCESS;
            break;
        }
    }
    while(--tqNum >= FDCBT_MIN_TIME_QUANTA);

    return (fgRet);
}

/**
  * @brief Calculates the improved timing values by specific baudrates for CANFD
  *
  * @param baudRate  The CANFD bus control speed in bps defined by user
  * @param baudRateFD  The CANFD bus data speed in bps defined by user
  * @param sourceClock_Hz The Source clock data speed in bps. Zero to disable baudrate switching
  * @param pTimingConfig Pointer to the FlexCAN timing configuration structure.
  *
  * @return SUCCESS if timing configuration found, ERROR if failed to find configuration
  */
ErrorStatus FLEXCAN_FDCalculateImprovedTimingValues(uint32_t baudRate, uint32_t baudRateFD, uint32_t sourceClock_Hz, flexcan_timing_config_t *pTimingConfig)
{
    ErrorStatus fgRet = ERROR;

    pTimingConfig->preDivider  = 0U;
    pTimingConfig->fpreDivider = 0U;

    if (FLEXCAN_CalculateImprovedTimingValuesByFDCBT(baudRateFD, sourceClock_Hz, pTimingConfig))
    {
        if (FLEXCAN_CalculateImprovedTimingValuesByCBT(baudRate, sourceClock_Hz, pTimingConfig))
        {
            fgRet = SUCCESS;
        }
    }

    return (fgRet);
}

/**
  * brief Configures a FlexCAN transmit message buffer.
  *
  * This function aborts the previous transmission, cleans the Message Buffer, and
  * configures it as a Transmit Message Buffer.
  *
  * param flex_can FlexCAN peripheral base address.
  * param mbIdx The Message Buffer index.
  * param enable Enable/disable Tx Message Buffer.
  *               - ENABLE: Enable Tx Message Buffer.
  *               - DISABLE: Disable Tx Message Buffer.
  */
void FLEXCAN_FDTxMbConfig(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, FunctionalState enable)
{
    uint8_t cnt           = 0;
    uint8_t payload_dword = 1;
    uint32_t dataSize;

    dataSize                  = (flex_can->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT;

    volatile uint32_t *mbAddr = &(flex_can->MB[0].CS);
    uint32_t offset           = FLEXCAN_GetFDMailboxOffset(flex_can, mbIdx);

     /* Inactivate Message Buffer. */
    if (enable)
    {
         /* Inactivate by writing CS. */
        mbAddr[offset] = CAN_CS_CODE(Enum_Flexcan_TxMbInactive);
    }
    else
    {
        mbAddr[offset] = 0x0;
    }

    /* Calculate the DWORD number, dataSize 0/1/2/3 corresponds to 8/16/32/64
       Bytes payload. */
    for (cnt = 0; cnt < (dataSize + 1U); cnt++)
    {
        payload_dword *= 2U;
    }

     /* Clean ID. */
    mbAddr[offset + 1U] = 0x0U;

     /* Clean Message Buffer content, DWORD by DWORD. */
    for (cnt = 0; cnt < payload_dword; cnt++)
    {
        mbAddr[offset + 2U + cnt] = 0x0U;
    }
}

/**
  * @brief Configures a FlexCAN Receive Message Buffer.
  *
  * This function cleans a FlexCAN build-in Message Buffer and configures it
  * as a Receive Message Buffer.
  *
  * @param flexcan: FlexCAN peripheral Struct Point.
  * @param mbIdx: The Message Buffer index.
  * @param pRxMbConfig: Pointer to the FlexCAN Message Buffer configuration structure.
  * @param state:  This parameter can be: ENABLE or DISABLE.
  */
void FLEXCAN_RxMbConfig(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, FunctionalState state)
{
    uint32_t cs_temp = 0;

     /* Inactivate Message Buffer. */
    flex_can->MB[mbIdx].CS = 0;

     /* Clean Message Buffer content. */
    flex_can->MB[mbIdx].ID    = 0x0;
    flex_can->MB[mbIdx].WORD0 = 0x0;
    flex_can->MB[mbIdx].WORD1 = 0x0;

    if (ENABLE)
    {
         /* Setup Message Buffer ID. */
        flex_can->MB[mbIdx].ID = pRxMbConfig->id;

         /* Setup Message Buffer format. */
        if (Enum_Flexcan_FrameFormatExtend == pRxMbConfig->format)
        {
            cs_temp |= CAN_CS_IDE_MASK;
        }

         /* Setup Message Buffer type. */
        if (Enum_Flexcan_FrameTypeRemote == pRxMbConfig->type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }

         /* Activate Rx Message Buffer. */
        cs_temp |= CAN_CS_CODE(Enum_Flexcan_RxMbEmpty);
        flex_can->MB[mbIdx].CS = cs_temp;
    }
}

/**
  * @brief Configures a FlexCAN Receive Message Buffer.
  *
  * This function cleans a FlexCAN build-in Message Buffer and configures it
  * as a Receive Message Buffer.
  *
  * @param flex_can FlexCAN peripheral base address.
  * @param mbIdx The Message Buffer index.
  * @param pRxMbConfig Pointer to the FlexCAN Message Buffer configuration structure.
  * @param enable Enable/disable Rx Message Buffer.
  *               - ENABLE: Enable Rx Message Buffer.
  *               - DISABLE: Disable Rx Message Buffer.
  */
void FLEXCAN_FDRxMbConfig(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, FunctionalState state)
{
    uint32_t cs_temp          = 0;
    uint8_t cnt               = 0;
    volatile uint32_t *mbAddr = &(flex_can->MB[0].CS);
    uint32_t offset           = FLEXCAN_GetFDMailboxOffset(flex_can, mbIdx);
    uint8_t payload_dword;
    uint32_t dataSize = (flex_can->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT;

     /* Inactivate Message Buffer. */
    mbAddr[offset] = 0U;

     /* Clean Message Buffer content. */
    mbAddr[offset + 1U] = 0U;
    /* Calculate the DWORD number, dataSize 0/1/2/3 corresponds to 8/16/32/64
       Bytes payload. */
    payload_dword = (2U << dataSize);

    for (cnt = 0; cnt < payload_dword; cnt++)
    {
        mbAddr[offset + 2U + cnt] = 0x0;
    }

    if (state)
    {
         /* Setup Message Buffer ID. */
        mbAddr[offset + 1U] = pRxMbConfig->id;

         /* Setup Message Buffer format. */
        if (Enum_Flexcan_FrameFormatExtend == pRxMbConfig->format)
        {
            cs_temp |= CAN_CS_IDE_MASK;
        }

         /* Setup Message Buffer type. */
        if (Enum_Flexcan_FrameTypeRemote == pRxMbConfig->type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }

         /* Activate Rx Message Buffer. */
        cs_temp |= CAN_CS_CODE(Enum_Flexcan_RxMbEmpty);
        mbAddr[offset] = cs_temp;
    }
}

/**
  * @brief Configures the FlexCAN Rx FIFO.
  *
  * This function configures the Rx FIFO with given Rx FIFO configuration.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param pRxFifoConfig Pointer to the FlexCAN Rx FIFO configuration structure.
  * @param enable Enable/disable Rx FIFO.
  *               - ENABLE: Enable Rx FIFO.
  *               - DISABLE: Disable Rx FIFO.
  */
void FLEXCAN_RxFifoConfig(FLEXCAN_TypeDef *flex_can, const flexcan_rx_fifo_config_t *pRxFifoConfig, FunctionalState state)
{
    volatile uint32_t *mbAddr;
    uint8_t i, j, k, rffn = 0, numMbOccupy;
    uint32_t setup_mb = 0;

     /* Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

    if (state)
    {
         /* Get the setup_mb value. */
        setup_mb = (uint8_t)((flex_can->MCR & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT);
        setup_mb = (setup_mb < (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can)) ?
                   setup_mb :
                   (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can);

         /* Determine RFFN value. */
        for (i = 0; i <= 0xFU; i++)
        {
            if ((8U * (i + 1U)) >= pRxFifoConfig->idFilterNum)
            {
                rffn = i;
                /*assert(((setup_mb - 8U) - (2U * rffn)) > 0U); */

                flex_can->CTRL2 = (flex_can->CTRL2 & ~CAN_CTRL2_RFFN_MASK) | CAN_CTRL2_RFFN(rffn);
                break;
            }
        }

         /* caculate the Number of Mailboxes occupied by RX Legacy FIFO and the filter. */
        numMbOccupy = 6U + (rffn + 1U) * 2U;

         /* Copy ID filter table to Message Buffer Region (Fix MISRA_C-2012 Rule 18.1). */
        j = 0U;

        for (i = 6U; i < numMbOccupy; i++)
        {
             /* Get address for current mail box. */
            mbAddr = &(flex_can->MB[i].CS);

             /* One Mail box contain 4U DWORD registers. */
            for (k = 0; k < 4U; k++)
            {
                /* Fill all valid filter in the mail box occupied by filter. */
                /* Disable unused Rx FIFO Filter, the other rest of register in the last Mail box occupied by fiter set */
                /* as 0xffffffff. */

                mbAddr[k] = (j < pRxFifoConfig->idFilterNum) ? (pRxFifoConfig->idFilterTable[j]) : 0xFFFFFFFFU;

                 /* Try to fill next filter in current Mail Box. */
                j++;
            }
        }

         /* Setup ID Fitlter Type. */
        switch (pRxFifoConfig->idFilterType)
        {
            case Enum_Flexcan_RxFifoFilterTypeA:
                flex_can->MCR = (flex_can->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x0);
                break;

            case Enum_Flexcan_RxFifoFilterTypeB:
                flex_can->MCR = (flex_can->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x1);
                break;

            case Enum_Flexcan_RxFifoFilterTypeC:
                flex_can->MCR = (flex_can->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x2);
                break;

            case Enum_Flexcan_RxFifoFilterTypeD:
                 /* All frames rejected. */
                flex_can->MCR = (flex_can->MCR & ~CAN_MCR_IDAM_MASK) | CAN_MCR_IDAM(0x3);
                break;

            default:
                 /* All the cases have been listed above, the default clause should not be reached. */
                break;
        }

         /* Setting Message Reception Priority. */
        flex_can->CTRL2 = (pRxFifoConfig->priority == Enum_Flexcan_RxFifoPrioHigh) ? (flex_can->CTRL2 & ~CAN_CTRL2_MRP_MASK) :
                          (flex_can->CTRL2 | CAN_CTRL2_MRP_MASK);

         /* Enable Rx Message FIFO. */
        flex_can->MCR |= CAN_MCR_RFEN_MASK;
    }
    else
    {
        rffn = (uint8_t)((flex_can->CTRL2 & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
         /* caculate the Number of Mailboxes occupied by RX Legacy FIFO and the filter. */
        numMbOccupy = 6U + (rffn + 1U) * 2U;

         /* Disable Rx Message FIFO. */
        flex_can->MCR &= ~CAN_MCR_RFEN_MASK;

         /* Clean MB0 ~ MB5 and all MB occupied by ID filters (Fix MISRA_C-2012 Rule 18.1). */
        for (i = 0; i < numMbOccupy; i++)
        {
            FLEXCAN_RxMbConfig(flex_can, i, NULL, DISABLE);
        }
    }

     /* Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

#if (defined (FLEXCAN_HAS_RX_FIFO_DMA) && FLEXCAN_HAS_RX_FIFO_DMA)

/**
  * @brief  Gets the Rx FIFO Head address.
  *
  * This function returns the FlexCAN Rx FIFO Head address, which is mainly used for the DMA/eDMA use case.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @return FlexCAN Rx FIFO Head address.
  */
__STATIC_INLINE uint32_t FLEXCAN_GetRxFifoHeadAddr(FLEXCAN_TypeDef *flex_can)
{
    return ((uint32_t)&(flex_can->MB[0].CS));
}

/**
  * @brief Enables or disables the FlexCAN Rx FIFO DMA request.
  *
  * This function enables or disables the DMA feature of FlexCAN build-in Rx FIFO.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param ENABLE true to enable, DISABLE to disable.
  */
void FLEXCAN_EnableRxFifoDMA(FLEXCAN_TypeDef *flex_can, FunctionalState state)
{
    if (state)
    {
         /* Enter Freeze Mode. */
        FLEXCAN_EnterFreezeMode(flex_can);

         /* Enable FlexCAN DMA. */
        flex_can->MCR |= CAN_MCR_DMA_MASK;

         /* Exit Freeze Mode. */
        FLEXCAN_ExitFreezeMode(flex_can);
    }
    else
    {
         /* Enter Freeze Mode. */
        FLEXCAN_EnterFreezeMode(flex_can);

         /* Disable FlexCAN DMA. */
        flex_can->MCR &= ~CAN_MCR_DMA_MASK;

         /* Exit Freeze Mode. */
        FLEXCAN_ExitFreezeMode(flex_can);
    }
}

#endif /* FLEXCAN_HAS_RX_FIFO_DMA */

#if (defined (FLEXCAN_HAS_ERRATA_6032) && FLEXCAN_HAS_ERRATA_6032)

/**
  * FlexCAN: A frame with wrong ID or payload is transmitted into
  * the CAN bus when the Message Buffer under transmission is
  * either aborted or deactivated while the CAN bus is in the Bus Idle state
  *
  * This function to do workaround for ERR006032
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mbIdx The FlexCAN Message Buffer index.
  */
static void FLEXCAN_ERRATA_6032(FLEXCAN_TypeDef *flex_can, volatile uint32_t *mbCSAddr)
{
    uint32_t dbg_temp      = 0U;
    uint32_t uint32_tTempCS     = 0U;
    uint32_t uint32_tTimeout    = DELAY_BUSIDLE;
    uint32_t uint32_tTempIMASK1 = flex_can->IMASK1;

     /* after backup all interruption, disable ALL interruption */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint32_t uint32_tTempIMASK2 = flex_can->IMASK2;

    flex_can->IMASK2           = 0;
#endif
    flex_can->IMASK1 = 0;
    dbg_temp         = (uint32_t)(flex_can->DBG1);

    switch (dbg_temp & CAN_DBG1_CFSM_MASK)
    {
        case RXINTERMISSION:

            if (CBN_VALUE3 == (dbg_temp & CAN_DBG1_CBN_MASK))
            {
                 /* wait until CFSM is different from RXINTERMISSION */
                while (RXINTERMISSION == (flex_can->DBG1 & CAN_DBG1_CFSM_MASK))
                {
                    __NOP();
                }
            }

            break;

        case TXINTERMISSION:

            if (CBN_VALUE3 == (dbg_temp & CAN_DBG1_CBN_MASK))
            {
                 /* wait until CFSM is different from TXINTERMISSION */
                while (TXINTERMISSION == (flex_can->DBG1 & CAN_DBG1_CFSM_MASK))
                {
                    __NOP();
                }
            }

            break;

        default:
             /* To avoid MISRA-C 2012 rule 16.4 issue. */
            break;
    }

     /* Anyway, BUSIDLE need to delay */
    if (BUSIDLE == (flex_can->DBG1 & CAN_DBG1_CFSM_MASK))
    {
        while (uint32_tTimeout-- > 0U)
        {
            __NOP();
        }

         /* Write 0x0 into Code field of CS word. */
        uint32_tTempCS  = (uint32_t)(*mbCSAddr);
        uint32_tTempCS &= ~CAN_CS_CODE_MASK;
        *mbCSAddr  = uint32_tTempCS;
    }

     /* restore interruption */
    flex_can->IMASK1 = uint32_tTempIMASK1;
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    flex_can->IMASK2 = uint32_tTempIMASK2;
#endif
}

#endif

/**
  * @brief Writes a FlexCAN Message to the Transmit Message Buffer.
  *
  * This function writes a CAN Message to the specified Transmit Message Buffer
  * and changes the Message Buffer state to start CAN Message transmit. After
  * that the function returns immediately.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mbIdx The FlexCAN Message Buffer index.
  * @param pTxFrame Pointer to CAN message frame to be sent.
  * @retval Status_Flexcan_Success - Write Tx Message Buffer Successfully.
  * @retval Status_Flexcan_Fail    - Tx Message Buffer is currently in use.
  */
uint32_t FLEXCAN_WriteTxMb(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_frame_t *pTxFrame)
{
    uint32_t cs_temp = 0;
    uint32_t status;

#if (defined (FLEXCAN_HAS_ERRATA_6032) && FLEXCAN_HAS_ERRATA_6032)
    FLEXCAN_ERRATA_6032(flex_can, &(flex_can->MB[mbIdx].CS));
#endif

     /* Check if Message Buffer is available. */
    if (CAN_CS_CODE(Enum_Flexcan_TxMbDataOrRemote) != (flex_can->MB[mbIdx].CS & CAN_CS_CODE_MASK))
    {
         /* Inactive Tx Message Buffer. */
        flex_can->MB[mbIdx].CS = (flex_can->MB[mbIdx].CS & ~CAN_CS_CODE_MASK) | CAN_CS_CODE(Enum_Flexcan_TxMbInactive);

         /* Fill Message ID field. */
        flex_can->MB[mbIdx].ID = pTxFrame->id;

         /* Fill Message Format field. */
        if ((uint32_t)Enum_Flexcan_FrameFormatExtend == pTxFrame->format)
        {
            cs_temp |= CAN_CS_SRR_MASK | CAN_CS_IDE_MASK;
        }

         /* Fill Message Type field. */
        if ((uint32_t)Enum_Flexcan_FrameTypeRemote == pTxFrame->type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }

        cs_temp |= CAN_CS_CODE(Enum_Flexcan_TxMbDataOrRemote) | CAN_CS_DLC(pTxFrame->length);

         /* Load Message Payload. */
        flex_can->MB[mbIdx].WORD0 = pTxFrame->dataWord0;
        flex_can->MB[mbIdx].WORD1 = pTxFrame->dataWord1;

         /* Activate Tx Message Buffer. */
        flex_can->MB[mbIdx].CS = cs_temp;

#if ((defined (FLEXCAN_HAS_ERRATA_5641) && FLEXCAN_HAS_ERRATA_5641) || \
        (defined (FLEXCAN_HAS_ERRATA_5829) && FLEXCAN_HAS_ERRATA_5829))
        flex_can->MB[FLEXCAN_GetFirstValidMb(flex_can)].CS = CAN_CS_CODE(Enum_Flexcan_TxMbInactive);
        flex_can->MB[FLEXCAN_GetFirstValidMb(flex_can)].CS = CAN_CS_CODE(Enum_Flexcan_TxMbInactive);
#endif

        status = Status_Flexcan_Success;
    }
    else
    {
         /* Tx Message Buffer is activated, return immediately. */
        status = Status_Flexcan_Fail;
    }

    return (status);
}

/*!
  * @brief Writes a FlexCAN FD Message to the Transmit Message Buffer.
  *
  * This function writes a CAN FD Message to the specified Transmit Message Buffer
  * and changes the Message Buffer state to start CAN FD Message transmit. After
  * that the function returns immediately.
  *
  * @param flex_can FlexCAN peripheral base address.
  * @param mbIdx The FlexCAN FD Message Buffer index.
  * @param pTxFrame Pointer to CAN FD message frame to be sent.
  * @retval Status_Success - Write Tx Message Buffer Successfully.
  * @retval Status_Fail    - Tx Message Buffer is currently in use.
  */
int32_t FLEXCAN_WriteFDTxMb(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_fd_frame_t *pTxFrame)
{
    int32_t status;
    uint32_t cs_temp      = 0;
    uint8_t cnt           = 0;
    uint32_t can_cs       = 0;
    uint8_t payload_dword = 1;
    uint32_t dataSize     = (flex_can->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT;

#if ((defined (FLEXCAN_HAS_ERRATA_5641) && FLEXCAN_HAS_ERRATA_5641) || \
    (defined (FLEXCAN_HAS_ERRATA_5829) && FLEXCAN_HAS_ERRATA_5829))
    uint32_t availoffset = FLEXCAN_GetFDMailboxOffset(flex_can, FLEXCAN_GetFirstValidMb(flex_can));
#endif
    volatile uint32_t *mbAddr = &(flex_can->MB[0].CS);
    uint32_t offset           = FLEXCAN_GetFDMailboxOffset(flex_can, mbIdx);

#if (defined (FLEXCAN_HAS_ERRATA_6032) && FLEXCAN_HAS_ERRATA_6032)
    FLEXCAN_ERRATA_6032(flex_can, &(mbAddr[offset]));
#endif

    can_cs = mbAddr[offset];

     /* Check if Message Buffer is available. */
    if (CAN_CS_CODE(Enum_Flexcan_TxMbDataOrRemote) != (can_cs & CAN_CS_CODE_MASK))
    {
         /* Inactive Tx Message Buffer and Fill Message ID field. */
        mbAddr[offset]      = (can_cs & ~CAN_CS_CODE_MASK) | CAN_CS_CODE(Enum_Flexcan_TxMbInactive);

         /* Fill Message Format field. */
        if ((uint32_t)Enum_Flexcan_FrameFormatExtend == pTxFrame->format)
        {
            mbAddr[offset + 1U] = pTxFrame->id;
            cs_temp |= CAN_CS_SRR_MASK | CAN_CS_IDE_MASK;
        }
        else
        {
            mbAddr[offset + 1U] = FLEXCAN_ID_STD(pTxFrame->id);
            cs_temp &= ~(CAN_CS_SRR_MASK | CAN_CS_IDE_MASK);
        }

         /* Fill Message Type field. */
        if ((uint32_t)Enum_Flexcan_FrameTypeRemote == pTxFrame->type)
        {
            cs_temp |= CAN_CS_RTR_MASK;
        }
        else
        {
            cs_temp &= ~CAN_CS_RTR_MASK;
        }

        cs_temp |= CAN_CS_CODE(Enum_Flexcan_TxMbDataOrRemote) | CAN_CS_DLC(pTxFrame->length) | CAN_CS_EDL(pTxFrame->edl) | CAN_CS_BRS(pTxFrame->brs);

        /* Calculate the DWORD number, dataSize 0/1/2/3 corresponds to 8/16/32/64
           Bytes payload. */
        for (cnt = 0; cnt < (dataSize + 1U); cnt++)
        {
            payload_dword *= 2U;
        }

         /* Load Message Payload and Activate Tx Message Buffer. */
        for (cnt = 0; cnt < payload_dword; cnt++)
        {
            mbAddr[offset + 2U + cnt] = pTxFrame->dataWord[cnt];
        }

        mbAddr[offset] = cs_temp;

#if ((defined (FLEXCAN_HAS_ERRATA_5641) && FLEXCAN_HAS_ERRATA_5641) || \
        (defined (FLEXCAN_HAS_ERRATA_5829) && FLEXCAN_HAS_ERRATA_5829))
        mbAddr[availoffset] = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
        mbAddr[availoffset] = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
#endif
        status = Status_Flexcan_Success;
    }
    else
    {
         /* Tx Message Buffer is activated, return immediately. */
        status = Status_Flexcan_Fail;
    }

    return (status);
}

/**
  * @brief Reads a FlexCAN Message from Receive Message Buffer.
  *
  * This function reads a CAN message from a specified Receive Message Buffer.
  * The function fills a receive CAN message frame structure with
  * just received data and activates the Message Buffer again.
  * The function returns immediately.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mbIdx The FlexCAN Message Buffer index.
  * @param pRxFrame Pointer to CAN message frame structure for reception.
  * @retval Status_Flexcan_Success            - Rx Message Buffer is full and has been read successfully.
  * @retval Status_Flexcan_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
  * @retval Status_Flexcan_Fail               - Rx Message Buffer is empty.
  */
uint32_t FLEXCAN_ReadRxMb(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pRxFrame)
{
    uint32_t cs_temp;
    uint32_t rx_code;
    uint32_t status;

     /* Read CS field of Rx Message Buffer to lock Message Buffer. */
    cs_temp = flex_can->MB[mbIdx].CS;
     /* Get Rx Message Buffer Code field. */
    rx_code = (cs_temp & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT;

     /* Check to see if Rx Message Buffer is full. */
    if (((uint32_t)Enum_Flexcan_RxMbFull == rx_code) || ((uint32_t)Enum_Flexcan_RxMbOverrun == rx_code))
    {
         /* Store Message ID. */
        pRxFrame->id = flex_can->MB[mbIdx].ID & (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);

         /* Get the message ID and format. */
        pRxFrame->format = (cs_temp & CAN_CS_IDE_MASK) != 0U ? (uint8_t)Enum_Flexcan_FrameFormatExtend :
                           (uint8_t)Enum_Flexcan_FrameFormatStandard;

         /* Get the message type. */
        pRxFrame->type =
            (cs_temp & CAN_CS_RTR_MASK) != 0U ? (uint8_t)Enum_Flexcan_FrameTypeRemote : (uint8_t)Enum_Flexcan_FrameTypeData;

         /* Get the message length. */
        pRxFrame->length = (uint8_t)((cs_temp & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);

         /* Get the time stamp. */
        pRxFrame->timestamp = (uint16_t)((cs_temp & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

         /* Store Message Payload. */
        pRxFrame->dataWord0 = flex_can->MB[mbIdx].WORD0;
        pRxFrame->dataWord1 = flex_can->MB[mbIdx].WORD1;

         /* Read free-running timer to unlock Rx Message Buffer. */
        (void)flex_can->TIMER;

        if ((uint32_t)Enum_Flexcan_RxMbFull == rx_code)
        {
            status = Status_Flexcan_Success;
        }
        else
        {
            status = Status_Flexcan_RxOverflow;
        }
    }
    else
    {
         /* Read free-running timer to unlock Rx Message Buffer. */
        (void)flex_can->TIMER;

        status = Status_Flexcan_Fail;
    }

    return (status);
}

/**
  * @brief Reads a FlexCAN FD Message from Receive Message Buffer.
  *
  * This function reads a CAN FD message from a specified Receive Message Buffer.
  * The function fills a receive CAN FD message frame structure with
  * just received data and activates the Message Buffer again.
  * The function returns immediately.
  *
  * @param flex_can FlexCAN peripheral base address.
  * @param mbIdx The FlexCAN FD Message Buffer index.
  * @param pRxFrame Pointer to CAN FD message frame structure for reception.
  * @retval Status_Success            - Rx Message Buffer is full and has been read successfully.
  * @retval Status_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
  * @retval Status_Fail               - Rx Message Buffer is empty.
  */
int32_t FLEXCAN_ReadFDRxMb(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_fd_frame_t *pRxFrame)
{
    int32_t status;
    uint32_t cs_temp;
    uint8_t rx_code;
    uint8_t cnt     = 0;
    uint32_t can_id = 0;
    uint32_t dataSize;

    dataSize                  = (flex_can->FDCTRL & CAN_FDCTRL_MBDSR0_MASK) >> CAN_FDCTRL_MBDSR0_SHIFT;

    uint8_t payload_dword     = 1;
    volatile uint32_t *mbAddr = &(flex_can->MB[0].CS);
    uint32_t offset           = FLEXCAN_GetFDMailboxOffset(flex_can, mbIdx);

     /* Read CS field of Rx Message Buffer to lock Message Buffer. */
    cs_temp = mbAddr[offset];
    can_id  = mbAddr[offset + 1U];

     /* Get Rx Message Buffer Code field. */
    rx_code = (uint8_t)((cs_temp & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT);

     /* Check to see if Rx Message Buffer is full. */
    if (((uint8_t)Enum_Flexcan_RxMbFull == rx_code) || ((uint8_t)Enum_Flexcan_RxMbOverrun == rx_code))
    {
         /* Store Message ID. */
        pRxFrame->id = can_id & (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);

         /* Get the message ID and format. */
        pRxFrame->format = (cs_temp & CAN_CS_IDE_MASK) != 0U ? (uint8_t)Enum_Flexcan_FrameFormatExtend :
                           (uint8_t)Enum_Flexcan_FrameFormatStandard;

         /* Get the message type. */
        pRxFrame->type =
            (cs_temp & CAN_CS_RTR_MASK) != 0U ? (uint8_t)Enum_Flexcan_FrameTypeRemote : (uint8_t)Enum_Flexcan_FrameTypeData;

         /* Get the message length. */
        pRxFrame->length = (uint8_t)((cs_temp & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);

         /* Get the time stamp. */
        pRxFrame->timestamp = (uint16_t)((cs_temp & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        /* Calculate the DWORD number, dataSize 0/1/2/3 corresponds to 8/16/32/64
           Bytes payload. */
        for (cnt = 0; cnt < (dataSize + 1U); cnt++)
        {
            payload_dword *= 2U;
        }

         /* Store Message Payload. */
        for (cnt = 0; cnt < payload_dword; cnt++)
        {
            pRxFrame->dataWord[cnt] = mbAddr[offset + 2U + cnt];
        }

         /* Read free-running timer to unlock Rx Message Buffer. */
        (void)flex_can->TIMER;

        if ((uint32_t)Enum_Flexcan_RxMbFull == rx_code)
        {
            status = Status_Flexcan_Success;
        }
        else
        {
            status = Status_Flexcan_RxOverflow;
        }
    }
    else
    {
         /* Read free-running timer to unlock Rx Message Buffer. */
        (void)flex_can->TIMER;

        status = Status_Flexcan_Fail;
    }

    return (status);
}

/**
  * @brief Reads a FlexCAN Message from Rx FIFO.
  *
  * This function reads a CAN message from the FlexCAN build-in Rx FIFO.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param pRxFrame Pointer to CAN message frame structure for reception.
  * @retval Status_Flexcan_Success - Read Message from Rx FIFO successfully.
  * @retval Status_Flexcan_Fail    - Rx FIFO is not enabled.
  */
uint32_t FLEXCAN_ReadRxFifo(FLEXCAN_TypeDef *flex_can, flexcan_frame_t *pRxFrame)
{
    uint32_t cs_temp;
    uint32_t status;

     /* Check if Rx FIFO is Enabled. */
    if (0U != (flex_can->MCR & CAN_MCR_RFEN_MASK))
    {
         /* Read CS field of Rx Message Buffer to lock Message Buffer. */
        cs_temp = flex_can->MB[0].CS;

         /* Read data from Rx FIFO output port. */
         /* Store Message ID. */
        pRxFrame->id = flex_can->MB[0].ID & (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);

         /* Get the message ID and format. */
        pRxFrame->format = (cs_temp & CAN_CS_IDE_MASK) != 0U ? (uint8_t)Enum_Flexcan_FrameFormatExtend :
                           (uint8_t)Enum_Flexcan_FrameFormatStandard;

         /* Get the message type. */
        pRxFrame->type =
            (cs_temp & CAN_CS_RTR_MASK) != 0U ? (uint8_t)Enum_Flexcan_FrameTypeRemote : (uint8_t)Enum_Flexcan_FrameTypeData;

         /* Get the message length. */
        pRxFrame->length = (uint8_t)((cs_temp & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);

         /* Get the time stamp. */
        pRxFrame->timestamp = (uint16_t)((cs_temp & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

         /* Store Message Payload. */
        pRxFrame->dataWord0 = flex_can->MB[0].WORD0;
        pRxFrame->dataWord1 = flex_can->MB[0].WORD1;

         /* Store ID Filter Hit Index. */
        pRxFrame->idhit = (uint16_t)(flex_can->RXFIR & CAN_RXFIR_IDHIT_MASK);

         /* Read free-running timer to unlock Rx Message Buffer. */
        (void)flex_can->TIMER;

        status = Status_Flexcan_Success;
    }
    else
    {
        status = Status_Flexcan_Fail;
    }

    return (status);
}

/**
  * @brief Performs a polling send transaction on the CAN bus.
  *
  * Note that a transfer handle does not need to be created  before calling this API.
  *
  * @param flex_can FlexCAN peripheral flex_can pointer.
  * @param mbIdx The FlexCAN Message Buffer index.
  * @param pTxFrame Pointer to CAN message frame to be sent.
  * @retval Status_Flexcan_Success - Write Tx Message Buffer Successfully.
  * @retval Status_Flexcan_Fail    - Tx Message Buffer is currently in use.
  */
uint32_t FLEXCAN_TransferSendBlocking(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pTxFrame)
{
    uint32_t status;
    uint32_t uint32_tflag = 1;

     /* Write Tx Message Buffer to initiate a data sending. */
    if (Status_Flexcan_Success == FLEXCAN_WriteTxMb(flex_can, mbIdx, (const flexcan_frame_t *)(uint32_t)pTxFrame))
    {
         /* Wait until CAN Message send out. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        uint64_t u64flag = 1;

        while (0U == FLEXCAN_GetMbStatusFlags(flex_can, u64flag << mbIdx))
#else
        while (0U == FLEXCAN_GetMbStatusFlags(flex_can, uint32_tflag << mbIdx))
#endif
        {
        }

         /* Clean Tx Message Buffer Flag. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        FLEXCAN_ClearMbStatusFlags(flex_can, u64flag << mbIdx);
#else
        FLEXCAN_ClearMbStatusFlags(flex_can, uint32_tflag << mbIdx);
#endif
         /* After TX MB tranfered success, update the Timestamp from MB[mbIdx].CS register */
        pTxFrame->timestamp = (uint16_t)((flex_can->MB[mbIdx].CS & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        status = Status_Flexcan_Success;
    }
    else
    {
        status = Status_Flexcan_Fail;
    }

    return (status);
}

/**
  * @brief Performs a polling receive transaction on the CAN bus.
  *
  * Note that a transfer handle does not need to be created  before calling this API.
  *
  * @param flex_can FlexCAN peripheral flex_can pointer.
  * @param mbIdx The FlexCAN Message Buffer index.
  * @param pRxFrame Pointer to CAN message frame structure for reception.
  * @retval Status_Flexcan_Success            - Rx Message Buffer is full and has been read successfully.
  * @retval Status_Flexcan_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
  * @retval Status_Flexcan_Fail               - Rx Message Buffer is empty.
  */
uint32_t FLEXCAN_TransferReceiveBlocking(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pRxFrame)
{
    uint32_t uint32_tflag = 1;

     /* Wait until Rx Message Buffer non-empty. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint64_t u64flag = 1;

    while (0U == FLEXCAN_GetMbStatusFlags(flex_can, u64flag << mbIdx))
#else
    while (0U == FLEXCAN_GetMbStatusFlags(flex_can, uint32_tflag << mbIdx))
#endif
    {
    }

     /* Clean Rx Message Buffer Flag. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    FLEXCAN_ClearMbStatusFlags(flex_can, u64flag << mbIdx);
#else
    FLEXCAN_ClearMbStatusFlags(flex_can, uint32_tflag << mbIdx);
#endif

     /* Read Received CAN Message. */
    return (FLEXCAN_ReadRxMb(flex_can, mbIdx, pRxFrame));
}

/**
  * @brief Performs a polling send transaction on the CAN bus.
  *
  * Note that a transfer handle does not need to be created before calling this API.
  *
  * @param flex_can FlexCAN peripheral base pointer.
  * @param mbIdx The FlexCAN FD Message Buffer index.
  * @param pTxFrame Pointer to CAN FD message frame to be sent.
  * @retval Status_Success - Write Tx Message Buffer Successfully.
  * @retval Status_Fail    - Tx Message Buffer is currently in use.
  */
int32_t FLEXCAN_TransferFDSendBlocking(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_fd_frame_t *pTxFrame)
{
    int32_t status;

     /* Write Tx Message Buffer to initiate a data sending. */
    if (Status_Flexcan_Success == FLEXCAN_WriteFDTxMb(flex_can, mbIdx, (const flexcan_fd_frame_t *)(uint32_t)pTxFrame))
    {
/* Wait until CAN Message send out. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        uint64_t u64flag = 1;

        while (0U == FLEXCAN_GetMbStatusFlags(flex_can, u64flag << mbIdx))
#else
        uint32_t uint32_tflag = 1;

        while (0U == FLEXCAN_GetMbStatusFlags(flex_can, uint32_tflag << mbIdx))
#endif
        {
        }

/* Clean Tx Message Buffer Flag. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        FLEXCAN_ClearMbStatusFlags(flex_can, u64flag << mbIdx);
#else
        FLEXCAN_ClearMbStatusFlags(flex_can, uint32_tflag << mbIdx);
#endif
         /*After TX MB tranfered success, update the Timestamp from flex_can->MB[offset for CANFD].CS register*/
        volatile uint32_t *mbAddr = &(flex_can->MB[0].CS);
        uint32_t offset           = FLEXCAN_GetFDMailboxOffset(flex_can, mbIdx);

        pTxFrame->timestamp       = (uint16_t)((mbAddr[offset] & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);

        status = Status_Flexcan_Success;
    }
    else
    {
        status = Status_Flexcan_Fail;
    }

    return (status);
}

/**
  * @brief Performs a polling receive transaction on the CAN bus.
  *
  * Note that a transfer handle does not need to be created before calling this API.
  *
  * @param flex_can FlexCAN peripheral base pointer.
  * @param mbIdx The FlexCAN FD Message Buffer index.
  * @param pRxFrame Pointer to CAN FD message frame structure for reception.
  * @retval Status_Success            - Rx Message Buffer is full and has been read successfully.
  * @retval Status_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
  * @retval Status_Fail               - Rx Message Buffer is empty.
  */
int32_t FLEXCAN_TransferFDReceiveBlocking(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_fd_frame_t *pRxFrame)
{
/* Wait until Rx Message Buffer non-empty. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint64_t u64flag = 1;

    while (0U == FLEXCAN_GetMbStatusFlags(flex_can, u64flag << mbIdx))
#else
    uint32_t uint32_tflag = 1;

    while (0U == FLEXCAN_GetMbStatusFlags(flex_can, uint32_tflag << mbIdx))
#endif
    {
    }

/* Clean Rx Message Buffer Flag. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    FLEXCAN_ClearMbStatusFlags(flex_can, u64flag << mbIdx);
#else
    FLEXCAN_ClearMbStatusFlags(flex_can, uint32_tflag << mbIdx);
#endif

     /* Read Received CAN Message. */
    return (FLEXCAN_ReadFDRxMb(flex_can, mbIdx, pRxFrame));
}

/**
  * @brief Performs a polling receive transaction from Rx FIFO on the CAN bus.
  *
  * Note that a transfer handle does not need to be created  before calling this API.
  *
  * @param flex_can FlexCAN peripheral flex_can pointer.
  * @param pRxFrame Pointer to CAN message frame structure for reception.
  * @retval Status_Flexcan_Success - Read Message from Rx FIFO successfully.
  * @retval Status_Flexcan_Fail    - Rx FIFO is not enabled.
  */
uint32_t FLEXCAN_TransferReceiveFifoBlocking(FLEXCAN_TypeDef *flex_can, flexcan_frame_t *pRxFrame)
{
    uint32_t rxFifoStatus;

     /* Wait until Rx FIFO non-empty. */
    while (0U == FLEXCAN_GetMbStatusFlags(flex_can, (uint32_t)Enum_Flexcan_RxFifoFrameAvlFlag))
    {
    }

    rxFifoStatus = FLEXCAN_ReadRxFifo(flex_can, pRxFrame);

     /* Clean Rx Fifo available flag. */
    FLEXCAN_ClearMbStatusFlags(flex_can, (uint32_t)Enum_Flexcan_RxFifoFrameAvlFlag);

    return (rxFifoStatus);
}

/**
  * @brief Initializes the FlexCAN handle.
  *
  * This function initializes the FlexCAN handle, which can be used for other FlexCAN
  * transactional APIs. Usually, for a specified FlexCAN instance,
  * call this API once to get the initialized handle.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param handle FlexCAN handle pointer.
  * @param callback The callback function.
  * @param userData The parameter of the callback function.
  */
void FLEXCAN_TransferCreateHandle(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_transfer_callback_t callback, void *userData)
{
     /* Clean FlexCAN transfer handle. */
    (void)memset(handle, 0, sizeof(*handle));

    /* Save the context in global variables to support the double weak mechanism. */
    /* s_flexcanHandle[instance] = handle; */

     /* Register Callback function. */
    handle->callback = callback;
    handle->userData = userData;

    /* s_flexcanIsr = FLEXCAN_TransferHandleIRQ; */

    /* We Enable Error & Status interrupt here, because this interrupt just */
    /* report current status of FlexCAN module through Callback function. */
    /* It is insignificance without a available callback function. */

    if (handle->callback != NULL)
    {
        FLEXCAN_EnableInterrupts(
            flex_can, (uint32_t)Enum_Flexcan_BusOffInterruptEnable | (uint32_t)Enum_Flexcan_ErrorInterruptEnable |
            (uint32_t)Enum_Flexcan_RxWarningInterruptEnable | (uint32_t)Enum_Flexcan_TxWarningInterruptEnable |
            (uint32_t)Enum_Flexcan_WakeUpInterruptEnable);
    }
    else
    {
        FLEXCAN_DisableInterrupts(
            flex_can, (uint32_t)Enum_Flexcan_BusOffInterruptEnable | (uint32_t)Enum_Flexcan_ErrorInterruptEnable |
            (uint32_t)Enum_Flexcan_RxWarningInterruptEnable | (uint32_t)Enum_Flexcan_TxWarningInterruptEnable |
            (uint32_t)Enum_Flexcan_WakeUpInterruptEnable);
    }
}

/**
  * @brief Sends a message using IRQ.
  *
  * This function sends a message using IRQ. This is a non-blocking function, which returns
  * right away. When messages have been sent out, the send callback function is called.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param handle FlexCAN handle pointer.
  * @param pMbXfer FlexCAN Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
  * @retval Status_Flexcan_Success        Start Tx Message Buffer sending process successfully.
  * @retval Status_Flexcan_Fail           Write Tx Message Buffer failed.
  * @retval Status_Flexcan_TxBusy Tx Message Buffer is in use.
  */
uint32_t FLEXCAN_TransferSendNonBlocking(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    uint32_t status;
    uint32_t uint32_tmask = 1;

     /* Check if Message Buffer is idle. */
    if ((uint8_t)Enum_Flexcan_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
         /* Distinguish transmit type. */
        if ((uint32_t)Enum_Flexcan_FrameTypeRemote == pMbXfer->frame->type)
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)Enum_Flexcan_StateTxRemote;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)Enum_Flexcan_StateTxData;
        }

        if (Status_Flexcan_Success == FLEXCAN_WriteTxMb(flex_can, pMbXfer->mbIdx, (const flexcan_frame_t *)(uint32_t)pMbXfer->frame))
        {
             /* Enable Message Buffer Interrupt. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
            uint64_t u64mask = 1;

            FLEXCAN_EnableMbInterrupts(flex_can, u64mask << pMbXfer->mbIdx);
#else
            FLEXCAN_EnableMbInterrupts(flex_can, uint32_tmask << pMbXfer->mbIdx);
#endif
            status = Status_Flexcan_Success;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)Enum_Flexcan_StateIdle;
            status                          = Status_Flexcan_Fail;
        }
    }
    else
    {
        status = Status_Flexcan_TxBusy;
    }

    return (status);
}

/**
  * @brief Receives a message using IRQ.
  *
  * This function receives a message using IRQ. This is non-blocking function, which returns
  * right away. When the message has been received, the receive callback function is called.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param handle FlexCAN handle pointer.
  * @param pMbXfer FlexCAN Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
  * @retval Status_Flexcan_Success        - Start Rx Message Buffer receiving process successfully.
  * @retval Status_Flexcan_RxBusy - Rx Message Buffer is in use.
  */
uint32_t FLEXCAN_TransferReceiveNonBlocking(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    uint32_t status;
    uint32_t uint32_tmask = 1;

     /* Check if Message Buffer is idle. */
    if ((uint8_t)Enum_Flexcan_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
        handle->mbState[pMbXfer->mbIdx] = (uint8_t)Enum_Flexcan_StateRxData;

         /* Register Message Buffer. */
        handle->mbFrameBuf[pMbXfer->mbIdx] = pMbXfer->frame;

         /* Enable Message Buffer Interrupt. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        uint64_t u64mask = 1;

        FLEXCAN_EnableMbInterrupts(flex_can, u64mask << pMbXfer->mbIdx);
#else
        FLEXCAN_EnableMbInterrupts(flex_can, uint32_tmask << pMbXfer->mbIdx);
#endif

        status = Status_Flexcan_Success;
    }
    else
    {
        status = Status_Flexcan_RxBusy;
    }

    return (status);
}

/**
  * @brief Sends a message using IRQ.
  *
  * This function sends a message using IRQ. This is a non-blocking function, which returns
  * right away. When messages have been sent out, the send callback function is called.
  *
  * @param flex_can FlexCAN peripheral base address.
  * @param handle FlexCAN handle pointer.
  * @param pMbXfer FlexCAN FD Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
  * @retval kStatus_Success        Start Tx Message Buffer sending process successfully.
  * @retval kStatus_Fail           Write Tx Message Buffer failed.
  * @retval kStatus_FLEXCAN_TxBusy Tx Message Buffer is in use.
  */
int32_t FLEXCAN_TransferFDSendNonBlocking(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    int32_t status;

     /* Check if Message Buffer is idle. */
    if ((uint8_t)Enum_Flexcan_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
         /* Distinguish transmit type. */
        if ((uint32_t)Enum_Flexcan_FrameTypeRemote == pMbXfer->framefd->type)
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)Enum_Flexcan_StateTxRemote;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)Enum_Flexcan_StateTxData;
        }

        if (Status_Flexcan_Success == FLEXCAN_WriteFDTxMb(flex_can, pMbXfer->mbIdx, (const flexcan_fd_frame_t *)(uint32_t)pMbXfer->framefd))
        {
/* Enable Message Buffer Interrupt. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
            uint64_t u64mask = 1;

            FLEXCAN_EnableMbInterrupts(flex_can, u64mask << pMbXfer->mbIdx);
#else
            uint32_t uint32_tmask = 1;

            FLEXCAN_EnableMbInterrupts(flex_can, uint32_tmask << pMbXfer->mbIdx);
#endif
            status = Status_Flexcan_Success;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)Enum_Flexcan_StateIdle;
            status                          = Status_Flexcan_Fail;
        }
    }
    else
    {
        status = Status_Flexcan_TxBusy;
    }

    return (status);
}

/**
  * @brief Receives a message using IRQ.
  *
  * This function receives a message using IRQ. This is non-blocking function, which returns
  * right away. When the message has been received, the receive callback function is called.
  *
  * @param flex_can FlexCAN peripheral base address.
  * @param handle FlexCAN handle pointer.
  * @param pMbXfer FlexCAN FD Message Buffer transfer structure. See the #flexcan_mb_transfer_t.
  * @retval kStatus_Success        - Start Rx Message Buffer receiving process successfully.
  * @retval kStatus_FLEXCAN_RxBusy - Rx Message Buffer is in use.
  */
int32_t FLEXCAN_TransferFDReceiveNonBlocking(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    int32_t status;

     /* Check if Message Buffer is idle. */
    if ((uint8_t)Enum_Flexcan_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
        handle->mbState[pMbXfer->mbIdx] = (uint8_t)Enum_Flexcan_StateRxData;

         /* Register Message Buffer. */
        handle->mbFDFrameBuf[pMbXfer->mbIdx] = pMbXfer->framefd;

/* Enable Message Buffer Interrupt. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        uint64_t u64mask = 1;

        FLEXCAN_EnableMbInterrupts(flex_can, u64mask << pMbXfer->mbIdx);
#else
        uint32_t uint32_tmask = 1;

        FLEXCAN_EnableMbInterrupts(flex_can, uint32_tmask << pMbXfer->mbIdx);
#endif

        status = Status_Flexcan_Success;
    }
    else
    {
        status = Status_Flexcan_RxBusy;
    }

    return (status);
}

/**
  * @brief Receives a message from Rx FIFO using IRQ.
  *
  * This function receives a message using IRQ. This is a non-blocking function, which returns
  * right away. When all messages have been received, the receive callback function is called.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param handle FlexCAN handle pointer.
  * @param pFifoXfer FlexCAN Rx FIFO transfer structure. See the ref flexcan_fifo_transfer_t.
  * @retval Status_Flexcan_Success            - Start Rx FIFO receiving process successfully.
  * @retval Status_Flexcan_RxFifoBusy - Rx FIFO is currently in use.
  */
uint32_t FLEXCAN_TransferReceiveFifoNonBlocking(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_fifo_transfer_t *pFifoXfer)
{
    uint32_t status;

     /* Check if Message Buffer is idle. */
    if ((uint8_t)Enum_Flexcan_StateIdle == handle->rxFifoState)
    {
        handle->rxFifoState = (uint8_t)Enum_Flexcan_StateRxFifo;

         /* Register Message Buffer. */
        handle->rxFifoFrameBuf = pFifoXfer->frame;

         /* Enable Message Buffer Interrupt. */
        FLEXCAN_EnableMbInterrupts(flex_can, (uint32_t)Enum_Flexcan_RxFifoOverflowFlag | (uint32_t)Enum_Flexcan_RxFifoWarningFlag |
                                   (uint32_t)Enum_Flexcan_RxFifoFrameAvlFlag);

        status = Status_Flexcan_Success;
    }
    else
    {
        status = Status_Flexcan_RxFifoBusy;
    }

    return (status);
}

/**
  * @brief Aborts the interrupt driven message send process.
  *
  * This function aborts the interrupt driven message send process.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param handle FlexCAN handle pointer.
  * @param mbIdx The FlexCAN Message Buffer index.
  */
void FLEXCAN_TransferAbortSend(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, uint8_t mbIdx)
{
    uint16_t timestamp;
    uint32_t uint32_tmask = 1;

     /* Disable Message Buffer Interrupt. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint64_t u64mask = 1;

    FLEXCAN_DisableMbInterrupts(flex_can, u64mask << mbIdx);
#else
    FLEXCAN_DisableMbInterrupts(flex_can, uint32_tmask << mbIdx);
#endif

     /* Update the TX frame 's time stamp by MB[mbIdx].cs. */
    timestamp                = (uint16_t)((flex_can->MB[mbIdx].CS & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);
    handle->timestamp[mbIdx] = timestamp;

     /* Clean Message Buffer. */
    FLEXCAN_TxMbConfig(flex_can, mbIdx, ENABLE);

    handle->mbState[mbIdx] = (uint8_t)Enum_Flexcan_StateIdle;
}

/*!
  * @brief Aborts the interrupt driven message send process.
  *
  * This function aborts the interrupt driven message send process.
  *
  * @param flex_can FlexCAN peripheral base address.
  * @param handle FlexCAN handle pointer.
  * @param mbIdx The FlexCAN FD Message Buffer index.
  */
void FLEXCAN_TransferFDAbortSend(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, uint8_t mbIdx)
{
    volatile uint32_t *mbAddr;
    uint32_t offset;
    uint16_t timestamp;

/* Disable Message Buffer Interrupt. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint64_t u64mask = 1;

    FLEXCAN_DisableMbInterrupts(flex_can, u64mask << mbIdx);
#else
    uint32_t uint32_tmask = 1;

    FLEXCAN_DisableMbInterrupts(flex_can, uint32_tmask << mbIdx);
#endif

     /* Update the TX frame 's time stamp by flex_can->MB[offset for CANFD].CS. */
    mbAddr                   = &(flex_can->MB[0].CS);
    offset                   = FLEXCAN_GetFDMailboxOffset(flex_can, mbIdx);
    timestamp                = (uint16_t)((mbAddr[offset] & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);
    handle->timestamp[mbIdx] = timestamp;

     /* Clean Message Buffer. */
    FLEXCAN_FDTxMbConfig(flex_can, mbIdx, ENABLE);

    handle->mbState[mbIdx] = (uint8_t)Enum_Flexcan_StateIdle;
}

/**
  * @brief Aborts the interrupt driven message receive process.
  *
  * This function aborts the interrupt driven message receive process.
  *
  * @param flex_can FlexCAN peripheral base address.
  * @param handle FlexCAN handle pointer.
  * @param mbIdx The FlexCAN FD Message Buffer index.
  */
void FLEXCAN_TransferFDAbortReceive(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, uint8_t mbIdx)
{
/* Disable Message Buffer Interrupt. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint64_t u64mask = 1;

    FLEXCAN_DisableMbInterrupts(flex_can, u64mask << mbIdx);
#else
    uint32_t uint32_tmask = 1;

    FLEXCAN_DisableMbInterrupts(flex_can, uint32_tmask << mbIdx);
#endif

     /* Un-register handle. */
    handle->mbFDFrameBuf[mbIdx] = NULL;
    handle->mbState[mbIdx]      = (uint8_t)Enum_Flexcan_StateIdle;
}

/**
  * @brief Aborts the interrupt driven message receive process.
  *
  * This function aborts the interrupt driven message receive process.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param handle FlexCAN handle pointer.
  * @param mbIdx The FlexCAN Message Buffer index.
  */
void FLEXCAN_TransferAbortReceive(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, uint8_t mbIdx)
{
    uint32_t uint32_tmask = 1;

     /* Disable Message Buffer Interrupt. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint64_t u64mask = 1;

    FLEXCAN_DisableMbInterrupts(flex_can, (u64mask << mbIdx));
#else
    FLEXCAN_DisableMbInterrupts(flex_can, (uint32_tmask << mbIdx));
#endif

     /* Un-register handle. */
    handle->mbFrameBuf[mbIdx] = NULL;
    handle->mbState[mbIdx]    = (uint8_t)Enum_Flexcan_StateIdle;
}

/**
  * @brief Aborts the interrupt driven message receive from Rx FIFO process.
  *
  * This function aborts the interrupt driven message receive from Rx FIFO process.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param handle FlexCAN handle pointer.
  */
void FLEXCAN_TransferAbortReceiveFifo(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle)
{
     /* Check if Rx FIFO is enabled. */
    if (0U != (flex_can->MCR & CAN_MCR_RFEN_MASK))
    {
         /* Disable Rx Message FIFO Interrupts. */
        FLEXCAN_DisableMbInterrupts(flex_can, (uint32_t)Enum_Flexcan_RxFifoOverflowFlag | (uint32_t)Enum_Flexcan_RxFifoWarningFlag |
                                    (uint32_t)Enum_Flexcan_RxFifoFrameAvlFlag);

         /* Un-register handle. */
        handle->rxFifoFrameBuf = NULL;
    }

    handle->rxFifoState = (uint8_t)Enum_Flexcan_StateIdle;
}

/**
  * @brief Gets the detail index of Mailbox's Timestamp by handle.
  *
  * Then function can only be used when calling non-blocking Data transfer (TX/RX) API,
  * After TX/RX data transfer done (User can get the status by handler's callback function),
  * we can get the detail index of Mailbox's timestamp by handle,
  * Detail non-blocking data transfer API (TX/RX) contain.
  *   -FLEXCAN_TransferSendNonBlocking
  *   -FLEXCAN_TransferFDSendNonBlocking
  *   -FLEXCAN_TransferReceiveNonBlocking
  *   -FLEXCAN_TransferFDReceiveNonBlocking
  *   -FLEXCAN_TransferReceiveFifoNonBlocking
  *
  * @param handle FlexCAN handle pointer.
  * @param mbIdx The FlexCAN FD Message Buffer index.
  * @return the index of mailbox 's timestamp stored in the handle.
  */
uint32_t FLEXCAN_GetTimeStamp(flexcan_handle_t *handle, uint8_t mbIdx)
{
    return ((uint32_t)(handle->timestamp[mbIdx]));
}

static bool FLEXCAN_CheckUnhandleInterruptEvents(FLEXCAN_TypeDef *flex_can)
{
    uint64_t tempmask;
    uint64_t tempflag;
    bool fgRet = false;

     /* Checking exist error flag. */
    if (0U == (FLEXCAN_GetStatusFlags(flex_can) &
               ((uint32_t)Enum_Flexcan_TxWarningIntFlag | (uint32_t)Enum_Flexcan_RxWarningIntFlag |
                (uint32_t)Enum_Flexcan_BusOffIntFlag | (uint32_t)Enum_Flexcan_ErrorIntFlag | (uint32_t)Enum_Flexcan_WakeUpIntFlag)))
    {
        tempmask = (uint64_t)flex_can->IMASK1;
        tempflag = (uint64_t)flex_can->IFLAG1;

#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
         /* Checking whether exist MB interrupt status and legacy RX FIFO interrupt status. */
        tempmask |= ((uint64_t)flex_can->IMASK2) << 32;
        tempflag |= ((uint64_t)flex_can->IFLAG2) << 32;
#endif
        fgRet = (0U != (tempmask & tempflag));
    }
    else
    {
        fgRet = true;
    }

    return (fgRet);
}

static uint32_t FLEXCAN_SubHandlerForDataTransfered(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, uint32_t *pResult)
{
    uint32_t status  = Status_Flexcan_UnHandled;
    uint32_t result  = 0xFFU;
    uint32_t uint32_tflag = 1;

     /* For this implementation, we solve the Message with lowest MB index first. */
    for (result = 0U; result < (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can); result++)

    {
         /* Get the lowest unhandled Message Buffer */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        uint64_t u64flag = 1;

        if (0U != FLEXCAN_GetMbStatusFlags(flex_can, u64flag << result))
#else
        if (0U != FLEXCAN_GetMbStatusFlags(flex_can, uint32_tflag << result))
#endif
        {
            if (FLEXCAN_IsMbIntEnabled(flex_can, (uint8_t)result))
            {
                break;
            }
        }
    }

     /* find Message to deal with. */
    if (result < (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can))
    {
         /* Solve Legacy Rx FIFO interrupt. */
        if (((uint8_t)Enum_Flexcan_StateIdle != handle->rxFifoState) && (result <= (uint32_t)CAN_IFLAG1_BUF7I_SHIFT))
        {
            uint32_t uint32_tmask = 1;

            switch (uint32_tmask << result)
            {
                case Enum_Flexcan_RxFifoOverflowFlag:
                    status = Status_Flexcan_RxFifoOverflow;
                    break;

                case Enum_Flexcan_RxFifoWarningFlag:
                    status = Status_Flexcan_RxFifoWarning;
                    break;

                case Enum_Flexcan_RxFifoFrameAvlFlag:
                    status = FLEXCAN_ReadRxFifo(flex_can, handle->rxFifoFrameBuf);

                    if (Status_Flexcan_Success == status)
                    {
                         /* Align the current (index 0) rxfifo timestamp to the timestamp array by handle. */
                        handle->timestamp[0] = handle->rxFifoFrameBuf->timestamp;
                        status               = Status_Flexcan_RxFifoIdle;
                    }

                    FLEXCAN_TransferAbortReceiveFifo(flex_can, handle);
                    break;

                default:
                    status = Status_Flexcan_UnHandled;
                    break;
            }
        }
        else
        {
             /* Get current State of Message Buffer. */
            switch (handle->mbState[result])
            {
                /* Solve Rx Data Frame. */
                case (uint8_t)Enum_Flexcan_StateRxData:

                    if (0U != (flex_can->MCR & CAN_MCR_FDEN_MASK))
                    {
                        status = FLEXCAN_ReadFDRxMb(flex_can, (uint8_t)result, handle->mbFDFrameBuf[result]);

                        if (Status_Flexcan_Success == status)
                        {
                             /* Align the current index of RX MB timestamp to the timestamp array by handle. */
                            handle->timestamp[result] = handle->mbFDFrameBuf[result]->timestamp;
                            status                    = Status_Flexcan_RxIdle;
                        }
                    }
                    else
                    {
                        status = FLEXCAN_ReadRxMb(flex_can, (uint8_t)result, handle->mbFrameBuf[result]);

                        if (Status_Flexcan_Success == status)
                        {
                             /* Align the current index of RX MB timestamp to the timestamp array by handle. */
                            handle->timestamp[result] = handle->mbFrameBuf[result]->timestamp;
                            status                    = Status_Flexcan_RxIdle;
                        }
                    }

                    if (0U != (flex_can->MCR & CAN_MCR_FDEN_MASK))
                    {
                        FLEXCAN_TransferFDAbortReceive(flex_can, handle, (uint8_t)result);
                    }
                    else
                    {
                        FLEXCAN_TransferAbortReceive(flex_can, handle, (uint8_t)result);
                    }

                    break;

                /* Sove Rx Remote Frame.  User need to Read the frame in Mail box in time by Read from MB API. */
                case (uint8_t)Enum_Flexcan_StateRxRemote:
                    status = Status_Flexcan_RxRemote;

                    if (0U != (flex_can->MCR & CAN_MCR_FDEN_MASK))
                    {
                        FLEXCAN_TransferFDAbortReceive(flex_can, handle, (uint8_t)result);
                    }
                    else
                    {
                        FLEXCAN_TransferAbortReceive(flex_can, handle, (uint8_t)result);
                    }

                    break;

                /* Solve Tx Data Frame. */
                case (uint8_t)Enum_Flexcan_StateTxData:
                    status = Status_Flexcan_TxIdle;

                    if (0U != (flex_can->MCR & CAN_MCR_FDEN_MASK))
                    {
                        FLEXCAN_TransferFDAbortSend(flex_can, handle, (uint8_t)result);
                    }
                    else
                    {
                        FLEXCAN_TransferAbortSend(flex_can, handle, (uint8_t)result);
                    }

                    break;

                /* Solve Tx Remote Frame. */
                case (uint8_t)Enum_Flexcan_StateTxRemote:
                    handle->mbState[result] = (uint8_t)Enum_Flexcan_StateRxRemote;
                    status                  = Status_Flexcan_TxSwitchToRx;
                    {
                        FLEXCAN_TransferAbortSend(flex_can, handle, (uint8_t)result);
                    }
                    break;

                default:
                    status = Status_Flexcan_UnHandled;
                    break;
            }
        }

         /* Clear resolved Message Buffer IRQ. */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        uint64_t u64flag = 1;

        FLEXCAN_ClearMbStatusFlags(flex_can, u64flag << result);
#else
        uint32_t uint32_tflag = 1;

        FLEXCAN_ClearMbStatusFlags(flex_can, uint32_tflag << result);
#endif
    }

    *pResult = result;

    return (status);
}

/**
  * @brief FlexCAN IRQ handle function.
  *
  * This function handles the FlexCAN Error, the Message Buffer, and the Rx FIFO IRQ request.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param handle FlexCAN handle pointer.
  */
void FLEXCAN_TransferHandleIRQ(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle)
{
    uint32_t status;
    uint32_t result    = 0xFFU;
    uint32_t EsrStatus = 0U;

    do
    {
         /* Get Current FlexCAN Module Error and Status. */
        EsrStatus = FLEXCAN_GetStatusFlags(flex_can);

         /* To handle FlexCAN Error and Status Interrupt first. */
        if (0U != (EsrStatus & ((uint32_t)Enum_Flexcan_TxWarningIntFlag | (uint32_t)Enum_Flexcan_RxWarningIntFlag |
                                (uint32_t)Enum_Flexcan_BusOffIntFlag | (uint32_t)Enum_Flexcan_ErrorIntFlag)))
        {
            status = Status_Flexcan_ErrorStatus;
             /* Clear FlexCAN Error and Status Interrupt. */
            FLEXCAN_ClearStatusFlags(flex_can, (uint32_t)Enum_Flexcan_TxWarningIntFlag | (uint32_t)Enum_Flexcan_RxWarningIntFlag |
                                     (uint32_t)Enum_Flexcan_BusOffIntFlag | (uint32_t)Enum_Flexcan_ErrorIntFlag);
            result = EsrStatus;
        }
        else if (0U != (EsrStatus & (uint32_t)Enum_Flexcan_WakeUpIntFlag))
        {
            status = Status_Flexcan_WakeUp;
            FLEXCAN_ClearStatusFlags(flex_can, (uint32_t)Enum_Flexcan_WakeUpIntFlag);
        }
        else
        {
             /* to handle real data transfer. */
            status = FLEXCAN_SubHandlerForDataTransfered(flex_can, handle, &result);
        }

         /* Calling Callback Function if has one. */
        if (handle->callback != NULL)
        {
            handle->callback(flex_can, handle, status, result, handle->userData);
        }
    }
    while(FLEXCAN_CheckUnhandleInterruptEvents(flex_can));
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

