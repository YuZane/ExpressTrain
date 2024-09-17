/***********************************************************************************************************************
    @file     hal_flexcan.h
    @author   VV TEAM
    @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE FLEXCAN FIRMWARE LIBRARY.
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
#ifndef __HAL_FLEXCAN_H
#define __HAL_FLEXCAN_H

/* Files includes ----------------------------------------------------------------------------------------------------*/
#include "mm32_device.h"

/** @addtogroup MM32_StdPeriph_Driver
  * @{
  */
/** @defgroup FLEXCAN
  * @{
  */
/** @defgroup FLEXCAN_Exported_Types
  * @{
  */

#define IDEAL_SP_LOW    (750U)
#define IDEAL_SP_MID    (800U)
#define IDEAL_SP_HIGH   (875U)
#define IDEAL_SP_FACTOR (1000U)

/* FLEXCAN module features */

/**
 * @brief Message buffer size
 */
#define FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(x) (16)
/**
  * @brief Has doze mode support (register bit field MCR[DOZE]).
  */
#define FLEXCAN_HAS_DOZE_MODE_SUPPORT (0)
/**
  * @brief Insatnce has doze mode support (register bit field MCR[DOZE]).
  */
#define FLEXCAN_INSTANCE_HAS_DOZE_MODE_SUPPORTn(x) (0)
/**
  * @brief Has a glitch filter on the receive pin (register bit field MCR[WAKSRC]).
  */
#define FLEXCAN_HAS_GLITCH_FILTER (1)
/**
  * @brief Has extended interrupt mask and flag register (register IMASK2, IFLAG2).
  */
#define FLEXCAN_HAS_EXTENDED_FLAG_REGISTER (0)
/**
  * @brief Instance has extended bit timing register (register CBT).
  */
#define FLEXCAN_INSTANCE_HAS_EXTENDED_TIMING_REGISTERn(x) (1)
/**
  * @brief Has a receive FIFO DMA feature (register bit field MCR[DMA]).
  */
#define FLEXCAN_HAS_RX_FIFO_DMA (1)
/**
  * @brief Instance has a receive FIFO DMA feature (register bit field MCR[DMA]).
  */
#define FLEXCAN_INSTANCE_HAS_RX_FIFO_DMAn(x) (1)
/**
  * @brief Has separate message buffer 0 interrupt flag (register bit field IFLAG1[BUF0I]).
  */
#define FLEXCAN_HAS_SEPARATE_BUFFER_0_FLAG (1)
/**
  * @brief Has bitfield name BUF31TO0M.
  */
#define FLEXCAN_HAS_BUF31TO0M (1)
/**
  * @brief Number of interrupt vectors.
  */
#define FLEXCAN_INTERRUPT_COUNT (6)
/**
  * @brief Is affected by errata with ID 5641 (Module does not transmit a message that is enabled to be transmitted at a specific moment during the arbitration process).
  */
#define FLEXCAN_HAS_ERRATA_5641 (0)


#define MAX_PROPSEG           (CAN_CTRL1_PROPSEG_MASK >> CAN_CTRL1_PROPSEG_SHIFT)
#define MAX_PSEG1             (CAN_CTRL1_PSEG1_MASK >> CAN_CTRL1_PSEG1_SHIFT)
#define MAX_PSEG2             (CAN_CTRL1_PSEG2_MASK >> CAN_CTRL1_PSEG2_SHIFT)
#define MAX_RJW               (CAN_CTRL1_RJW_MASK >> CAN_CTRL1_RJW_SHIFT)
#define MAX_PRESDIV           (CAN_CTRL1_PRESDIV_MASK >> CAN_CTRL1_PRESDIV_SHIFT)
#define CTRL1_MAX_TIME_QUANTA (1U + MAX_PROPSEG + 1U + MAX_PSEG1 + 1U + MAX_PSEG2 + 1U)
#define CTRL1_MIN_TIME_QUANTA (8U)

#define MAX_EPROPSEG        (CAN_CBT_EPROPSEG_MASK >> CAN_CBT_EPROPSEG_SHIFT)
#define MAX_EPSEG1          (CAN_CBT_EPSEG1_MASK >> CAN_CBT_EPSEG1_SHIFT)
#define MAX_EPSEG2          (CAN_CBT_EPSEG2_MASK >> CAN_CBT_EPSEG2_SHIFT)
#define MAX_ERJW            (CAN_CBT_ERJW_MASK >> CAN_CBT_ERJW_SHIFT)
#define MAX_EPRESDIV        (CAN_CBT_EPRESDIV_MASK >> CAN_CBT_EPRESDIV_SHIFT)
#define CBT_MAX_TIME_QUANTA (1U + MAX_EPROPSEG + 1U + MAX_EPSEG1 + 1U + MAX_EPSEG2 + 1U)
#define CBT_MIN_TIME_QUANTA (8U)

#define MAX_FPROPSEG          (CAN_FDCBT_FPROPSEG_MASK >> CAN_FDCBT_FPROPSEG_SHIFT)
#define MAX_FPSEG1            (CAN_FDCBT_FPSEG1_MASK >> CAN_FDCBT_FPSEG1_SHIFT)
#define MAX_FPSEG2            (CAN_FDCBT_FPSEG2_MASK >> CAN_FDCBT_FPSEG2_SHIFT)
#define MAX_FRJW              (CAN_FDCBT_FRJW_MASK >> CAN_FDCBT_FRJW_SHIFT)
#define MAX_FPRESDIV          (CAN_FDCBT_FPRESDIV_MASK >> CAN_FDCBT_FPRESDIV_SHIFT)
#define FDCBT_MAX_TIME_QUANTA (1U + MAX_FPROPSEG + 0U + MAX_FPSEG1 + 1U + MAX_FPSEG2 + 1U)
#define FDCBT_MIN_TIME_QUANTA (5U)

#define MAX_CANFD_BAUDRATE (8000000U)
#define MAX_CAN_BAUDRATE   (1000000U)

#ifndef CAN_CLOCK_CHECK_NO_AFFECTS
/* If no define such MACRO, it mean that the CAN in current device have no clock affect issue. */
#define CAN_CLOCK_CHECK_NO_AFFECTS (true)
#endif


/**
  * @brief FlexCAN Frame ID helper macro.
  */
#define FLEXCAN_ID_STD(id) \
        (((uint32_t)(((uint32_t)(id)) << CAN_ID_STD_SHIFT))&CAN_ID_STD_MASK) /*!< Standard Frame ID helper macro. */
#define FLEXCAN_ID_EXT(id)                                    \
        (((uint32_t)(((uint32_t)(id)) << CAN_ID_EXT_SHIFT)) & \
         (CAN_ID_EXT_MASK | CAN_ID_STD_MASK))                                /*!< Extend Frame ID helper macro. */

/**
  * @brief FlexCAN Rx Message Buffer Mask helper macro.
  */
#define FLEXCAN_RX_MB_STD_MASK(id, rtr, ide)                                       \
        (((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
         FLEXCAN_ID_STD(id))           /*!< Standard Rx Message Buffer Mask helper macro. */
#define FLEXCAN_RX_MB_EXT_MASK(id, rtr, ide)                                       \
        (((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
         FLEXCAN_ID_EXT(id))           /*!< Extend Rx Message Buffer Mask helper macro. */

/**
  * @brief FlexCAN Rx FIFO Mask helper macro.
  */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_A(id, rtr, ide)                              \
        (((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
         (FLEXCAN_ID_STD(id) << 1))        /*!< Standard Rx FIFO Mask helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_HIGH(id, rtr, ide)                         \
        (((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
         (((uint32_t)(id) & 0x7FF) << 19)) /*!< Standard Rx FIFO Mask helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_LOW(id, rtr, ide)                          \
        (((uint32_t)((uint32_t)(rtr) << 15) | (uint32_t)((uint32_t)(ide) << 14)) | \
         (((uint32_t)(id) & 0x7FF) << 3))  /*!< Standard Rx FIFO Mask helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_HIGH(id) \
        (((uint32_t)(id) & 0x7F8) << 21)   /*!< Standard Rx FIFO Mask helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_HIGH(id) \
        (((uint32_t)(id) & 0x7F8) << 13)   /*!< Standard Rx FIFO Mask helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_LOW(id) \
        (((uint32_t)(id) & 0x7F8) << 5)    /*!< Standard Rx FIFO Mask helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_LOW(id) \
        (((uint32_t)(id) & 0x7F8) >> 3)    /*!< Standard Rx FIFO Mask helper macro Type C lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_A(id, rtr, ide)                              \
        (((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
         (FLEXCAN_ID_EXT(id) << 1))        /*!< Extend Rx FIFO Mask helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(id, rtr, ide)                            \
        (                                                                             \
            ((uint32_t)((uint32_t)(rtr) << 31) | (uint32_t)((uint32_t)(ide) << 30)) | \
            ((FLEXCAN_ID_EXT(id) & 0x1FFF8000)                                        \
                << 1))                 /*!< Extend Rx FIFO Mask helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(id, rtr, ide)                          \
        (((uint32_t)((uint32_t)(rtr) << 15) | (uint32_t)((uint32_t)(ide) << 14)) | \
         ((FLEXCAN_ID_EXT(id) & 0x1FFF8000) >>                                     \
          15))                                    /*!< Extend Rx FIFO Mask helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_HIGH(id) \
        ((FLEXCAN_ID_EXT(id) & 0x1FE00000) << 3)  /*!< Extend Rx FIFO Mask helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_HIGH(id) \
        ((FLEXCAN_ID_EXT(id) & 0x1FE00000) >>        \
         5)                                       /*!< Extend Rx FIFO Mask helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_LOW(id) \
        ((FLEXCAN_ID_EXT(id) & 0x1FE00000) >>       \
         13)                                      /*!< Extend Rx FIFO Mask helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_LOW(id) \
        ((FLEXCAN_ID_EXT(id) & 0x1FE00000) >> 21) /*!< Extend Rx FIFO Mask helper macro Type C lower part helper macro. */

/**
  * @brief FlexCAN Rx FIFO Filter helper macro.
  */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(id, rtr, ide) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_A(id, rtr, ide) /*!< Standard Rx FIFO Filter helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_B_HIGH(id, rtr, ide) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_HIGH(                \
            id, rtr, ide)                             /*!< Standard Rx FIFO Filter helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_B_LOW(id, rtr, ide) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_LOW(                \
            id, rtr, ide)                             /*!< Standard Rx FIFO Filter helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_HIGH(id) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_HIGH(      \
            id)                                       /*!< Standard Rx FIFO Filter helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_MID_HIGH(id) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_HIGH(      \
            id)                                       /*!< Standard Rx FIFO Filter helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_MID_LOW(id) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_LOW(      \
            id)                                       /*!< Standard Rx FIFO Filter helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_LOW(id) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_LOW(      \
            id)                                       /*!< Standard Rx FIFO Filter helper macro Type C lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_A(id, rtr, ide) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_A(id, rtr, ide) /*!< Extend Rx FIFO Filter helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_B_HIGH(id, rtr, ide) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(                \
            id, rtr, ide)                             /*!< Extend Rx FIFO Filter helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_B_LOW(id, rtr, ide) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(                \
            id, rtr, ide)                             /*!< Extend Rx FIFO Filter helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_HIGH(id) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_HIGH(      \
            id)                                       /*!< Extend Rx FIFO Filter helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_MID_HIGH(id) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_HIGH(      \
            id)                                       /*!< Extend Rx FIFO Filter helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_MID_LOW(id) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_LOW(      \
            id)                                       /*!< Extend Rx FIFO Filter helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_LOW(id) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_LOW(id)       /*!< Extend Rx FIFO Filter helper macro Type C lower part helper macro. */

/**
  * @brief FlexCAN transfer status.
  */
#define    Status_Flexcan_TxBusy                 0    /*!< Tx Message Buffer is Busy. */
#define    Status_Flexcan_TxIdle                 1    /*!< Tx Message Buffer is Idle. */
#define    Status_Flexcan_TxSwitchToRx           2    /*!< Remote Message is send out and Message buffer changed to Receive one. */
#define    Status_Flexcan_RxBusy                 3    /*!< Rx Message Buffer is Busy. */
#define    Status_Flexcan_RxIdle                 4    /*!< Rx Message Buffer is Idle. */
#define    Status_Flexcan_RxOverflow             5    /*!< Rx Message Buffer is Overflowed. */
#define    Status_Flexcan_RxFifoBusy             6    /*!< Rx Message FIFO is Busy. */
#define    Status_Flexcan_RxFifoIdle             7    /*!< Rx Message FIFO is Idle. */
#define    Status_Flexcan_RxFifoOverflow         8    /*!< Rx Message FIFO is overflowed. */
#define    Status_Flexcan_RxFifoWarning          9    /*!< Rx Message FIFO is almost overflowed. */
#define    Status_Flexcan_ErrorStatus            10   /*!< FlexCAN Module Error and Status. */
#define    Status_Flexcan_WakeUp                 11   /*!< FlexCAN is waken up from STOP mode. */
#define    Status_Flexcan_UnHandled              12   /*!< UnHadled Interrupt asserted. */
#define    Status_Flexcan_RxRemote               13   /*!< Rx Remote Message Received in Mail box. */

#define    Status_Flexcan_Success                0    /*!< Generic status for Success. */
#define    Status_Flexcan_Fail                   1    /*!< Generic status for Fail. */
#define    Status_Flexcan_ReadOnly               2    /*!< Generic status for read only failure. */
#define    Status_Flexcan_OutOfRange             3    /*!< Generic status for out of range access. */
#define    Status_Flexcan_InvalidArgument        4    /*!< Generic status for invalid argument check. */
#define    Status_Flexcan_Timeout                5    /*!< Generic status for timeout. */
#define    Status_Flexcan_NoTransferInProgress   6    /*!< Generic status for no transfer in progress. */

enum _flexcan_state
{
    Enum_Flexcan_StateIdle     = 0x0,  /*!< MB/RxFIFO idle. */
    Enum_Flexcan_StateRxData   = 0x1,  /*!< MB receiving. */
    Enum_Flexcan_StateRxRemote = 0x2,  /*!< MB receiving remote reply. */
    Enum_Flexcan_StateTxData   = 0x3,  /*!< MB transmitting. */
    Enum_Flexcan_StateTxRemote = 0x4,  /*!< MB transmitting remote request. */
    Enum_Flexcan_StateRxFifo   = 0x5   /*!< RxFIFO receiving. */
};

/**
  * @brief FlexCAN Message Buffer Payload size.
  */
typedef enum _flexcan_mb_size
{
    FLEXCAN_8BperMB  = 0x0U,           /*!< Selects 8 bytes per Message Buffer. */
    FLEXCAN_16BperMB = 0x1U,           /*!< Selects 16 bytes per Message Buffer. */
    FLEXCAN_32BperMB = 0x2U,           /*!< Selects 32 bytes per Message Buffer. */
    FLEXCAN_64BperMB = 0x3U            /*!< Selects 64 bytes per Message Buffer. */
} flexcan_mb_size_t;

/**
  * @brief FlexCAN FD Message Buffer Size.
  */
typedef enum
{
    FlexCANFD_TX_0Byte_DataLen   = 0,  /* FlexCANFD sends 0 byte of data length per frame */
    FlexCANFD_TX_1Byte_DataLen   = 1,  /* FlexCANFD sends 1 byte of data length per frame */
    FlexCANFD_TX_2Bytes_DataLen  = 2,  /* FlexCANFD sends 2 bytes of data length per frame */
    FlexCANFD_TX_3Bytes_DataLen  = 3,  /* FlexCANFD sends 3 bytes of data length per frame */
    FlexCANFD_TX_4Bytes_DataLen  = 4,  /* FlexCANFD sends 4 bytes of data length per frame */
    FlexCANFD_TX_5Bytes_DataLen  = 5,  /* FlexCANFD sends 5 bytes of data length per frame */
    FlexCANFD_TX_6Bytes_DataLen  = 6,  /* FlexCANFD sends 6 bytes of data length per frame */
    FlexCANFD_TX_7Bytes_DataLen  = 7,  /* FlexCANFD sends 7 bytes of data length per frame */
    FlexCANFD_TX_8Bytes_DataLen  = 8,  /* FlexCANFD sends 8 bytes of data length per frame */
    FlexCANFD_TX_12Bytes_DataLen = 9,  /* FlexCANFD sends 12 bytes of data length per frame */
    FlexCANFD_TX_16Bytes_DataLen = 10, /* FlexCANFD sends 16 bytes of data length per frame */
    FlexCANFD_TX_20Bytes_DataLen = 11, /* FlexCANFD sends 20 bytes of data length per frame */
    FlexCANFD_TX_24Bytes_DataLen = 12, /* FlexCANFD sends 24 bytes of data length per frame */
    FlexCANFD_TX_32Bytes_DataLen = 13, /* FlexCANFD sends 32 bytes of data length per frame */
    FlexCANFD_TX_48Bytes_DataLen = 14, /* FlexCANFD sends 48 bytes of data length per frame */
    FlexCANFD_TX_64Bytes_DataLen = 15  /* FlexCANFD sends 64 bytes of data length per frame */
} FLEXCANFD_Tx_DataLen_Type;

/**
  * @brief FlexCAN message buffer CODE for Rx buffers.
  */
enum _flexcan_mb_code_rx
{
    Enum_Flexcan_RxMbInactive = 0x0,   /*!< MB is not active. */
    Enum_Flexcan_RxMbFull     = 0x2,   /*!< MB is full. */
    Enum_Flexcan_RxMbEmpty    = 0x4,   /*!< MB is active and empty. */
    Enum_Flexcan_RxMbOverrun  = 0x6,   /*!< MB is overwritten into a full buffer. */
    Enum_Flexcan_RxMbBusy     = 0x8,   /*!< FlexCAN is updating the contents of the MB. The CPU must not access the MB. */
    Enum_Flexcan_RxMbRanswer  = 0xA,   /*!< A frame was configured to recognize a Remote Request Frame and transmit a Response Frame in return. */
    Enum_Flexcan_RxMbNotUsed  = 0xF    /*!< Not used. */
};

/**
  * @brief FlexCAN message buffer CODE FOR Tx buffers.
  */
enum _flexcan_mb_code_tx
{
    Enum_Flexcan_TxMbInactive     = 0x8, /*!< MB is not active. */
    Enum_Flexcan_TxMbAbort        = 0x9, /*!< MB is aborted. */
    Enum_Flexcan_TxMbDataOrRemote = 0xC, /*!< MB is a TX Data Frame(when MB RTR = 0) or MB is a TX Remote Request Frame (when MB RTR = 1). */
    Enum_Flexcan_TxMbTanswer      = 0xE, /*!< MB is a TX Response Request Frame from an incoming Remote Request Frame. */
    Enum_Flexcan_TxMbNotUsed      = 0xF  /*!< Not used. */
};

/**
  * @brief FlexCAN frame format.
  */
typedef enum _flexcan_frame_format
{
    Enum_Flexcan_FrameFormatStandard = 0x0U, /*!< Standard frame format attribute. */
    Enum_Flexcan_FrameFormatExtend   = 0x1U  /*!< Extend frame format attribute. */
} flexcan_frame_format_t;

/**
  * @brief FlexCAN frame type.
  */
typedef enum _flexcan_frame_type
{
    Enum_Flexcan_FrameTypeData   = 0x0U, /*!< Data frame type attribute. */
    Enum_Flexcan_FrameTypeRemote = 0x1U  /*!< Remote frame type attribute. */
} flexcan_frame_type_t;

/**
  * @brief FlexCAN clock source.
  * @deprecated Do not use the Enum_Flexcan_ClkSrcOs.  It has been superceded Enum_Flexcan_ClkSrc0
  * @deprecated Do not use the Enum_Flexcan_ClkSrcPeri.  It has been superceded Enum_Flexcan_ClkSrc1
  * @{
  */
typedef enum _flexcan_clock_source
{
    Enum_Flexcan_ClkSrcOsc  = 0x0U,    /*!< FlexCAN Protocol Engine clock from Oscillator. */
    Enum_Flexcan_ClkSrcPeri = 0x1U,    /*!< FlexCAN Protocol Engine clock from Peripheral Clock. */
    Enum_Flexcan_ClkSrc0    = 0x0U,    /*!< FlexCAN Protocol Engine clock selected by user as SRC == 0. */
    Enum_Flexcan_ClkSrc1    = 0x1U     /*!< FlexCAN Protocol Engine clock selected by user as SRC == 1. */
} flexcan_clock_source_t;

/**
  * @brief FlexCAN wake up source.
  */
typedef enum _flexcan_wake_up_source
{
    Enum_Flexcan_WakeupSrcUnfiltered = 0x0U, /*!< FlexCAN uses unfiltered Rx input to detect edge. */
    Enum_Flexcan_WakeupSrcFiltered   = 0x1U  /*!< FlexCAN uses filtered Rx input to detect edge. */
} flexcan_wake_up_source_t;

/**
  * @brief FlexCAN Rx Fifo Filter type.
  */
typedef enum _flexcan_rx_fifo_filter_type
{
    Enum_Flexcan_RxFifoFilterTypeA = 0x0U, /*!< One full ID (standard and extended) per ID Filter element. */
    Enum_Flexcan_RxFifoFilterTypeB = 0x1U, /*!< Two full standard IDs or two partial 14-bit ID slices per ID Filter Table element. */
    Enum_Flexcan_RxFifoFilterTypeC = 0x2U, /*!< Four partial 8-bit Standard or extended ID slices per ID Filter Table element. */
    Enum_Flexcan_RxFifoFilterTypeD = 0x3U  /*!< All frames rejected. */
} flexcan_rx_fifo_filter_type_t;

/**
  * @brief  FlexCAN Rx FIFO priority.
  *
  * The matching process starts from the Rx MB(or Rx FIFO) with higher priority.
  * If no MB(or Rx FIFO filter) is satisfied, the matching process goes on with
  * the Rx FIFO(or Rx MB) with lower priority.
  */
typedef enum _flexcan_rx_fifo_priority
{
    Enum_Flexcan_RxFifoPrioLow  = 0x0U, /*!< Matching process start from Rx Message Buffer first */
    Enum_Flexcan_RxFifoPrioHigh = 0x1U  /*!< Matching process start from Rx FIFO first */
} flexcan_rx_fifo_priority_t;

/**
  * @brief  FlexCAN interrupt configuration structure, default settings all disabled.
  *
  * This structure contains the settings for all of the FlexCAN Module interrupt configurations.
  * Note: FlexCAN Message Buffers and Rx FIFO have their own interrupts.
  */
enum _flexcan_interrupt_enable
{
    Enum_Flexcan_BusOffInterruptEnable    = CAN_CTRL1_BOFFMSK_MASK, /*!< Bus Off interrupt. */
    Enum_Flexcan_ErrorInterruptEnable     = CAN_CTRL1_ERRMSK_MASK,  /*!< Error interrupt. */
    Enum_Flexcan_RxWarningInterruptEnable = CAN_CTRL1_RWRNMSK_MASK, /*!< Rx Warning interrupt. */
    Enum_Flexcan_TxWarningInterruptEnable = CAN_CTRL1_TWRNMSK_MASK, /*!< Tx Warning interrupt. */
    Enum_Flexcan_WakeUpInterruptEnable    = CAN_MCR_WAKMSK_MASK     /*!< Wake Up interrupt. */
};

/**
  * @brief  FlexCAN status flags.
  *
  * This provides constants for the FlexCAN status flags for use in the FlexCAN functions.
  * Note: The CPU read action clears FlEXCAN_ErrorFlag, therefore user need to
  * read FlEXCAN_ErrorFlag and distinguish which error is occur using
  * @ref _flexcan_error_flags enumerations.
  */
enum _flexcan_flags
{
    Enum_Flexcan_SynchFlag            = CAN_ESR1_SYNCH_MASK,   /*!< CAN Synchronization Status. */
    Enum_Flexcan_TxWarningIntFlag     = CAN_ESR1_TWRNINT_MASK, /*!< Tx Warning Interrupt Flag. */
    Enum_Flexcan_RxWarningIntFlag     = CAN_ESR1_RWRNINT_MASK, /*!< Rx Warning Interrupt Flag. */
    Enum_Flexcan_TxErrorWarningFlag   = CAN_ESR1_TXWRN_MASK,   /*!< Tx Error Warning Status. */
    Enum_Flexcan_RxErrorWarningFlag   = CAN_ESR1_RXWRN_MASK,   /*!< Rx Error Warning Status. */
    Enum_Flexcan_IdleFlag             = CAN_ESR1_IDLE_MASK,    /*!< CAN IDLE Status Flag. */
    Enum_Flexcan_FaultConfinementFlag = CAN_ESR1_FLTCONF_MASK, /*!< Fault Confinement State Flag. */
    Enum_Flexcan_TransmittingFlag     = CAN_ESR1_TX_MASK,      /*!< FlexCAN In Transmission Status. */
    Enum_Flexcan_ReceivingFlag        = CAN_ESR1_RX_MASK,      /*!< FlexCAN In Reception Status. */
    Enum_Flexcan_BusOffIntFlag        = CAN_ESR1_BOFFINT_MASK, /*!< Bus Off Interrupt Flag. */
    Enum_Flexcan_ErrorIntFlag         = CAN_ESR1_ERRINT_MASK,  /*!< Error Interrupt Flag. */
    Enum_Flexcan_WakeUpIntFlag        = CAN_ESR1_WAKINT_MASK,  /*!< Wake-Up Interrupt Flag. */
    Enum_Flexcan_ErrorFlag            = (int)(                 /*!< All FlexCAN Error Status. */
        CAN_ESR1_BIT1ERR_MASK | CAN_ESR1_BIT0ERR_MASK | CAN_ESR1_ACKERR_MASK |
        CAN_ESR1_CRCERR_MASK | CAN_ESR1_FRMERR_MASK | CAN_ESR1_STFERR_MASK)
};

/**
  * @brief  FlexCAN error status flags.
  *
  * The FlexCAN Error Status enumerations is used to report current error of the FlexCAN bus.
  * This enumerations should be used with Enum_Flexcan_ErrorFlag in @ref _flexcan_flags enumerations
  * to ditermine which error is generated.
  */
enum _flexcan_error_flags
{
    Enum_Flexcan_StuffingError = CAN_ESR1_STFERR_MASK,  /*!< Stuffing Error. */
    Enum_Flexcan_FormError     = CAN_ESR1_FRMERR_MASK,  /*!< Form Error. */
    Enum_Flexcan_CrcError      = CAN_ESR1_CRCERR_MASK,  /*!< Cyclic Redundancy Check Error. */
    Enum_Flexcan_AckError      = CAN_ESR1_ACKERR_MASK,  /*!< Received no ACK on transmission. */
    Enum_Flexcan_Bit0Error     = CAN_ESR1_BIT0ERR_MASK, /*!< Unable to send dominant bit. */
    Enum_Flexcan_Bit1Error     = CAN_ESR1_BIT1ERR_MASK  /*!< Unable to send recessive bit. */
};

/**
  * @brief  FlexCAN Rx FIFO status flags.
  *
  * The FlexCAN Rx FIFO Status enumerations are used to determine the status of the
  * Rx FIFO. Because Rx FIFO occupy the MB0 ~ MB7 (Rx Fifo filter also occupies
  * more Message Buffer space), Rx FIFO status flags are mapped to the corresponding
  * Message Buffer status flags.
  */
enum
{
    Enum_Flexcan_RxFifoOverflowFlag = CAN_IFLAG1_BUF7I_MASK, /*!< Rx FIFO overflow flag. */
    Enum_Flexcan_RxFifoWarningFlag  = CAN_IFLAG1_BUF6I_MASK, /*!< Rx FIFO almost full flag. */
    Enum_Flexcan_RxFifoFrameAvlFlag = CAN_IFLAG1_BUF5I_MASK  /*!< Frames available in Rx FIFO flag. */
};

/**
  * @brief  Construct a status code value from a group and code number.
  */
#define MAKE_STATUS(group, code) ((((group) * 100) + (code)))

/**
  * @brief  Status group numbers.
  */
enum _status_groups
{
    StatusGroup_Generic = 0,           /*!< Group number for generic status codes. */
    StatusGroup_FLEXCAN = 53
};

#if defined (__CC_ARM)
#pragma anon_unions
#endif

/**
  * @brief FlexCAN message frame structure.
  */
typedef struct _flexcan_frame
{
    struct
    {
        uint32_t timestamp : 16;       /*!< FlexCAN internal Free-Running Counter Time Stamp. */
        uint32_t length    : 4;        /*!< CAN frame payload length in bytes(Range: 0~8). */
        uint32_t type      : 1;        /*!< CAN Frame Type(DATA or REMOTE). */
        uint32_t format    : 1;        /*!< CAN Frame Identifier(STD or EXT format). */

        uint32_t           : 1;        /*!< Reserved. */

        uint32_t idhit     : 9;        /*!< CAN Rx FIFO filter hit id(This value is only used in Rx FIFO receive mode). */
    };
    struct
    {
        uint32_t id : 29;              /*!< CAN Frame Identifier, should be set using FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */

        uint32_t    : 3;               /*!< Reserved. */
    };
    union
    {
        struct
        {
            uint32_t dataWord0;        /*!< CAN Frame payload word0. */
            uint32_t dataWord1;        /*!< CAN Frame payload word1. */
        };
        struct
        {
            uint8_t dataByte3;         /*!< CAN Frame payload byte3. */
            uint8_t dataByte2;         /*!< CAN Frame payload byte2. */
            uint8_t dataByte1;         /*!< CAN Frame payload byte1. */
            uint8_t dataByte0;         /*!< CAN Frame payload byte0. */
            uint8_t dataByte7;         /*!< CAN Frame payload byte7. */
            uint8_t dataByte6;         /*!< CAN Frame payload byte6. */
            uint8_t dataByte5;         /*!< CAN Frame payload byte5. */
            uint8_t dataByte4;         /*!< CAN Frame payload byte4. */
        };
    };
} flexcan_frame_t;

/**
  * @brief FlexCAN FD message frame structure.
  */
typedef struct _flexcan_fd_frame
{
    struct
    {
        uint32_t timestamp  : 16;      /*!< FlexCAN internal Free-Running Counter Time Stamp. */
        uint32_t length     : 4;       /*!< CAN frame data length in bytes, range see @ref _flexcan_fd_frame_length. */
        uint32_t type       : 1;       /*!< CAN Frame Type(DATA or REMOTE). */
        uint32_t format     : 1;       /*!< CAN Frame Identifier(STD or EXT format). */
        uint32_t srr        : 1;       /*!< Substitute Remote request. */
        uint32_t RESERVED_0 : 1;
        uint32_t code       : 4;       /*!< Message Buffer Code. */
        uint32_t RESERVED_1 : 1;
        uint32_t esi        : 1;       /*!< Error State Indicator. */
        uint32_t brs        : 1;       /*!< Bit Rate Switch. */
        uint32_t edl        : 1;       /*!< Extended Data Length. */
    };
    struct
    {
        uint32_t id         : 29;      /*!< CAN Frame Identifier, should be set using FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */
        uint32_t RESERVED_3 : 3;       /*!< Reserved. */
    };
    union
    {
        struct
        {
            uint32_t dataWord[16];     /*!< CAN FD Frame payload, 16 double word maximum. */
        };
        struct
        {
            uint8_t dataByte3;         /*!< CAN Frame payload byte3. */
            uint8_t dataByte2;         /*!< CAN Frame payload byte2. */
            uint8_t dataByte1;         /*!< CAN Frame payload byte1. */
            uint8_t dataByte0;         /*!< CAN Frame payload byte0. */
            uint8_t dataByte7;         /*!< CAN Frame payload byte7. */
            uint8_t dataByte6;         /*!< CAN Frame payload byte6. */
            uint8_t dataByte5;         /*!< CAN Frame payload byte5. */
            uint8_t dataByte4;         /*!< CAN Frame payload byte4. */
            uint8_t dataByte11;        /*!< CAN Frame payload byte11. */
            uint8_t dataByte10;        /*!< CAN Frame payload byte10. */
            uint8_t dataByte9;         /*!< CAN Frame payload byte9. */
            uint8_t dataByte8;         /*!< CAN Frame payload byte8. */
            uint8_t dataByte15;        /*!< CAN Frame payload byte15. */
            uint8_t dataByte14;        /*!< CAN Frame payload byte14. */
            uint8_t dataByte13;        /*!< CAN Frame payload byte13. */
            uint8_t dataByte12;        /*!< CAN Frame payload byte12. */
            uint8_t dataByte19;        /*!< CAN Frame payload byte19. */
            uint8_t dataByte18;        /*!< CAN Frame payload byte18. */
            uint8_t dataByte17;        /*!< CAN Frame payload byte17. */
            uint8_t dataByte16;        /*!< CAN Frame payload byte16. */
            uint8_t dataByte23;        /*!< CAN Frame payload byte23. */
            uint8_t dataByte22;        /*!< CAN Frame payload byte22. */
            uint8_t dataByte21;        /*!< CAN Frame payload byte21. */
            uint8_t dataByte20;        /*!< CAN Frame payload byte20. */
            uint8_t dataByte27;        /*!< CAN Frame payload byte27. */
            uint8_t dataByte26;        /*!< CAN Frame payload byte26. */
            uint8_t dataByte25;        /*!< CAN Frame payload byte25. */
            uint8_t dataByte24;        /*!< CAN Frame payload byte24. */
            uint8_t dataByte31;        /*!< CAN Frame payload byte31. */
            uint8_t dataByte30;        /*!< CAN Frame payload byte30. */
            uint8_t dataByte29;        /*!< CAN Frame payload byte29. */
            uint8_t dataByte28;        /*!< CAN Frame payload byte28. */
            uint8_t dataByte35;        /*!< CAN Frame payload byte35. */
            uint8_t dataByte34;        /*!< CAN Frame payload byte34. */
            uint8_t dataByte33;        /*!< CAN Frame payload byte33. */
            uint8_t dataByte32;        /*!< CAN Frame payload byte32. */
            uint8_t dataByte39;        /*!< CAN Frame payload byte39. */
            uint8_t dataByte38;        /*!< CAN Frame payload byte38. */
            uint8_t dataByte37;        /*!< CAN Frame payload byte37. */
            uint8_t dataByte36;        /*!< CAN Frame payload byte36. */
            uint8_t dataByte43;        /*!< CAN Frame payload byte43. */
            uint8_t dataByte42;        /*!< CAN Frame payload byte42. */
            uint8_t dataByte41;        /*!< CAN Frame payload byte41. */
            uint8_t dataByte40;        /*!< CAN Frame payload byte40. */
            uint8_t dataByte47;        /*!< CAN Frame payload byte47. */
            uint8_t dataByte46;        /*!< CAN Frame payload byte46. */
            uint8_t dataByte45;        /*!< CAN Frame payload byte45. */
            uint8_t dataByte44;        /*!< CAN Frame payload byte44. */
            uint8_t dataByte51;        /*!< CAN Frame payload byte51. */
            uint8_t dataByte50;        /*!< CAN Frame payload byte50. */
            uint8_t dataByte49;        /*!< CAN Frame payload byte49. */
            uint8_t dataByte48;        /*!< CAN Frame payload byte48. */
            uint8_t dataByte55;        /*!< CAN Frame payload byte55. */
            uint8_t dataByte54;        /*!< CAN Frame payload byte54. */
            uint8_t dataByte53;        /*!< CAN Frame payload byte53. */
            uint8_t dataByte52;        /*!< CAN Frame payload byte52. */
            uint8_t dataByte59;        /*!< CAN Frame payload byte59. */
            uint8_t dataByte58;        /*!< CAN Frame payload byte58. */
            uint8_t dataByte57;        /*!< CAN Frame payload byte57. */
            uint8_t dataByte56;        /*!< CAN Frame payload byte56. */
            uint8_t dataByte63;        /*!< CAN Frame payload byte63. */
            uint8_t dataByte62;        /*!< CAN Frame payload byte62. */
            uint8_t dataByte61;        /*!< CAN Frame payload byte61. */
            uint8_t dataByte60;        /*!< CAN Frame payload byte60. */
        };
    };
} flexcan_fd_frame_t;

/**
  * @brief FlexCAN protocol timing characteristic configuration structure.
  */
typedef struct _flexcan_timing_config
{
    uint16_t      preDivider;          /*!< Clock Pre-scaler Division Factor. */
    uint8_t       rJumpwidth;          /*!< Re-sync Jump Width. */
    uint8_t       phaseSeg1;           /*!< Phase Segment 1. */
    uint8_t       phaseSeg2;           /*!< Phase Segment 2. */
    uint8_t       propSeg;             /*!< Propagation Segment. */

     /*!< FlexCAN has flexible data rate. */
    uint16_t      fpreDivider;         /*!< Fast Clock Pre-scaler Division Factor. */
    uint8_t       frJumpwidth;         /*!< Fast Re-sync Jump Width. */
    uint8_t       fphaseSeg1;          /*!< Fast Phase Segment 1. */
    uint8_t       fphaseSeg2;          /*!< Fast Phase Segment 2. */
    uint8_t       fpropSeg;            /*!< Fast Propagation Segment. */
} flexcan_timing_config_t;

/**
  * @brief FlexCAN module configuration structure.
  */
typedef struct _flexcan_config
{
    uint32_t                 baudRate;             /*!< FlexCAN baud rate in bps. */

     /*!< FlexCAN has flexible data rate. */
    uint32_t                 baudRateFD;           /*!< FlexCAN FD baud rate in bps. */
    flexcan_clock_source_t   clkSrc;               /*!< Clock source for FlexCAN Protocol Engine. */
    flexcan_wake_up_source_t wakeupSrc;            /*!< Wake up source selection. */
    uint8_t                  maxMbNum;             /*!< The maximum number of Message Buffers used by user. */
    bool                     enableLoopBack;       /*!< Enable or Disable Loop Back Self Test Mode. */
    bool                     enableTimerSync;      /*!< Enable or Disable Timer Synchronization. */
    bool                     enableSelfWakeup;     /*!< Enable or Disable Self Wakeup Mode. */
    bool                     enableIndividMask;    /*!< Enable or Disable Rx Individual Mask. */
    bool                     disableSelfReception; /*!< Enable or Disable Self Reflection. */
    bool                     enableListenOnlyMode; /*!< Enable or Disable Listen Only Mode. */

#if (defined (FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    bool                     enableDoze;           /*!< Enable or Disable Doze Mode. */
#endif
    flexcan_timing_config_t  timingConfig;         /*!< Protocol timing. */
} flexcan_config_t;

/**
  * @brief  FlexCAN Receive Message Buffer configuration structure
  *
  * This structure is used as the parameter of FLEXCAN_RxMbConfig() function.
  * The FLEXCAN_RxMbConfig() function is used to configure FlexCAN Receive
  * Message Buffer. The function abort previous receiving process, clean the
  * Message Buffer and activate the Rx Message Buffer using given Message Buffer
  * setting.
  */
typedef struct _flexcan_rx_mb_config
{
    uint32_t               id;         /*!< CAN Message Buffer Frame Identifier, should be set using FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */
    flexcan_frame_format_t format;     /*!< CAN Frame Identifier format(Standard of Extend). */
    flexcan_frame_type_t   type;       /*!< CAN Frame Type(Data or Remote). */
} flexcan_rx_mb_config_t;

/**
  * @brief FlexCAN Rx FIFO configuration structure.
  */
typedef struct _flexcan_rx_fifo_config
{
    uint32_t                     *idFilterTable; /*!< Pointer to the FlexCAN Rx FIFO identifier filter table. */
    uint8_t                       idFilterNum;   /*!< The quantity of filter elements. */
    flexcan_rx_fifo_filter_type_t idFilterType;  /*!< The FlexCAN Rx FIFO Filter type. */
    flexcan_rx_fifo_priority_t    priority;      /*!< The FlexCAN Rx FIFO receive priority. */
} flexcan_rx_fifo_config_t;

/**
  * @brief FlexCAN Message Buffer transfer.
  */
typedef struct _flexcan_mb_transfer
{
    flexcan_fd_frame_t      *framefd;  /*!< The buffer of CAN FD Message to be transfer. */
    flexcan_frame_t         *frame;    /*!< The buffer of CAN Message to be transfer. */
    uint8_t                  mbIdx;    /*!< The index of Message buffer used to transfer Message. */
} flexcan_mb_transfer_t;

/**
  * @brief FlexCAN Rx FIFO transfer.
  */
typedef struct _flexcan_fifo_transfer
{
    flexcan_frame_t *frame;            /*!< The buffer of CAN Message to be received from Rx FIFO. */
} flexcan_fifo_transfer_t;

/**
  * @brief FlexCAN handle structure definition.
  */
typedef struct _flexcan_handle flexcan_handle_t;

/**
  * @brief FlexCAN transfer callback function.
  *
  * The FlexCAN transfer callback returns a value from the underlying layer.
  * If the status equals to Status_Flexcan_ErrorStatus, the result parameter is the Content of
  * FlexCAN status register which can be used to get the working status(or error status) of FlexCAN module.
  * If the status equals to other FlexCAN Message Buffer transfer status, the result is the index of
  * Message Buffer that generate transfer event.
  * If the status equals to other FlexCAN Message Buffer transfer status, the result is meaningless and should be
  * Ignored.
  */

typedef void (*flexcan_transfer_callback_t)(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, uint32_t status, uint32_t result, void *userData);

/**
  * @brief FlexCAN handle structure.
  */
struct _flexcan_handle
{
    flexcan_transfer_callback_t       callback; /*!< Callback function. */
    void                             *userData; /*!< FlexCAN callback function parameter. */
    flexcan_frame_t *volatile         mbFrameBuf[CAN_WORD1_COUNT];
    flexcan_fd_frame_t *volatile      mbFDFrameBuf[CAN_WORD1_COUNT];

     /*!< The buffer for received data from Message Buffers. ------------------*/
    flexcan_frame_t *volatile         rxFifoFrameBuf;             /*!< The buffer for received data from Rx FIFO. */
    volatile uint8_t                  mbState[CAN_WORD1_COUNT];   /*!< Message Buffer transfer state. */
    volatile uint8_t                  rxFifoState;                /*!< Rx FIFO transfer state. */
    volatile uint32_t                 timestamp[CAN_WORD1_COUNT]; /*!< Mailbox transfer timestamp. */
};

/**
  * @}
  */

/** @defgroup ADC_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @defgroup ADC_Exported_Functions
  * @{
  */

/**
  * @brief  Clears status flags with the provided mask.
  *
  * This function clears the FlexCAN status flags with a provided mask. An automatically cleared flag
  * can't be cleared by this function.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mask The status flags to be cleared, it is logical OR value of @ref _flexcan_flags.
  */
__STATIC_INLINE void FLEXCAN_ClearStatusFlags(FLEXCAN_TypeDef *flex_can, uint32_t mask)
{
     /* Write 1 to clear status flag. -----------------------------------------*/
    flex_can->ESR1 = mask;
}

/**
  * @brief  Gets the FlexCAN Bus Error Counter value.
  *
  * This function gets the FlexCAN Bus Error Counter value for both Tx and
  * Rx direction. These values may be needed in the upper layer error handling.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param txErrBuf Buffer to store Tx Error Counter value.
  * @param rxErrBuf Buffer to store Rx Error Counter value.
  */
__STATIC_INLINE void FLEXCAN_GetBusErrCount(FLEXCAN_TypeDef *flex_can, uint8_t *txErrBuf, uint8_t *rxErrBuf)
{
    if (NULL != txErrBuf)
    {
        *txErrBuf = (uint8_t)((flex_can->ECR & CAN_ECR_TXERRCNT_MASK) >> CAN_ECR_TXERRCNT_SHIFT);
    }

    if (NULL != rxErrBuf)
    {
        *rxErrBuf = (uint8_t)((flex_can->ECR & CAN_ECR_RXERRCNT_MASK) >> CAN_ECR_RXERRCNT_SHIFT);
    }
}

/**
  * @brief  Gets the FlexCAN Message Buffer interrupt flags.
  *
  * This function gets the interrupt flags of a given Message Buffers.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mask The ORed FlexCAN Message Buffer mask.
  * @return The status of given Message Buffers.
  */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
__STATIC_INLINE uint64_t FLEXCAN_GetMbStatusFlags(FLEXCAN_TypeDef *flex_can, uint64_t mask)
#else
__STATIC_INLINE uint32_t FLEXCAN_GetMbStatusFlags(FLEXCAN_TypeDef *flex_can, uint32_t mask)
#endif
{
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint64_t tempflag = (uint64_t)flex_can->IFLAG1;

    return ((tempflag | (((uint64_t)flex_can->IFLAG2) << 32)) & mask);
#else
    return (flex_can->IFLAG1 & mask);
#endif
}

/**
  * @brief  Clears the FlexCAN Message Buffer interrupt flags.
  *
  * This function clears the interrupt flags of a given Message Buffers.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mask The ORed FlexCAN Message Buffer mask.
  */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
__STATIC_INLINE void FLEXCAN_ClearMbStatusFlags(FLEXCAN_TypeDef *flex_can, uint64_t mask)
#else
__STATIC_INLINE void FLEXCAN_ClearMbStatusFlags(FLEXCAN_TypeDef *flex_can, uint32_t mask)
#endif
{
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    flex_can->IFLAG1 = (uint32_t)(mask & 0xFFFFFFFFU);
    flex_can->IFLAG2 = (uint32_t)(mask >> 32);
#else
    flex_can->IFLAG1 = mask;
#endif
}

/**
  * @brief  Enables FlexCAN interrupts according to the provided mask.
  *
  * This function enables the FlexCAN interrupts according to the provided mask. The mask
  * is a logical OR of enumeration members, see @ref _flexcan_interrupt_enable.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mask The interrupts to enable. Logical OR of @ref _flexcan_interrupt_enable.
  */
__STATIC_INLINE void FLEXCAN_EnableInterrupts(FLEXCAN_TypeDef *flex_can, uint32_t mask)
{
     /* Solve Wake Up Interrupt. ----------------------------------------------*/
    if ((uint32_t)Enum_Flexcan_WakeUpInterruptEnable == (mask & (uint32_t)Enum_Flexcan_WakeUpInterruptEnable))
    {
        flex_can->MCR |= CAN_MCR_WAKMSK_MASK;
    }

     /* Solve others. ---------------------------------------------------------*/
    flex_can->CTRL1 |= (mask & (~((uint32_t)Enum_Flexcan_WakeUpInterruptEnable)));
}

/**
  * @brief  Disables FlexCAN interrupts according to the provided mask.
  *
  * This function disables the FlexCAN interrupts according to the provided mask. The mask
  * is a logical OR of enumeration members, see @ref _flexcan_interrupt_enable.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mask The interrupts to disable. Logical OR of @ref _flexcan_interrupt_enable.
  */
__STATIC_INLINE void FLEXCAN_DisableInterrupts(FLEXCAN_TypeDef *flex_can, uint32_t mask)
{
     /* Solve Wake Up Interrupt. ----------------------------------------------*/
    if ((uint32_t)Enum_Flexcan_WakeUpInterruptEnable == (mask & (uint32_t)Enum_Flexcan_WakeUpInterruptEnable))
    {
        flex_can->MCR &= ~CAN_MCR_WAKMSK_MASK;
    }

     /* Solve others. ---------------------------------------------------------*/
    flex_can->CTRL1 &= ~(mask & (~((uint32_t)Enum_Flexcan_WakeUpInterruptEnable)));
}

/**
  * @brief  Enables FlexCAN Message Buffer interrupts.
  *
  * This function enables the interrupts of given Message Buffers.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mask The ORed FlexCAN Message Buffer mask.
  */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
__STATIC_INLINE void FLEXCAN_EnableMbInterrupts(FLEXCAN_TypeDef *flex_can, uint64_t mask)
#else
__STATIC_INLINE void FLEXCAN_EnableMbInterrupts(FLEXCAN_TypeDef *flex_can, uint32_t mask)
#endif
{
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    flex_can->IMASK1 |= (uint32_t)(mask & 0xFFFFFFFFU);
    flex_can->IMASK2 |= (uint32_t)(mask >> 32);
#else
    flex_can->IMASK1 |= mask;
#endif
}

/**
  * @brief  Disables FlexCAN Message Buffer interrupts.
  *
  * This function disables the interrupts of given Message Buffers.
  *
  * @param flex_can FlexCAN peripheral Struct Point.
  * @param mask The ORed FlexCAN Message Buffer mask.
  */
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
__STATIC_INLINE void FLEXCAN_DisableMbInterrupts(FLEXCAN_TypeDef *flex_can, uint64_t mask)
#else
__STATIC_INLINE void FLEXCAN_DisableMbInterrupts(FLEXCAN_TypeDef *flex_can, uint32_t mask)
#endif
{
#if (defined (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    flex_can->IMASK1 &= ~((uint32_t)(mask & 0xFFFFFFFFU));
    flex_can->IMASK2 &= ~((uint32_t)(mask >> 32));
#else
    flex_can->IMASK1 &= ~mask;
#endif
}

/**
  * @brief  Enables or disables the FlexCAN module operation.
  *
  * This function enables or disables the FlexCAN module.
  *
  * @param flex_can FlexCAN flex_can pointer.
  * @param  state: new state of the adc peripheral.
  */
__STATIC_INLINE void FLEXCAN_Cmd(FLEXCAN_TypeDef *flex_can, FunctionalState state)
{
    if (state)
    {
        flex_can->MCR &= ~CAN_MCR_MDIS_MASK; /*!< Enable FlexCAN */

         /* Wait FlexCAN exit from low-power mode. ----------------------------*/
        while (0U != (flex_can->MCR & CAN_MCR_LPMACK_MASK))
        {
        }
    }
    else
    {
        flex_can->MCR |= CAN_MCR_MDIS_MASK; /*!< Disable FlexCAN */

         /* Wait FlexCAN enter low-power mode. --------------------------------*/
        while (0U == (flex_can->MCR & CAN_MCR_LPMACK_MASK))
        {
        }
    }
}

void FLEXCAN_EnterFreezeMode(FLEXCAN_TypeDef *flex_can);
void FLEXCAN_ExitFreezeMode(FLEXCAN_TypeDef *flex_can);
uint32_t FLEXCAN_GetInstance(FLEXCAN_TypeDef *flex_can);
ErrorStatus FLEXCAN_CalculateImprovedTimingValues(uint32_t baudRate, uint32_t sourceClock_Hz, flexcan_timing_config_t *pTimingConfig);
void FLEXCAN_Init(FLEXCAN_TypeDef *flex_can, const flexcan_config_t *pConfig);
void FLEXCAN_Deinit(FLEXCAN_TypeDef *flex_can);
void FLEXCAN_GetDefaultConfig(flexcan_config_t *pConfig);
void FLEXCAN_SetTimingConfig(FLEXCAN_TypeDef *flex_can, const flexcan_timing_config_t *pConfig);
void FLEXCAN_SetRxMbGlobalMask(FLEXCAN_TypeDef *flex_can, uint32_t mask);
void FLEXCAN_SetRxFifoGlobalMask(FLEXCAN_TypeDef *flex_can, uint32_t mask);
void FLEXCAN_SetRxIndividualMask(FLEXCAN_TypeDef *flex_can, uint8_t maskIdx, uint32_t mask);
void FLEXCAN_TxMbConfig(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, FunctionalState state);
void FLEXCAN_RxMbConfig(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, FunctionalState state);
void FLEXCAN_RxFifoConfig(FLEXCAN_TypeDef *flex_can, const flexcan_rx_fifo_config_t *pRxFifoConfig, FunctionalState state);

#if (defined (FLEXCAN_HAS_RX_FIFO_DMA) && FLEXCAN_HAS_RX_FIFO_DMA)
void FLEXCAN_EnableRxFifoDMA(FLEXCAN_TypeDef *flex_can, FunctionalState state);
#endif 

uint32_t FLEXCAN_WriteTxMb(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_frame_t *pTxFrame);
uint32_t FLEXCAN_ReadRxMb(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pRxFrame);
void FLEXCAN_SetFDTimingConfig(FLEXCAN_TypeDef *flex_can, const flexcan_timing_config_t *pConfig);
uint32_t FLEXCAN_ReadRxFifo(FLEXCAN_TypeDef *flex_can, flexcan_frame_t *pRxFrame);
uint32_t FLEXCAN_TransferSendBlocking(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pTxFrame);
uint32_t FLEXCAN_TransferReceiveBlocking(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pRxFrame);
uint32_t FLEXCAN_TransferReceiveFifoBlocking(FLEXCAN_TypeDef *flex_can, flexcan_frame_t *pRxFrame);
void FLEXCAN_TransferCreateHandle(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_transfer_callback_t callback, void *userData);
uint32_t FLEXCAN_TransferSendNonBlocking(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);
uint32_t FLEXCAN_TransferReceiveNonBlocking(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);
uint32_t FLEXCAN_TransferReceiveFifoNonBlocking(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_fifo_transfer_t *pFifoXfer);
uint32_t FLEXCAN_GetTimeStamp(flexcan_handle_t *handle, uint8_t mbIdx);
void FLEXCAN_TransferAbortSend(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, uint8_t mbIdx);
void FLEXCAN_TransferAbortReceive(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, uint8_t mbIdx);
void FLEXCAN_TransferAbortReceiveFifo(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle);
void FLEXCAN_TransferHandleIRQ(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle);
void FLEXCAN_FDTxMbConfig(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, FunctionalState enable);
void FLEXCAN_FDInit(FLEXCAN_TypeDef *flex_can, const flexcan_config_t *pConfig, flexcan_mb_size_t dataSize, FunctionalState brs);
void FLEXCAN_SetBaudRate(FLEXCAN_TypeDef *flex_can, flexcan_timing_config_t timingConfig);
void FLEXCAN_SetFDBaudRate(FLEXCAN_TypeDef *flex_can, flexcan_timing_config_t timingConfig);
void FLEXCAN_FDRxMbConfig(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, FunctionalState state);
int32_t FLEXCAN_TransferFDReceiveNonBlocking(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);
int32_t FLEXCAN_TransferFDSendNonBlocking(FLEXCAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);
int32_t FLEXCAN_WriteFDTxMb(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_fd_frame_t *pTxFrame);
int32_t FLEXCAN_ReadFDRxMb(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_fd_frame_t *pRxFrame);
uint32_t FLEXCAN_GetFDMailboxOffset(FLEXCAN_TypeDef *flex_can, uint8_t mbIdx);
ErrorStatus FLEXCAN_FDCalculateImprovedTimingValues(uint32_t baudRate, uint32_t baudRateFD, uint32_t sourceClock_Hz, flexcan_timing_config_t *pTimingConfig);

#endif

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

