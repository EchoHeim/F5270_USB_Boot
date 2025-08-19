/***********************************************************************************************************************
    @file     hal_flexcan.h
    @author   VV TEAM
    @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE FLEXCAN
              FIRMWARE LIBRARY.
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
#ifndef __HAL_FLEXCAN_H
#define __HAL_FLEXCAN_H

/* Files includes ------------------------------------------------------------*/
#include "mm32_device.h"

/**
  * @brief Message buffer size
  */
#define FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(x) (16)

/* The count of CAN_WORD1 ----------------------------------------------------*/
#define CAN_WORD1_COUNT                 (64U)
/* The count of CAN_WORD0 ----------------------------------------------------*/
#define CAN_WORD0_COUNT                 (64U)
/* The count of CAN_CS -------------------------------------------------------*/
#define CAN_CS_COUNT                    (64U)

#define IDEAL_SP_LOW                    (750U)
#define IDEAL_SP_MID                    (800U)
#define IDEAL_SP_HIGH                   (875U)
#define IDEAL_SP_FACTOR                 (1000U)

#define MAX_PROPSEG                     (CAN_CTRL1_PROPSEG_Msk >> CAN_CTRL1_PROPSEG_Pos)
#define MAX_PSEG1                       (CAN_CTRL1_PSEG1_Msk >> CAN_CTRL1_PSEG1_Pos)
#define MAX_PSEG2                       (CAN_CTRL1_PSEG2_Msk >> CAN_CTRL1_PSEG2_Pos)
#define MAX_RJW                         (CAN_CTRL1_RJW_Msk >> CAN_CTRL1_RJW_Pos)
#define MAX_PRESDIV                     (CAN_CTRL1_PRESDIV_Msk >> CAN_CTRL1_PRESDIV_Pos)
#define CTRL1_MAX_TIME_QUANTA           (1U + MAX_PROPSEG + 1U + MAX_PSEG1 + 1U + MAX_PSEG2 + 1U)
#define CTRL1_MIN_TIME_QUANTA           (8U)

#define MAX_EPROPSEG                    (CAN_CBT_EPROPSEG_Msk >> CAN_CBT_EPROPSEG_Pos)
#define MAX_EPSEG1                      (CAN_CBT_EPSEG1_Msk >> CAN_CBT_EPSEG1_Pos)
#define MAX_EPSEG2                      (CAN_CBT_EPSEG2_Msk >> CAN_CBT_EPSEG2_Pos)
#define MAX_ERJW                        (CAN_CBT_ERJW_Msk >> CAN_CBT_ERJW_Pos)
#define MAX_EPRESDIV                    (CAN_CBT_EPRESDIV_Msk >> CAN_CBT_EPRESDIV_Pos)
#define CBT_MAX_TIME_QUANTA             (1U + MAX_EPROPSEG + 1U + MAX_EPSEG1 + 1U + MAX_EPSEG2 + 1U)
#define CBT_MIN_TIME_QUANTA             (8U)

#define MAX_FPROPSEG                    (CAN_FDCBT_FPROPSEG_MASK >> CAN_FDCBT_FPROPSEG_SHIFT)
#define MAX_FPSEG1                      (CAN_FDCBT_FPSEG1_MASK >> CAN_FDCBT_FPSEG1_SHIFT)
#define MAX_FPSEG2                      (CAN_FDCBT_FPSEG2_MASK >> CAN_FDCBT_FPSEG2_SHIFT)
#define MAX_FRJW                        (CAN_FDCBT_FRJW_MASK >> CAN_FDCBT_FRJW_SHIFT)
#define MAX_FPRESDIV                    (CAN_FDCBT_FPRESDIV_MASK >> CAN_FDCBT_FPRESDIV_SHIFT)
#define FDCBT_MAX_TIME_QUANTA           (1U + MAX_FPROPSEG + 0U + MAX_FPSEG1 + 1U + MAX_FPSEG2 + 1U)
#define FDCBT_MIN_TIME_QUANTA           (5U)

/**
  * @brief FlexCAN Frame ID helper macro.
  */
#define FLEXCAN_ID_STD(id) \
        (((uint32_t)(((uint32_t)(id)) << CAN_ID_STD_Pos))&CAN_ID_STD_Msk) /*!< Standard Frame ID helper macro. */
#define FLEXCAN_ID_EXT(id)                                  \
        (((uint32_t)(((uint32_t)(id)) << CAN_ID_EXT_Pos)) & \
         (CAN_ID_EXT_Msk | CAN_ID_STD_Msk))                               /*!< Extend Frame ID helper macro. */

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
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_A(id, rtr, ide)           /*!< Standard Rx FIFO Filter helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_B_HIGH(id, rtr, ide) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_HIGH(                \
            id, rtr, ide)                                       /*!< Standard Rx FIFO Filter helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_B_LOW(id, rtr, ide) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_B_LOW(                \
            id, rtr, ide)                                       /*!< Standard Rx FIFO Filter helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_HIGH(id) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_HIGH(      \
            id)                                                 /*!< Standard Rx FIFO Filter helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_MID_HIGH(id) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_HIGH(      \
            id)                                                 /*!< Standard Rx FIFO Filter helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_MID_LOW(id) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_MID_LOW(      \
            id)                                                 /*!< Standard Rx FIFO Filter helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_STD_FILTER_TYPE_C_LOW(id) \
        FLEXCAN_RX_FIFO_STD_MASK_TYPE_C_LOW(      \
            id)                                                 /*!< Standard Rx FIFO Filter helper macro Type C lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_A(id, rtr, ide) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_A(id, rtr, ide)           /*!< Extend Rx FIFO Filter helper macro Type A helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_B_HIGH(id, rtr, ide) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(                \
            id, rtr, ide)                                       /*!< Extend Rx FIFO Filter helper macro Type B upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_B_LOW(id, rtr, ide) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(                \
            id, rtr, ide)                                       /*!< Extend Rx FIFO Filter helper macro Type B lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_HIGH(id) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_HIGH(      \
            id)                                                 /*!< Extend Rx FIFO Filter helper macro Type C upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_MID_HIGH(id) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_HIGH(      \
            id)                                                 /*!< Extend Rx FIFO Filter helper macro Type C mid-upper part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_MID_LOW(id) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_MID_LOW(      \
            id)                                                 /*!< Extend Rx FIFO Filter helper macro Type C mid-lower part helper macro. */
#define FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_C_LOW(id) \
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_C_LOW(id)                 /*!< Extend Rx FIFO Filter helper macro Type C lower part helper macro. */

/**
  * @brief FlexCAN transfer status.
  */
#define Status_Flexcan_TxBusy                               0   /*!< Tx Message Buffer is Busy. */
#define Status_Flexcan_TxIdle                               1   /*!< Tx Message Buffer is Idle. */
#define Status_Flexcan_TxSwitchToRx                         2   /*!< Remote Message is send out and Message buffer changed to Receive one. */
#define Status_Flexcan_RxBusy                               3   /*!< Rx Message Buffer is Busy. */
#define Status_Flexcan_RxIdle                               4   /*!< Rx Message Buffer is Idle. */
#define Status_Flexcan_RxOverflow                           5   /*!< Rx Message Buffer is Overflowed. */
#define Status_Flexcan_RxFifoBusy                           6   /*!< Rx Message FIFO is Busy. */
#define Status_Flexcan_RxFifoIdle                           7   /*!< Rx Message FIFO is Idle. */
#define Status_Flexcan_RxFifoOverflow                       8   /*!< Rx Message FIFO is overflowed. */
#define Status_Flexcan_RxFifoWarning                        9   /*!< Rx Message FIFO is almost overflowed. */
#define Status_Flexcan_ErrorStatus                          10  /*!< FlexCAN Module Error and Status. */
#define Status_Flexcan_WakeUp                               11  /*!< FlexCAN is waken up from STOP mode. */
#define Status_Flexcan_UnHandled                            12  /*!< UnHadled Interrupt asserted. */
#define Status_Flexcan_RxRemote                             13  /*!< Rx Remote Message Received in Mail box. */

#define Status_Flexcan_Success                              0   /*!< Generic status for Success. */
#define Status_Flexcan_Fail                                 1   /*!< Generic status for Fail. */
#define Status_Flexcan_ReadOnly                             2   /*!< Generic status for read only failure. */
#define Status_Flexcan_OutOfRange                           3   /*!< Generic status for out of range access. */
#define Status_Flexcan_InvalidArgument                      4   /*!< Generic status for invalid argument check. */
#define Status_Flexcan_Timeout                              5   /*!< Generic status for timeout. */
#define Status_Flexcan_NoTransferInProgress                 6   /*!< Generic status for no transfer in progress. */

#define Flexcan_StateIdle                                   0x0 /*!< MB/RxFIFO idle. */
#define Flexcan_StateRxData                                 0x1 /*!< MB receiving. */
#define Flexcan_StateRxRemote                               0x2 /*!< MB receiving remote reply. */
#define Flexcan_StateTxData                                 0x3 /*!< MB transmitting. */
#define Flexcan_StateTxRemote                               0x4 /*!< MB transmitting remote request. */
#define Flexcan_StateRxFifo                                 0x5 /*!< RxFIFO receiving. */

/**
  * @brief FlexCAN message buffer CODE for Rx buffers.
  */
#define Flexcan_RxMbInactive                                0x0 /*!< MB is not active. */
#define Flexcan_RxMbFull                                    0x2 /*!< MB is full. */
#define Flexcan_RxMbEmpty                                   0x4 /*!< MB is active and empty. */
#define Flexcan_RxMbOverrun                                 0x6 /*!< MB is overwritten into a full buffer. */
#define Flexcan_RxMbBusy                                    0x8 /*!< FlexCAN is updating the contents of the MB. The CPU must not access the MB. */
#define Flexcan_RxMbRanswer                                 0xA /*!< A frame was configured to recognize a Remote Request Frame and transmit a Response Frame in return. */
#define Flexcan_RxMbNotUsed                                 0xF /*!< Not used. */

/**
  * @brief FlexCAN message buffer CODE FOR Tx buffers.
  */
#define Flexcan_TxMbInactive                                0x8 /*!< MB is not active. */
#define Flexcan_TxMbAbort                                   0x9 /*!< MB is aborted. */
#define Flexcan_TxMbDataOrRemote                            0xC /*!< MB is a TX Data Frame(when MB RTR = 0) or MB is a TX Remote Request Frame (when MB RTR = 1). */
#define Flexcan_TxMbTanswer                                 0xE /*!< MB is a TX Response Request Frame from an incoming Remote Request Frame. */
#define Flexcan_TxMbNotUsed                                 0xF /*!< Not used. */

/**
  * @brief FlexCAN frame format.
  */
typedef enum _flexcan_frame_format
{
    Enum_Flexcan_FrameFormatStandard    = 0x0U, /*!< Standard frame format attribute. */
    Enum_Flexcan_FrameFormatExtend      = 0x1U  /*!< Extend frame format attribute. */
} flexcan_frame_format_t;

/**
  * @brief FlexCAN frame type.
  */
typedef enum _flexcan_frame_type
{
    Enum_Flexcan_FrameTypeData          = 0x0U, /*!< Data frame type attribute. */
    Enum_Flexcan_FrameTypeRemote        = 0x1U  /*!< Remote frame type attribute. */
} flexcan_frame_type_t;

/**
  * @brief FlexCAN clock source.
  * @deprecated Do not use the Enum_Flexcan_ClkSrcOs.  It has been superceded Enum_Flexcan_ClkSrc0
  * @deprecated Do not use the Enum_Flexcan_ClkSrcPeri.  It has been superceded Enum_Flexcan_ClkSrc1
  */

typedef enum _flexcan_clock_source
{
    Enum_Flexcan_ClkSrcOsc              = 0x0U, /*!< FlexCAN Protocol Engine clock from Oscillator. */
    Enum_Flexcan_ClkSrcPeri             = 0x1U, /*!< FlexCAN Protocol Engine clock from Peripheral Clock. */
    Enum_Flexcan_ClkSrc0                = 0x0U, /*!< FlexCAN Protocol Engine clock selected by user as SRC == 0. */
    Enum_Flexcan_ClkSrc1                = 0x1U  /*!< FlexCAN Protocol Engine clock selected by user as SRC == 1. */
} flexcan_clock_source_t;

/**
  * @brief FlexCAN wake up source.
  */
typedef enum _flexcan_wake_up_source
{
    Enum_Flexcan_WakeupSrcUnfiltered    = 0x0U, /*!< FlexCAN uses unfiltered Rx input to detect edge. */
    Enum_Flexcan_WakeupSrcFiltered      = 0x1U  /*!< FlexCAN uses filtered Rx input to detect edge. */
} flexcan_wake_up_source_t;

/**
  * @brief FlexCAN Rx Fifo Filter type.
  */
typedef enum _flexcan_rx_fifo_filter_type
{
    Enum_Flexcan_RxFifoFilterTypeA      = 0x0U, /*!< One full ID (standard and extended) per ID Filter element. */
    Enum_Flexcan_RxFifoFilterTypeB      = 0x1U, /*!< Two full standard IDs or two partial 14-bit ID slices per ID Filter Table element. */
    Enum_Flexcan_RxFifoFilterTypeC      = 0x2U, /*!< Four partial 8-bit Standard or extended ID slices per ID Filter Table element. */
    Enum_Flexcan_RxFifoFilterTypeD      = 0x3U  /*!< All frames rejected. */
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
    Enum_Flexcan_RxFifoPrioLow          = 0x0U, /*!< Matching process start from Rx Message Buffer first */
    Enum_Flexcan_RxFifoPrioHigh         = 0x1U  /*!< Matching process start from Rx FIFO first */
} flexcan_rx_fifo_priority_t;

/**
  * @brief  FlexCAN interrupt configuration structure, default settings all disabled.
  *
  * This structure contains the settings for all of the FlexCAN Module interrupt configurations.
  * Note: FlexCAN Message Buffers and Rx FIFO have their own interrupts.
  */
#define FLEXCAN_IT_BusOffInterrupt         CAN_CTRL1_BOFFMSK_Msk  /*!< Bus Off interrupt. */
#define FLEXCAN_IT_ErrorInterrupt          CAN_CTRL1_ERRMSK_Msk   /*!< Error interrupt. */
#define FLEXCAN_IT_RxWarningInterrupt      CAN_CTRL1_RWRNMASK_Msk /*!< Rx Warning interrupt. */
#define FLEXCAN_IT_TxWarningInterrupt      CAN_CTRL1_TWRNMASK_Msk /*!< Tx Warning interrupt. */
#define FLEXCAN_IT_WakeUpInterrupt         CAN_MCR_WAKMSK_Msk     /*!< Wake Up interrupt. */

/**
  * @brief  FlexCAN Flags Definition.
  * This provides constants for the FlexCAN status flags for use in the FlexCAN functions.
  */
#define FLEXCAN_FLAG_Synch                 CAN_ESR1_SYNCH_Msk     /*!< CAN Synchronization Status. */
#define FLEXCAN_FLAG_TxWarningInt          CAN_ESR1_TWRNINT_Msk   /*!< Tx Warning Interrupt Flag. */
#define FLEXCAN_FLAG_RxWarningInt          CAN_ESR1_RWRNINT_Msk   /*!< Rx Warning Interrupt Flag. */
#define FLEXCAN_FLAG_TxErrorWarning        CAN_ESR1_TXWRN_Msk     /*!< Tx Error Warning Status. */
#define FLEXCAN_FLAG_RxErrorWarning        CAN_ESR1_RXWRN_Msk     /*!< Rx Error Warning Status. */
#define FLEXCAN_FLAG_Idle                  CAN_ESR1_IDLE_Msk      /*!< CAN IDLE Status Flag. */
#define FLEXCAN_FLAG_FaultConfinement      CAN_ESR1_FLTCONF_Msk   /*!< Fault Confinement State Flag. */
#define FLEXCAN_FLAG_Transmitting          CAN_ESR1_TX_Msk        /*!< FlexCAN In Transmission Status. */
#define FLEXCAN_FLAG_Receiving             CAN_ESR1_RX_Msk        /*!< FlexCAN In Reception Status. */
#define FLEXCAN_FLAG_BusOffInt             CAN_ESR1_BOFFINT_Msk   /*!< Bus Off Interrupt Flag. */
#define FLEXCAN_FLAG_ErrorInt              CAN_ESR1_ERRINT_Msk    /*!< Error Interrupt Flag. */
#define FLEXCAN_FLAG_WakeUpInt             CAN_ESR1_WAKINT_Msk    /*!< Wake-Up Interrupt Flag. */

#define FLEXCAN_ERROR_Stuffing             CAN_ESR1_STFERR_Msk    /*!< Stuffing Error. */
#define FLEXCAN_ERROR_Form                 CAN_ESR1_FRMERR_Msk    /*!< Form Error. */
#define FLEXCAN_ERROR_Crc                  CAN_ESR1_CRCERR_Msk    /*!< Cyclic Redundancy Check Error. */
#define FLEXCAN_ERROR_Ack                  CAN_ESR1_ACKERR_Msk    /*!< Received no ACK on transmission. */
#define FLEXCAN_ERROR_Bit0                 CAN_ESR1_BIT0ERR_Msk   /*!< Unable to send dominant bit. */
#define FLEXCAN_ERROR_Bit1                 CAN_ESR1_BIT1ERR_Msk   /*!< Unable to send recessive bit. */

/**
  * @brief  FlexCAN Rx FIFO status flags.
  *
  * The FlexCAN Rx FIFO Status enumerations are used to determine the status of the
  * Rx FIFO. Because Rx FIFO occupy the MB0 ~ MB7 (Rx Fifo filter also occupies
  * more Message Buffer space), Rx FIFO status flags are mapped to the corresponding
  * Message Buffer status flags.
  */
#define Flexcan_RxFifoOverflowFlag         CAN_IFLAG1_BUF7I_Msk   /*!< Rx FIFO overflow flag. */
#define Flexcan_RxFifoWarningFlag          CAN_IFLAG1_BUF6I_Msk   /*!< Rx FIFO almost full flag. */
#define Flexcan_RxFifoFrameAvlFlag         CAN_IFLAG1_BUF5I_Msk   /*!< Frames available in Rx FIFO flag. */

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
  * @brief FlexCAN protocol timing characteristic configuration structure.
  */
typedef struct _flexcan_timing_config
{
    uint16_t preDivider;               /*!< Clock Pre-scaler Division Factor. */
    uint8_t  rJumpwidth;               /*!< Re-sync Jump Width. */
    uint8_t  phaseSeg1;                /*!< Phase Segment 1. */
    uint8_t  phaseSeg2;                /*!< Phase Segment 2. */
    uint8_t  propSeg;                  /*!< Propagation Segment. */
} flexcan_timing_config_t;

/**
  * @brief FlexCAN module configuration structure.
  */
typedef struct _flexcan_config
{
    uint32_t                 baudRate;             /*!< FlexCAN baud rate in bps. */
    flexcan_clock_source_t   clkSrc;               /*!< Clock source for FlexCAN Protocol Engine. */
    flexcan_wake_up_source_t wakeupSrc;            /*!< Wake up source selection. */
    uint8_t                  maxMbNum;             /*!< The maximum number of Message Buffers used by user. */
    bool                     enableLoopBack;       /*!< Enable or Disable Loop Back Self Test Mode. */
    bool                     enableTimerSync;      /*!< Enable or Disable Timer Synchronization. */
    bool                     enableSelfWakeup;     /*!< Enable or Disable Self Wakeup Mode. */
    bool                     enableIndividMask;    /*!< Enable or Disable Rx Individual Mask. */
    bool                     disableSelfReception; /*!< Enable or Disable Self Reflection. */
    bool                     enableListenOnlyMode; /*!< Enable or Disable Listen Only Mode. */
    flexcan_timing_config_t  timingConfig;         /*!< Protocol timing . */
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
    flexcan_frame_t *frame;            /*!< The buffer of CAN Message to be transfer. */
    uint8_t          mbIdx;            /*!< The index of Message buffer used to transfer Message. */
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
  *  The FlexCAN transfer callback returns a value from the underlying layer.
  *  If the status equals to Status_Flexcan_ErrorStatus, the result parameter is the Content of
  *  FlexCAN status register which can be used to get the working status(or error status) of FlexCAN module.
  *  If the status equals to other FlexCAN Message Buffer transfer status, the result is the index of
  *  Message Buffer that generate transfer event.
  *  If the status equals to other FlexCAN Message Buffer transfer status, the result is meaningless and should be
  *  Ignored.
  */

typedef void (*flexcan_transfer_callback_t)(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, uint32_t status, uint32_t result, void *userData);

/**
  * @brief FlexCAN handle structure.
  */
struct _flexcan_handle
{
    flexcan_transfer_callback_t callback;                    /*!< Callback function. */
    void                       *userData;                    /*!< FlexCAN callback function parameter. */
    flexcan_frame_t *volatile   mbFrameBuf[CAN_WORD1_COUNT]; /*!< The buffer for received data from Message Buffers. */
    flexcan_frame_t *volatile   rxFifoFrameBuf;              /*!< The buffer for received data from Rx FIFO. */
    volatile uint8_t            mbState[CAN_WORD1_COUNT];    /*!< Message Buffer transfer state. */
    volatile uint8_t            rxFifoState;                 /*!< Rx FIFO transfer state. */
    volatile uint32_t           timestamp[CAN_WORD1_COUNT];  /*!< Mailbox transfer timestamp. */
};

/** @defgroup FLEXCAN_Exported_Functions
  * @{
  */
void FLEXCAN_DeInit(Flex_CAN_TypeDef *flex_can);
FlagStatus FLEXCAN_GetFlagStatus(Flex_CAN_TypeDef *flex_can, uint32_t flexcan_flag);
void FLEXCAN_ClearFlag(Flex_CAN_TypeDef *flex_can, uint32_t flexcan_flag);
void FLEXCAN_GetBusErrCount(Flex_CAN_TypeDef *flex_can, uint8_t *txErrBuf, uint8_t *rxErrBuf);
FlagStatus FLEXCAN_GetMbFlagStatus(Flex_CAN_TypeDef *flex_can, uint32_t mask);
void FLEXCAN_ClearMbFlag(Flex_CAN_TypeDef *flex_can, uint32_t mask);
void FLEXCAN_ITConfig(Flex_CAN_TypeDef *flex_can, uint32_t mask, FunctionalState state);
void FLEXCAN_MbInterruptCmd(Flex_CAN_TypeDef *flex_can, uint32_t flexcan_mbinterrupt, FunctionalState state);
void FLEXCAN_RxFifoDMACmd(Flex_CAN_TypeDef *flex_can, FunctionalState state);
uint32_t FLEXCAN_GetRxFifoHeadAddr(Flex_CAN_TypeDef *flex_can);
void FLEXCAN_Cmd(Flex_CAN_TypeDef *flex_can, FunctionalState state);
void FLEXCAN_EnterFreezeMode(Flex_CAN_TypeDef *flex_can);
void FLEXCAN_ExitFreezeMode(Flex_CAN_TypeDef *flex_can);
void FLEXCAN_Init(Flex_CAN_TypeDef *flex_can, const flexcan_config_t *pConfig, uint32_t sourceClock);
void FLEXCAN_Deinit(Flex_CAN_TypeDef *flex_can);
void FLEXCAN_GetDefaultConfig(flexcan_config_t *pConfig);
void FLEXCAN_SetTimingConfig(Flex_CAN_TypeDef *flex_can, const flexcan_timing_config_t *pConfig);
void FLEXCAN_SetRxMbGlobalMask(Flex_CAN_TypeDef *flex_can, uint32_t mask);
void FLEXCAN_SetRxFifoGlobalMask(Flex_CAN_TypeDef *flex_can, uint32_t mask);
void FLEXCAN_SetRxIndividualMask(Flex_CAN_TypeDef *flex_can, uint8_t maskIdx, uint32_t mask);
void FLEXCAN_TxMbConfig(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, FunctionalState state);
ErrorStatus FLEXCAN_CalculateImprovedTimingValues(uint32_t baudRate, uint32_t sourceClock, flexcan_timing_config_t *pTimingConfig);
void FLEXCAN_RxMbConfig(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, FunctionalState state);
void FLEXCAN_RxFifoConfig(Flex_CAN_TypeDef *flex_can, const flexcan_rx_fifo_config_t *pRxFifoConfig, FunctionalState state);
uint32_t FLEXCAN_WriteTxMb(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_frame_t *pTxFrame);
uint32_t FLEXCAN_ReadRxMb(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pRxFrame);
uint32_t FLEXCAN_ReadRxFifo(Flex_CAN_TypeDef *flex_can, flexcan_frame_t *pRxFrame);
uint32_t FLEXCAN_TransferSendBlocking(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pTxFrame);
uint32_t FLEXCAN_TransferReceiveBlocking(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pRxFrame);
uint32_t FLEXCAN_TransferReceiveFifoBlocking(Flex_CAN_TypeDef *flex_can, flexcan_frame_t *pRxFrame);
void FLEXCAN_TransferCreateHandle(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_transfer_callback_t callback, void *userData);
uint32_t FLEXCAN_TransferSendNonBlocking(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);
uint32_t FLEXCAN_TransferReceiveNonBlocking(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer);
uint32_t FLEXCAN_TransferReceiveFifoNonBlocking(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_fifo_transfer_t *pFifoXfer);
void FLEXCAN_TransferAbortSend(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, uint8_t mbIdx);
void FLEXCAN_TransferAbortReceive(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, uint8_t mbIdx);
void FLEXCAN_TransferAbortReceiveFifo(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle);
uint32_t FLEXCAN_GetTimeStamp(flexcan_handle_t *handle, uint8_t mbIdx);
void FLEXCAN_TransferHandleIRQ(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle);

/**
  * @}
  */

/**
  * @}
  */

#endif

