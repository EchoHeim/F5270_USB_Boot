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
/* Define to prevent recursive inclusion -------------------------------------*/
#define __HAL_FLEXCAN_C

/* Files includes ------------------------------------------------------------*/
#include "hal_flexcan.h"

#include "string.h"

/** @addtogroup MM32_Hardware_Abstract_Layer
  * @{
  */

/** @addtogroup CAN_HAL
  * @{
  */

/** @addtogroup CAN_Exported_Functions
  * @{
  */
/**
  * @brief  Deinitializes the CAN peripheral registers to their default reset values.
  * @param  CANx: where x can be 1 or 2 to select the CAN peripheral.
  * @retval None.
  */
void FLEXCAN_DeInit(Flex_CAN_TypeDef *flex_can)
{
    if (flex_can == FLEX_CAN1)
    {
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_FLEXCAN1, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_FLEXCAN1, DISABLE);
    }
}

/**
  * @brief  Gets the FlexCAN module  flags.
  * @param  flex_can: FlexCAN peripheral Struct Point.
  * @param  flexcan_flag: specifies the flag to check.
  *    @arg FLEXCAN_FLAG_Synch
  *    @arg FLEXCAN_FLAG_TxWarningInt
  *    @arg FLEXCAN_FLAG_RxWarningInt
  *    @arg FLEXCAN_FLAG_TxErrorWarning
  *    @arg FLEXCAN_FLAG_RxErrorWarning
  *    @arg FLEXCAN_FLAG_Idle
  *    @arg FLEXCAN_FLAG_FaultConfinement
  *    @arg FLEXCAN_FLAG_Transmitting
  *    @arg FLEXCAN_FLAG_Receiving
  *    @arg FLEXCAN_FLAG_BusOffInt
  *    @arg FLEXCAN_FLAG_ErrorInt
  *    @arg FLEXCAN_FLAG_WakeUpInt

  *    @arg FLEXCAN_ERROR_Stuffing
  *    @arg FLEXCAN_ERROR_Form
  *    @arg FLEXCAN_ERROR_Crc
  *    @arg FLEXCAN_ERROR_Ack
  *    @arg FLEXCAN_ERROR_Bit0
  *    @arg FLEXCAN_ERROR_Bit1
  * @retval The New state of adc_flag (SET or RESET).
  */
FlagStatus FLEXCAN_GetFlagStatus(Flex_CAN_TypeDef *flex_can, uint32_t flexcan_flag)
{
    return ((flex_can->ESR1 & flexcan_flag) ? SET : RESET);
}

/**
  * @brief  Clears status flags with the provided mask.
  *
  * This function clears the FlexCAN status flags with a provided mask. An automatically cleared flag
  * can't be cleared by this function.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param  flexcan_flag: specifies the flag to check.
  *    @arg FLEXCAN_FLAG_Synch
  *    @arg FLEXCAN_FLAG_TxWarningInt
  *    @arg FLEXCAN_FLAG_RxWarningInt
  *    @arg FLEXCAN_FLAG_TxErrorWarning
  *    @arg FLEXCAN_FLAG_RxErrorWarning
  *    @arg FLEXCAN_FLAG_Idle
  *    @arg FLEXCAN_FLAG_FaultConfinement
  *    @arg FLEXCAN_FLAG_Transmitting
  *    @arg FLEXCAN_FLAG_Receiving
  *    @arg FLEXCAN_FLAG_BusOffInt
  *    @arg FLEXCAN_FLAG_ErrorInt
  *    @arg FLEXCAN_FLAG_WakeUpInt

  *    @arg FLEXCAN_ERROR_Stuffing
  *    @arg FLEXCAN_ERROR_Form
  *    @arg FLEXCAN_ERROR_Crc
  *    @arg FLEXCAN_ERROR_Ack
  *    @arg FLEXCAN_ERROR_Bit0
  *    @arg FLEXCAN_ERROR_Bit1
  */
void FLEXCAN_ClearFlag(Flex_CAN_TypeDef *flex_can, uint32_t flexcan_flag)
{
    /*!< Write 1 to clear status flag. */
    flex_can->ESR1 = flexcan_flag;
}

/**
  * @brief  Gets the FlexCAN Bus Error Counter value.
  *
  * This function gets the FlexCAN Bus Error Counter value for both Tx and
  * Rx direction. These values may be needed in the upper layer error handling.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param txErrBuf: Buffer to store Tx Error Counter value.
  * @param rxErrBuf: Buffer to store Rx Error Counter value.
  */
void FLEXCAN_GetBusErrCount(Flex_CAN_TypeDef *flex_can, uint8_t *txErrBuf, uint8_t *rxErrBuf)
{
    if (NULL != txErrBuf)
    {
        *txErrBuf = (uint8_t)((flex_can->ECR & CAN_ECR_TXERRCNT_Msk) >> CAN_ECR_TXERRCNT_Pos);
    }

    if (NULL != rxErrBuf)
    {
        *rxErrBuf = (uint8_t)((flex_can->ECR & CAN_ECR_RXERRCNT_Msk) >> CAN_ECR_RXERRCNT_Pos);
    }
}

/**
  * @brief  Gets the FlexCAN Message Buffer interrupt flags.
  *
  * This function gets the interrupt flags of a given Message Buffers.
  *
  * @param flex_can： FlexCAN peripheral Struct Point.
  * @param flexcan_mbflag: The  FlexCAN Message Buffer mask.
  * @retval The New state of adc_flag (SET or RESET).
  */
FlagStatus FLEXCAN_GetMbFlagStatus(Flex_CAN_TypeDef *flex_can, uint32_t flexcan_mbflag)
{
    return ((flex_can->IFLAG1 & flexcan_mbflag) ? SET : RESET);
}

/**
  * @brief  Clears the FlexCAN Message Buffer interrupt flags.
  *
  * This function clears the interrupt flags of a given Message Buffers.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param flexcan_mbflag: The ORed FlexCAN Message Buffer mask.
  */
void FLEXCAN_ClearMbFlag(Flex_CAN_TypeDef *flex_can, uint32_t flexcan_mbflag)
{
    flex_can->IFLAG1 = flexcan_mbflag;
}

/**
  * @brief  Enables or disables FlexCAN interrupts .
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param flexcan_interrupt: The interrupts to enable.
  * @param  state: New state of the specified FLEXCAN interrupts.
  */
void FLEXCAN_ITConfig(Flex_CAN_TypeDef *flex_can, uint32_t flexcan_interrupt, FunctionalState state)
{
    if (state)
    {
        /*!< Solve Wake Up Interrupt. */
        if (FLEXCAN_IT_WakeUpInterrupt == (flexcan_interrupt & FLEXCAN_IT_WakeUpInterrupt))
        {
            flex_can->MCR |= CAN_MCR_WAKMSK_Msk;
        }

        /*!< Solve others. */
        flex_can->CTRL1 |= (flexcan_interrupt & (~FLEXCAN_IT_WakeUpInterrupt));
    }
    else
    {
        /*!< Solve Wake Up Interrupt. */
        if (FLEXCAN_IT_WakeUpInterrupt == (flexcan_interrupt & FLEXCAN_IT_WakeUpInterrupt))
        {
            flex_can->MCR &= ~CAN_MCR_WAKMSK_Msk;
        }

        /*!< Solve others. */
        flex_can->CTRL1 &= ~(flexcan_interrupt & (~FLEXCAN_IT_WakeUpInterrupt));
    }
}

/**
  * @brief  Enables or disables the FlexCAN Message Buffer interrupts.
  * @param  flex_can: FlexCAN peripheral Struct Point.
  * @param  flexcan_mbinterrupt: the flexcan Message Buffer interrupts
  * @param  state: new state of the adc peripheral.
  * @retval None.
  */
void FLEXCAN_MbInterruptCmd(Flex_CAN_TypeDef *flex_can, uint32_t flexcan_mbinterrupt, FunctionalState state)
{
    (state) ?                                   \
    (flex_can->IMASK1 |= flexcan_mbinterrupt) : \
    (flex_can->IMASK1 &= ~flexcan_mbinterrupt);
}

/**
  * @brief  Enables or disables the FlexCAN Rx FIFO DMA request.
  *
  * This function enables or disables the DMA feature of FlexCAN build-in Rx FIFO.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param  state: New state of the selected flex can DMA transfer.
  * @retval None.
  */
void FLEXCAN_RxFifoDMACmd(Flex_CAN_TypeDef *flex_can, FunctionalState state)
{
    if (state)
    {
        /*!< Enter Freeze Mode. */
        FLEXCAN_EnterFreezeMode(flex_can);

        /*!< Enable FlexCAN DMA. */
        flex_can->MCR |= CAN_MCR_DMA_Msk;

        /*!< Exit Freeze Mode. */
        FLEXCAN_ExitFreezeMode(flex_can);
    }
    else
    {
        /*!< Enter Freeze Mode. */
        FLEXCAN_EnterFreezeMode(flex_can);

        /*!< Disable FlexCAN DMA. */
        flex_can->MCR &= ~CAN_MCR_DMA_Msk;

        /*!< Exit Freeze Mode. */
        FLEXCAN_ExitFreezeMode(flex_can);
    }
}

/**
  * @brief  Gets the Rx FIFO Head address.
  *
  * This function returns the FlexCAN Rx FIFO Head address, which is mainly used for the DMA/eDMA use case.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @return FlexCAN Rx FIFO Head address.
  */
uint32_t FLEXCAN_GetRxFifoHeadAddr(Flex_CAN_TypeDef *flex_can)
{
    return ((uint32_t)&(flex_can->MB[0].CS));
}

/**
  * @brief  Enables or disables the FlexCAN module operation.
  *
  * This function enables or disables the FlexCAN module.
  *
  * @param flex_can: FlexCAN flex_can pointer.
  * @param  state: new state of the adc peripheral.
  */
void FLEXCAN_Cmd(Flex_CAN_TypeDef *flex_can, FunctionalState state)
{
    if (state)
    {
        flex_can->MCR &= ~CAN_MCR_MDIS_Msk;

        /*!< Wait FlexCAN exit from low-power mode. */
        while (0U != (flex_can->MCR & CAN_MCR_LPMACK_Msk))
        {
        }
    }
    else
    {
        flex_can->MCR |= CAN_MCR_MDIS_Msk;

        /*!< Wait FlexCAN enter low-power mode. */
        while (0U == (flex_can->MCR & CAN_MCR_LPMACK_Msk))
        {
        }
    }
}

void FLEXCAN_EnterFreezeMode(Flex_CAN_TypeDef *flex_can)
{
    /*!< Set Freeze, Halt bits. */
    flex_can->MCR |= CAN_MCR_FRZ_Msk;
    flex_can->MCR |= CAN_MCR_HALT_Msk;

    while (0U == (flex_can->MCR & CAN_MCR_FRZACK_Msk))
    {
    }
}

void FLEXCAN_ExitFreezeMode(Flex_CAN_TypeDef *flex_can)
{
    /*!< Clear Freeze, Halt bits. */
    flex_can->MCR &= ~CAN_MCR_HALT_Msk;
    flex_can->MCR &= ~CAN_MCR_FRZ_Msk;

    /*!< Wait until the FlexCAN Module exit freeze mode. */
    while (0U != (flex_can->MCR & CAN_MCR_FRZACK_Msk))
    {
    }
}

static void FLEXCAN_Reset(Flex_CAN_TypeDef *flex_can)
{
    /*!< The module must should be first exit from low power
       mode, and then soft reset can be applied. */

    uint8_t i;

    /*!< Wait until FlexCAN exit from any Low Power Mode. */
    while (0U != (flex_can->MCR & CAN_MCR_LPMACK_Msk))
    {
    }

    /*!< assert Soft Reset Signal. */
    flex_can->MCR |= CAN_MCR_SOFTRST_Msk;

    /*!< Wait until FlexCAN reset completes. */
    while (0U != (flex_can->MCR & CAN_MCR_SOFTRST_Msk))
    {
    }

    /*!< Reset MCR register. */
    flex_can->MCR |=
        CAN_MCR_WRNEN_Msk | (((uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can) - 1U) << CAN_MCR_MAXMB_Pos);

    /*!< Reset CTRL1 and CTRL2 register. */

    flex_can->CTRL1 = CAN_CTRL1_SMP_Msk;
    flex_can->CTRL2 = (0x16 << CAN_CTRL2_TASD_Pos) | CAN_CTRL2_RRS_Msk | CAN_CTRL2_EACEN_Msk;

    /*!< Clean all individual Rx Mask of Message Buffers. */
    for (i = 0; i < (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can); i++)
    {
        flex_can->RXIMR[i] = 0x3FFFFFFF;
    }

    /*!< Clean Global Mask of Message Buffers. */
    flex_can->RXMGMASK = 0x3FFFFFFF;
    /*!< Clean Global Mask of Message Buffer 14. */
    flex_can->RX14MASK = 0x3FFFFFFF;
    /*!< Clean Global Mask of Message Buffer 15. */
    flex_can->RX15MASK = 0x3FFFFFFF;
    /*!< Clean Global Mask of Rx FIFO. */
    flex_can->RXFGMASK = 0x3FFFFFFF;

    /*!< Clean all Message Buffer CS fields. */
    for (i = 0; i < (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can); i++)
    {
        flex_can->MB[i].CS = 0x0;
    }
}

/*!< Type definitions */
/*!< brief Structure type for grouping CAN bus timing related information. */
typedef struct t_can_bus_timing
{
    uint8_t timeQuanta;                /*!< Total number of time quanta */
    uint8_t propSeg;                   /*!< CAN propagation segment */
    uint8_t phaseSeg1;                 /*!< CAN phase segment 1 */
    uint8_t phaseSeg2;                 /*!< CAN phase segment 2 */
} tCanBusTiming;

/*!<Local constant declarations */
/*!<  \brief CAN bit timing table for dynamically calculating the bittiming settings. */
/*!<  \details According to the CAN protocol 1 bit-time can be made up of between 8..25 */
/*!<           time quanta (TQ). The total TQ in a bit is SYNC + TIME1SEG + TIME2SEG with SYNC */
/*!<           always being 1. The sample point is */
/*!<                 (SYNCSEG + TIME1SEG) / (SYNCSEG + TIME1SEG + TIME2SEG) * 100% */
/*!<           This array contains possible and valid time quanta configurations */
/*!<           with a sample point between 68..78%. A visual representation of the TQ in */
/*!<           a bit is: */
/*!<             | SYNCSEG +      TIME1SEG       + TIME2SEG  |  = Time_Quanta */
/*!<           Or with an alternative representation: */
/*!<             | SYNCSEG + PROPSEG + PHASE1SEG + PHASE2SEG |  = Time-Quanta */
/*!<           With the alternative representation TIME1SEG = PROPSEG + PHASE1SEG. */
/*!<                                               TIME2SEG = PHASE2SEG */
/*!<               SYNCSEG = 1 */

static const tCanBusTiming canTiming[] =
{
    /*!< Sample-point rate = (SYNCSEG + TIME1SEG) / (SYNCSEG + TIME1SEG + TIME2SEG) * 100% */
    /*!<                   = (1 + PROPSEG + PHASE1SEG) / (1 + PROPSEG + PHASE1SEG + PHASE2SEG) * 100% */
    /*!< 1 bit-time can be made up of between 8..25 */
    /*!< SYNCSEG = 1 */
    /*!< PROPSEG + PHASE1SEG can be made up of between 2..16 */
    /*!< PHASE2SEG can be made up of between 2..8 */
    /*!< PROPSEG + PHASE1SEG + PHASE2SEG must be bigger than 5 */
    /*!<                             | SYNCSEG | PROPSEG  | PHASE1SEG | PHASE2SEG |  PreSample | Time-Quanta   | Sample-Point | */
    /*!< ----------------------------+---------+----------+-----------+-----------+------------+---------------+--------------+ */
    {  8U, 3U, 2U, 2U },               //|    1    |  3       |  2        |  2        |   1+3+2= 6 | 1+3+2+2= 8    |  6/ 8 = 75%  |
    {  9U, 3U, 3U, 2U },               //|    1    |  3       |  3        |  2        |   1+3+3= 7 | 1+3+3+2= 9    |  7/ 9 = 78%  |
    { 10U, 3U, 3U, 3U },               //|    1    |  3       |  3        |  3        |   1+3+3= 7 | 1+3+3+3=10    |  7/10 = 70%  |
    { 11U, 4U, 3U, 3U },               //|    1    |  4       |  3        |  3        |   1+4+3= 8 | 1+4+3+3=11    |  8/11 = 73%  |
    { 12U, 4U, 4U, 3U },               //|    1    |  4       |  4        |  3        |   1+4+4= 9 | 1+4+4+3=12    |  9/12 = 75%  |
    { 13U, 5U, 4U, 3U },               //|    1    |  5       |  4        |  3        |   1+5+4=10 | 1+5+4+3=13    | 10/13 = 77%  |
    { 14U, 5U, 4U, 4U },               //|    1    |  5       |  4        |  4        |   1+5+4=10 | 1+5+4+4=14    | 10/14 = 71%  |
    { 15U, 6U, 4U, 4U },               //|    1    |  6       |  4        |  4        |   1+6+4=11 | 1+6+4+4=15    | 11/15 = 73%  |
    { 16U, 6U, 5U, 4U },               //|    1    |  6       |  5        |  4        |   1+6+5=12 | 1+6+5+4=16    | 12/16 = 75%  |
    { 17U, 7U, 5U, 4U },               //|    1    |  7       |  5        |  4        |   1+7+5=13 | 1+7+5+4=17    | 13/17 = 76%  |
    { 18U, 7U, 5U, 5U },               //|    1    |  7       |  5        |  5        |   1+7+5=13 | 1+7+5+5=18    | 13/18 = 72%  |
    { 19U, 8U, 5U, 5U },               //|    1    |  8       |  5        |  5        |   1+8+5=14 | 1+8+5+5=19    | 14/19 = 74%  |
    { 20U, 8U, 6U, 5U },               //|    1    |  8       |  6        |  5        |   1+8+6=15 | 1+8+6+5=20    | 15/20 = 75%  |
    { 21U, 8U, 7U, 5U },               //|    1    |  8       |  7        |  5        |   1+8+7=16 | 1+8+7+5=21    | 16/21 = 76%  |
    { 22U, 8U, 7U, 6U },               //|    1    |  8       |  7        |  6        |   1+8+7=16 | 1+8+7+6=22    | 16/22 = 73%  |
    { 23U, 8U, 8U, 6U },               //|    1    |  8       |  8        |  6        |   1+8+8=17 | 1+8+8+6=23    | 17/23 = 74%  |
    { 24U, 8U, 8U, 7U },               //|    1    |  8       |  8        |  7        |   1+8+8=17 | 1+8+8+7=24    | 17/24 = 71%  |
    { 25U, 8U, 8U, 8U }                //|    1    |  8       |  8        |  8        |   1+8+8=17 | 1+8+8+8=25    | 17/25 = 68%  |
};

static void FLEXCAN_SetBaudRate(Flex_CAN_TypeDef *flex_can, uint32_t sourceClock, uint32_t baudRate_Bps, flexcan_timing_config_t timingConfig)
{
    uint8_t  cnt;
    uint32_t priDiv;

    /*!< Loop through all possible time quanta configurations to find a match. */
    for (cnt = 0; cnt < sizeof(canTiming) / sizeof(canTiming[0]); cnt++)
    {
        if ((sourceClock % (baudRate_Bps * canTiming[cnt].timeQuanta)) == 0U)
        {
            /*!< Compute the prescaler that goes with this TQ configuration. */
            priDiv = sourceClock / (baudRate_Bps * canTiming[cnt].timeQuanta);
            timingConfig.preDivider = (uint16_t)priDiv;

            /*!< Make sure the prescaler is valid. */
            if ((priDiv > 0U) && (priDiv <= 256U))
            {
                /*!< Found a good bus timing configuration. */
                timingConfig.phaseSeg1 = canTiming[cnt].phaseSeg1;
                timingConfig.phaseSeg2 = canTiming[cnt].phaseSeg2;
                timingConfig.propSeg   = canTiming[cnt].propSeg;
                break;
            }
        }
    }

    /*!< Update actual timing characteristic. */
    FLEXCAN_SetTimingConfig(flex_can, (const flexcan_timing_config_t *)(uint32_t)&timingConfig);
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
  * param flex_can FlexCAN peripheral Struct Point.
  * param pConfig Pointer to the user-defined configuration structure.
  * param sourceClock FlexCAN Protocol Engine clock source frequency in Hz.
  */
void FLEXCAN_Init(Flex_CAN_TypeDef *flex_can, const flexcan_config_t *pConfig, uint32_t sourceClock)
{
    uint32_t mcrTemp;
    uint32_t ctrl1Temp;

    /*!< Enable FlexCAN Module for configuration. */
    FLEXCAN_Cmd(flex_can, ENABLE);

    /*!< Reset to known status. */
    FLEXCAN_Reset(flex_can);

    /*!< Save current CTRL1 value and enable to enter Freeze mode(enabled by default). */
    ctrl1Temp = flex_can->CTRL1;

    /*!< Save current MCR value and enable to enter Freeze mode(enabled by default). */
    mcrTemp = flex_can->MCR;

    /*!< Enable Loop Back Mode? */
    ctrl1Temp = (pConfig->enableLoopBack) ? (ctrl1Temp | CAN_CTRL1_LPB_Msk) : (ctrl1Temp & ~CAN_CTRL1_LPB_Msk);

    /*!< Enable Timer Sync? */
    ctrl1Temp = (pConfig->enableTimerSync) ? (ctrl1Temp | CAN_CTRL1_TSYN_Msk) : (ctrl1Temp & ~CAN_CTRL1_TSYN_Msk);

    /*!< Enable Listen Only Mode? */
    ctrl1Temp = (pConfig->enableListenOnlyMode) ? ctrl1Temp | CAN_CTRL1_LOM_Msk : ctrl1Temp & ~CAN_CTRL1_LOM_Msk;

    /*!< Set the maximum number of Message Buffers */
    mcrTemp = (mcrTemp & ~CAN_MCR_MAXMB_Msk) | (((uint32_t)pConfig->maxMbNum - 1U) << CAN_MCR_MAXMB_Pos);

    /*!< Enable Self Wake Up Mode and configure the wake up source. */
    mcrTemp = (pConfig->enableSelfWakeup) ? (mcrTemp | CAN_MCR_SLFWAK_Msk) : (mcrTemp & ~CAN_MCR_SLFWAK_Msk);
    mcrTemp = (Enum_Flexcan_WakeupSrcFiltered == pConfig->wakeupSrc) ? (mcrTemp | CAN_MCR_WAKSRC_Msk) :
              (mcrTemp & ~CAN_MCR_WAKSRC_Msk);

    /*!< Enable Individual Rx Masking? */
    mcrTemp = (pConfig->enableIndividMask) ? (mcrTemp | CAN_MCR_IRMQ_Msk) : (mcrTemp & ~CAN_MCR_IRMQ_Msk);

    /*!< Disable Self Reception? */
    mcrTemp = (pConfig->disableSelfReception) ? mcrTemp | CAN_MCR_SRXDIS_Msk : mcrTemp & ~CAN_MCR_SRXDIS_Msk;

    /*!< Write back CTRL1 Configuration to register. */
    flex_can->CTRL1 = ctrl1Temp;

    /*!< Write back MCR Configuration to register. */
    flex_can->MCR = mcrTemp;

    /*!< Baud Rate Configuration. */
    FLEXCAN_SetBaudRate(flex_can, sourceClock, pConfig->baudRate, pConfig->timingConfig);
}

/**
  * brief De-initializes a FlexCAN instance.
  *
  * This function disables the FlexCAN module clock and sets all register values
  * to the reset value.
  *
  * param flex_can FlexCAN peripheral Struct Point.
  */
void FLEXCAN_Deinit(Flex_CAN_TypeDef *flex_can)
{
    /*!< Reset all Register Contents. */
    FLEXCAN_Reset(flex_can);

    /*!< Disable FlexCAN module. */
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
  * param pConfig Pointer to the FlexCAN configuration structure.
  */
void FLEXCAN_GetDefaultConfig(flexcan_config_t *pConfig)
{
    /*!< Initializes the configure structure to zero. */
    (void)memset(pConfig, 0, sizeof(*pConfig));

    /*!< Initialize FlexCAN Module config struct with default value. */
    pConfig->clkSrc   = Enum_Flexcan_ClkSrc0;
    pConfig->baudRate = 500000U;

    pConfig->maxMbNum             = 16;
    pConfig->enableLoopBack       = false;
    pConfig->enableTimerSync      = true;
    pConfig->enableSelfWakeup     = false;
    pConfig->wakeupSrc            = Enum_Flexcan_WakeupSrcUnfiltered;
    pConfig->enableIndividMask    = false;
    pConfig->disableSelfReception = false;
    pConfig->enableListenOnlyMode = false;
    /*!< Default protocol timing configuration, time quantum is 10. */
    pConfig->timingConfig.phaseSeg1  = 3;
    pConfig->timingConfig.phaseSeg2  = 2;
    pConfig->timingConfig.propSeg    = 1;
    pConfig->timingConfig.rJumpwidth = 1;
}

/**
  * @brief :Sets the FlexCAN protocol timing characteristic.
  *
  * This function gives user settings to CAN bus timing characteristic.
  * The function is for an experienced user. For less experienced users, call
  * the FLEXCAN_Init() and fill the baud rate field with a desired value.
  * This provides the default timing characteristics to the module.
  *
  * Note that calling FLEXCAN_SetTimingConfig() overrides the baud rate set
  * in FLEXCAN_Init().
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param pConfig: Pointer to the timing configuration structure.
  */
void FLEXCAN_SetTimingConfig(Flex_CAN_TypeDef *flex_can, const flexcan_timing_config_t *pConfig)
{
    /*!< Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

    /*!< Cleaning previous Timing Setting. */
    flex_can->CTRL1 &= ~(CAN_CTRL1_PRESDIV_Msk | CAN_CTRL1_RJW_Msk | CAN_CTRL1_PSEG1_Msk | CAN_CTRL1_PSEG2_Msk |
                         CAN_CTRL1_PROPSEG_Msk);

    /*!< Updating Timing Setting according to configuration structure. */
    flex_can->CTRL1 |= (((pConfig->preDivider - 1) << CAN_CTRL1_PRESDIV_Pos) | ((pConfig->rJumpwidth) << CAN_CTRL1_RJW_Pos) |
                        ((pConfig->phaseSeg1 - 1) << CAN_CTRL1_PSEG1_Pos) | ((pConfig->phaseSeg2 - 1) << CAN_CTRL1_PSEG2_Pos) |
                        ((pConfig->propSeg - 1) << CAN_CTRL1_PROPSEG_Pos));

    /*!< Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

/**
  * @brief Sets the FlexCAN receive message buffer global mask.
  *
  * This function sets the global mask for the FlexCAN message buffer in a matching process.
  * The configuration is only effective when the Rx individual mask is disabled in the FLEXCAN_Init().
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param mask: Rx Message Buffer Global Mask value.
  */
void FLEXCAN_SetRxMbGlobalMask(Flex_CAN_TypeDef *flex_can, uint32_t mask)
{
    /*!< Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

    /*!< Setting Rx Message Buffer Global Mask value. */
    flex_can->RXMGMASK = mask;
    flex_can->RX14MASK = mask;
    flex_can->RX15MASK = mask;

    /*!< Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

/**
  * @brief Sets the FlexCAN receive FIFO global mask.
  *
  * This function sets the global mask for FlexCAN FIFO in a matching process.
  *
  * @param flex_can： FlexCAN peripheral Struct Point.
  * @param mask： Rx Fifo Global Mask value.
  */
void FLEXCAN_SetRxFifoGlobalMask(Flex_CAN_TypeDef *flex_can, uint32_t mask)
{
    /*!< Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

    /*!< Setting Rx FIFO Global Mask value. */
    flex_can->RXFGMASK = mask;

    /*!< Exit Freeze Mode. */
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
  * @param flex_can： FlexCAN peripheral Struct Point.
  * @param maskIdx： The Index of individual Mask.
  * @param mask： Rx Individual Mask value.
  */
void FLEXCAN_SetRxIndividualMask(Flex_CAN_TypeDef *flex_can, uint8_t maskIdx, uint32_t mask)
{
    /*!< Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

    /*!< Setting Rx Individual Mask value. */
    flex_can->RXIMR[maskIdx] = mask;

    /*!< Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

/**
  * @brief Configures a FlexCAN transmit message buffer.
  *
  * This function aborts the previous transmission, cleans the Message Buffer, and
  * configures it as a Transmit Message Buffer.
  *
  * @param flex_can： FlexCAN peripheral Struct Point.
  * @param mbIdx： The Message Buffer index.
  * @param state:  This parameter can be: ENABLE or DISABLE.
  */
void FLEXCAN_TxMbConfig(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, FunctionalState state)
{
    /*!< Inactivate Message Buffer. */
    if (state)
    {
        flex_can->MB[mbIdx].CS = ((Flexcan_TxMbInactive) << CAN_CS_CODE_Pos);
    }
    else
    {
        flex_can->MB[mbIdx].CS = 0;
    }

    /*!< Clean Message Buffer content. */
    flex_can->MB[mbIdx].ID    = 0x0;
    flex_can->MB[mbIdx].WORD0 = 0x0;
    flex_can->MB[mbIdx].WORD1 = 0x0;
}

/**
  * @brief Calculates the segment values for a single bit time for classical CAN
  *
  * @param baudRate： The data speed in bps
  * @param tqNum： Number of time quantas per bit
  * @param pTimingConfig： Pointer to the FlexCAN timing configuration structure.
  *
  * @return SUCCESS if Calculates the segment success, ERROR if Calculates the segment success
  */
static ErrorStatus FLEXCAN_GetSegments(uint32_t baudRate, uint32_t tqNum, flexcan_timing_config_t *pTimingConfig)
{
    uint32_t ideal_sp;
    uint32_t p1;
    ErrorStatus fgRet = ERROR;

    /*!< Get ideal sample point. For the Bit field in CTRL1 register can't calculate higher ideal SP, we set it as the
       lowest one(75%). */
    ideal_sp = IDEAL_SP_LOW;

    /*!< distribute time quanta. */
    p1                     = tqNum * (uint32_t)ideal_sp;
    pTimingConfig->propSeg = (uint8_t)(p1 / (uint32_t)IDEAL_SP_FACTOR - 2U);

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

        /*!< The value of prog Seg should be not larger than tqNum -4U. */
        if ((pTimingConfig->propSeg + pTimingConfig->phaseSeg1) < ((uint8_t)tqNum - 4U))
        {
            pTimingConfig->phaseSeg2 = (uint8_t)tqNum - (pTimingConfig->phaseSeg1 + pTimingConfig->propSeg + 4U);

            if (pTimingConfig->phaseSeg2 <= MAX_PSEG1)
            {
                if ((pTimingConfig->phaseSeg1 < pTimingConfig->phaseSeg2) &&
                    (pTimingConfig->propSeg > (pTimingConfig->phaseSeg2 - pTimingConfig->phaseSeg1)))
                {
                    pTimingConfig->propSeg  -= (pTimingConfig->phaseSeg2 - pTimingConfig->phaseSeg1);
                    pTimingConfig->phaseSeg1 = pTimingConfig->phaseSeg2;
                }

                /*!< subtract one TQ for sync seg. */
                /*!< sjw is 20% of total TQ, rounded to nearest int. */
                pTimingConfig->rJumpwidth = ((uint8_t)tqNum + 4U) / 5U - 1U;

                /*!< The max tqNum for CBT will reach to 129, ERJW would not be larger than 26. */
                /*!< Considering that max ERJW is 31, rJumpwidth will always be smaller than MAX_ERJW. */
                if (pTimingConfig->rJumpwidth > MAX_RJW)
                {
                    pTimingConfig->rJumpwidth = MAX_RJW;
                }

                fgRet = SUCCESS;
            }
        }
    }

    return (fgRet);
}

/**
  * @brief Calculates the improved timing values by specific baudrates for classical CAN
  *
  * @param baudRate：  The classical CAN speed in bps defined by user
  * @param sourceClock： The Source clock data speed in bps. Zero to disable baudrate switching
  * @param pTimingConfig： Pointer to the FlexCAN timing configuration structure.
  *
  * @return SUCCESS if timing configuration found, ERROR if failed to find configuration
  */
ErrorStatus FLEXCAN_CalculateImprovedTimingValues(uint32_t baudRate, uint32_t sourceClock, flexcan_timing_config_t *pTimingConfig)
{
    uint32_t clk;                      /*!< the clock is tqNumb x baudRateFD. */
    uint32_t tqNum;                    /*!< Numbers of TQ. */
    ErrorStatus fgRet = ERROR;

    /*!<  Auto Improved Protocal timing for CTRL1. */
    tqNum = CTRL1_MAX_TIME_QUANTA;

    do
    {
        clk = baudRate * tqNum;

        if (clk > sourceClock)
        {
            continue;                  /*!< tqNum too large, clk has been exceed sourceClock. */
        }

        if ((sourceClock / clk * clk) != sourceClock)
        {
            continue;                  /*!< Non-supporting: the frequency of clock source is not divisible by target baud rate, the user
                                          should change a divisible baud rate. */
        }

        pTimingConfig->preDivider = (uint16_t)(sourceClock / clk) - 1U;

        if (pTimingConfig->preDivider > MAX_PRESDIV)
        {
            break;                     /*!< The frequency of source clock is too large or the baud rate is too small, the pre-divider could
                                          not handle it. */
        }

        /*!< Try to get the best timing configuration. */
        if (FLEXCAN_GetSegments(baudRate, tqNum, pTimingConfig))
        {
            fgRet = SUCCESS;
            break;
        }
    }
    while(--tqNum >= CTRL1_MIN_TIME_QUANTA);

    return (fgRet);
}

/**
  * @brief Configures a FlexCAN Receive Message Buffer.
  *
  * This function cleans a FlexCAN build-in Message Buffer and configures it
  * as a Receive Message Buffer.
  *
  * @param flex_can： FlexCAN peripheral Struct Point.
  * @param mbIdx： The Message Buffer index.
  * @param pRxMbConfig： Pointer to the FlexCAN Message Buffer configuration structure.
  * @param state:  This parameter can be: ENABLE or DISABLE.
  */
void FLEXCAN_RxMbConfig(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_rx_mb_config_t *pRxMbConfig, FunctionalState state)
{
    uint32_t cs_temp = 0;

    /*!< Inactivate Message Buffer. */
    flex_can->MB[mbIdx].CS = 0;

    /*!< Clean Message Buffer content. */
    flex_can->MB[mbIdx].ID    = 0x0;
    flex_can->MB[mbIdx].WORD0 = 0x0;
    flex_can->MB[mbIdx].WORD1 = 0x0;

    if (state)
    {
        /*!< Setup Message Buffer ID. */
        flex_can->MB[mbIdx].ID = pRxMbConfig->id;

        /*!< Setup Message Buffer format. */
        if (Enum_Flexcan_FrameFormatExtend == pRxMbConfig->format)
        {
            cs_temp |= CAN_CS_IDE_Msk;
        }

        /*!< Setup Message Buffer type. */
        if (Enum_Flexcan_FrameTypeRemote == pRxMbConfig->type)
        {
            cs_temp |= CAN_CS_RTR_Msk;
        }

        /*!< Activate Rx Message Buffer. */
        cs_temp |= ((Flexcan_RxMbEmpty) << CAN_CS_CODE_Pos);
        flex_can->MB[mbIdx].CS = cs_temp;
    }
}

/**
  * @brief Configures the FlexCAN Rx FIFO.
  *
  * This function configures the Rx FIFO with given Rx FIFO configuration.
  *
  * @param flex_can： FlexCAN peripheral Struct Point.
  * @param pRxFifoConfig： Pointer to the FlexCAN Rx FIFO configuration structure.
  * @param state:  This parameter can be: ENABLE or DISABLE.
  */
void FLEXCAN_RxFifoConfig(Flex_CAN_TypeDef *flex_can, const flexcan_rx_fifo_config_t *pRxFifoConfig, FunctionalState state)
{
    volatile uint32_t *mbAddr;
    uint8_t i, j, k, rffn = 0, numMbOccupy;
    uint32_t setup_mb = 0;

    /*!< Enter Freeze Mode. */
    FLEXCAN_EnterFreezeMode(flex_can);

    if (state)
    {
        /*!< Get the setup_mb value. */
        setup_mb = (uint8_t)((flex_can->MCR & CAN_MCR_MAXMB_Msk) >> CAN_MCR_MAXMB_Pos);
        setup_mb = (setup_mb < (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can)) ?
                   setup_mb :
                   (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can);

        /*!< Determine RFFN value. */
        for (i = 0; i <= 0xFU; i++)
        {
            if ((8U * (i + 1U)) >= pRxFifoConfig->idFilterNum)
            {
                rffn = i;
                /*!< assert(((setup_mb - 8U) - (2U * rffn)) > 0U); */

                flex_can->CTRL2 = (flex_can->CTRL2 & ~CAN_CTRL2_RFFN_Msk) | (rffn << CAN_CTRL2_RFFN_Pos);
                break;
            }
        }

        /*!< caculate the Number of Mailboxes occupied by RX Legacy FIFO and the filter. */
        numMbOccupy = 6U + (rffn + 1U) * 2U;

        /*!< Copy ID filter table to Message Buffer Region (Fix MISRA_C-2012 Rule 18.1). */
        j = 0U;

        for (i = 6U; i < numMbOccupy; i++)
        {
            /*!< Get address for current mail box. */
            mbAddr = &(flex_can->MB[i].CS);

            /*!< One Mail box contain 4U DWORD registers. */
            for (k = 0; k < 4U; k++)
            {
                /*!< Fill all valid filter in the mail box occupied by filter. */
                /*!< Disable unused Rx FIFO Filter, the other rest of register in the last Mail box occupied by fiter set */
                /*!< as 0xffffffff. */

                mbAddr[k] = (j < pRxFifoConfig->idFilterNum) ? (pRxFifoConfig->idFilterTable[j]) : 0xFFFFFFFFU;

                /*!< Try to fill next filter in current Mail Box. */
                j++;
            }
        }

        /*!< Setup ID Fitlter Type. */
        switch (pRxFifoConfig->idFilterType)
        {
            case Enum_Flexcan_RxFifoFilterTypeA:
                flex_can->MCR = (flex_can->MCR & ~CAN_MCR_IDAM_Msk) | ((0x0) << CAN_MCR_IDAM_Pos);
                break;

            case Enum_Flexcan_RxFifoFilterTypeB:
                flex_can->MCR = (flex_can->MCR & ~CAN_MCR_IDAM_Msk) | ((0x1) << CAN_MCR_IDAM_Pos);
                break;

            case Enum_Flexcan_RxFifoFilterTypeC:
                flex_can->MCR = (flex_can->MCR & ~CAN_MCR_IDAM_Msk) | ((0x2) << CAN_MCR_IDAM_Pos);
                break;

            case Enum_Flexcan_RxFifoFilterTypeD:
                /*!< All frames rejected. */
                flex_can->MCR = (flex_can->MCR & ~CAN_MCR_IDAM_Msk) | ((0x3) << CAN_MCR_IDAM_Pos);
                break;

            default:
                /*!< All the cases have been listed above, the default clause should not be reached. */
                break;
        }

        /*!< Setting Message Reception Priority. */
        flex_can->CTRL2 = (pRxFifoConfig->priority == Enum_Flexcan_RxFifoPrioHigh) ? (flex_can->CTRL2 & ~CAN_CTRL2_MRP_Msk) :
                          (flex_can->CTRL2 | CAN_CTRL2_MRP_Msk);

        /*!< Enable Rx Message FIFO. */
        flex_can->MCR |= CAN_MCR_RFEN_Msk;
    }
    else
    {
        rffn = (uint8_t)((flex_can->CTRL2 & CAN_CTRL2_RFFN_Msk) >> CAN_CTRL2_RFFN_Pos);
        /*!< caculate the Number of Mailboxes occupied by RX Legacy FIFO and the filter. */
        numMbOccupy = 6U + (rffn + 1U) * 2U;

        /*!< Disable Rx Message FIFO. */
        flex_can->MCR &= ~CAN_MCR_RFEN_Msk;

        /*!< Clean MB0 ~ MB5 and all MB occupied by ID filters (Fix MISRA_C-2012 Rule 18.1). */
        for (i = 0; i < numMbOccupy; i++)
        {
            FLEXCAN_RxMbConfig(flex_can, i, NULL, DISABLE);
        }
    }

    /*!< Exit Freeze Mode. */
    FLEXCAN_ExitFreezeMode(flex_can);
}

/**
  * @brief Writes a FlexCAN Message to the Transmit Message Buffer.
  *
  * This function writes a CAN Message to the specified Transmit Message Buffer
  * and changes the Message Buffer state to start CAN Message transmit. After
  * that the function returns immediately.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param mbIdx: The FlexCAN Message Buffer index.
  * @param pTxFrame: Pointer to CAN message frame to be sent.
  * @retval Status_Flexcan_Success - Write Tx Message Buffer Successfully.
  *         Status_Flexcan_Fail    - Tx Message Buffer is currently in use.
  */
uint32_t FLEXCAN_WriteTxMb(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, const flexcan_frame_t *pTxFrame)
{
    uint32_t cs_temp = 0;
    uint32_t status;

    /*!< Check if Message Buffer is available. */
    if (((Flexcan_TxMbDataOrRemote) << CAN_CS_CODE_Pos) != (flex_can->MB[mbIdx].CS & CAN_CS_CODE_Msk))
    {
        /*!< Inactive Tx Message Buffer. */
        flex_can->MB[mbIdx].CS = (flex_can->MB[mbIdx].CS & ~CAN_CS_CODE_Msk) | ((Flexcan_TxMbInactive) << CAN_CS_CODE_Pos);

        /*!< Fill Message ID field. */
        flex_can->MB[mbIdx].ID = pTxFrame->id;

        /*!< Fill Message Format field. */
        if ((uint32_t)Enum_Flexcan_FrameFormatExtend == pTxFrame->format)
        {
            cs_temp |= CAN_CS_SRR_Msk | CAN_CS_IDE_Msk;
        }

        /*!< Fill Message Type field. */
        if ((uint32_t)Enum_Flexcan_FrameTypeRemote == pTxFrame->type)
        {
            cs_temp |= CAN_CS_RTR_Msk;
        }

        cs_temp |= ((Flexcan_TxMbDataOrRemote) << CAN_CS_CODE_Pos) | ((pTxFrame->length) << CAN_CS_DLC_Pos);

        /*!< Load Message Payload. */
        flex_can->MB[mbIdx].WORD0 = pTxFrame->dataWord0;
        flex_can->MB[mbIdx].WORD1 = pTxFrame->dataWord1;

        /*!< Activate Tx Message Buffer. */
        flex_can->MB[mbIdx].CS = cs_temp;

        status = Status_Flexcan_Success;
    }
    else
    {
        /*!< Tx Message Buffer is activated, return immediately. */
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
  * @param flex_can： FlexCAN peripheral Struct Point.
  * @param mbIdx： The FlexCAN Message Buffer index.
  * @param pRxFrame： Pointer to CAN message frame structure for reception.
  * @retval Status_Flexcan_Success            - Rx Message Buffer is full and has been read successfully.
  *         Status_Flexcan_RxOverflow    - Rx Message Buffer is already overflowed and has been read successfully.
  *         Status_Flexcan_Fail               - Rx Message Buffer is empty.
  */
uint32_t FLEXCAN_ReadRxMb(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pRxFrame)
{
    uint32_t cs_temp;
    uint32_t rx_code;
    uint32_t status;

    /*!< Read CS field of Rx Message Buffer to lock Message Buffer. */
    cs_temp = flex_can->MB[mbIdx].CS;
    /*!< Get Rx Message Buffer Code field. */
    rx_code = (cs_temp & CAN_CS_CODE_Msk) >> CAN_CS_CODE_Pos;

    /*!< Check to see if Rx Message Buffer is full. */
    if (((uint32_t)Flexcan_RxMbFull == rx_code) || ((uint32_t)Flexcan_RxMbOverrun == rx_code))
    {
        /*!< Store Message ID. */
        pRxFrame->id = flex_can->MB[mbIdx].ID & (CAN_ID_EXT_Msk | CAN_ID_STD_Msk);

        /*!< Get the message ID and format. */
        pRxFrame->format = (cs_temp & CAN_CS_IDE_Msk) != 0U ? (uint8_t)Enum_Flexcan_FrameFormatExtend :
                           (uint8_t)Enum_Flexcan_FrameFormatStandard;

        /*!< Get the message type. */
        pRxFrame->type =
            (cs_temp & CAN_CS_RTR_Msk) != 0U ? (uint8_t)Enum_Flexcan_FrameTypeRemote : (uint8_t)Enum_Flexcan_FrameTypeData;

        /*!< Get the message length. */
        pRxFrame->length = (uint8_t)((cs_temp & CAN_CS_DLC_Msk) >> CAN_CS_DLC_Pos);

        /*!< Get the time stamp. */
        pRxFrame->timestamp = (uint16_t)((cs_temp & CAN_CS_TIME_STAMP_Msk) >> CAN_CS_TIME_STAMP_Pos);

        /*!< Store Message Payload. */
        pRxFrame->dataWord0 = flex_can->MB[mbIdx].WORD0;
        pRxFrame->dataWord1 = flex_can->MB[mbIdx].WORD1;

        /*!< Read free-running timer to unlock Rx Message Buffer. */
        (void)flex_can->TIMER;

        if ((uint32_t)Flexcan_RxMbFull == rx_code)
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
        /*!< Read free-running timer to unlock Rx Message Buffer. */
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
  * @param flex_can： FlexCAN peripheral Struct Point.
  * @param pRxFrame： Pointer to CAN message frame structure for reception.
  * @retval Status_Flexcan_Success - Read Message from Rx FIFO successfully.
  *         Status_Flexcan_Fail    - Rx FIFO is not enabled.
  */
uint32_t FLEXCAN_ReadRxFifo(Flex_CAN_TypeDef *flex_can, flexcan_frame_t *pRxFrame)
{
    uint32_t cs_temp;
    uint32_t status;

    /*!< Check if Rx FIFO is Enabled. */
    if (0U != (flex_can->MCR & CAN_MCR_RFEN_Msk))
    {
        /*!< Read CS field of Rx Message Buffer to lock Message Buffer. */
        cs_temp = flex_can->MB[0].CS;

        /*!< Read data from Rx FIFO output port. */
        /*!< Store Message ID. */
        pRxFrame->id = flex_can->MB[0].ID & (CAN_ID_EXT_Msk | CAN_ID_STD_Msk);

        /*!< Get the message ID and format. */
        pRxFrame->format = (cs_temp & CAN_CS_IDE_Msk) != 0U ? (uint8_t)Enum_Flexcan_FrameFormatExtend :
                           (uint8_t)Enum_Flexcan_FrameFormatStandard;

        /*!< Get the message type. */
        pRxFrame->type =
            (cs_temp & CAN_CS_RTR_Msk) != 0U ? (uint8_t)Enum_Flexcan_FrameTypeRemote : (uint8_t)Enum_Flexcan_FrameTypeData;

        /*!< Get the message length. */
        pRxFrame->length = (uint8_t)((cs_temp & CAN_CS_DLC_Msk) >> CAN_CS_DLC_Pos);

        /*!< Get the time stamp. */
        pRxFrame->timestamp = (uint16_t)((cs_temp & CAN_CS_TIME_STAMP_Msk) >> CAN_CS_TIME_STAMP_Pos);

        /*!< Store Message Payload. */
        pRxFrame->dataWord0 = flex_can->MB[0].WORD0;
        pRxFrame->dataWord1 = flex_can->MB[0].WORD1;

        /*!< Store ID Filter Hit Index. */
        pRxFrame->idhit = (uint16_t)(flex_can->RXFIR & CAN_RXFIR_IDHIT_Msk);

        /*!< Read free-running timer to unlock Rx Message Buffer. */
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
  * @param flex_can： FlexCAN peripheral flex_can pointer.
  * @param mbIdx： The FlexCAN Message Buffer index.
  * @param pTxFrame： Pointer to CAN message frame to be sent.
  * @retval Status_Flexcan_Success - Write Tx Message Buffer Successfully.
  *         Status_Flexcan_Fail    - Tx Message Buffer is currently in use.
  */
uint32_t FLEXCAN_TransferSendBlocking(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pTxFrame)
{
    uint32_t status;

    /*!< Write Tx Message Buffer to initiate a data sending. */
    if (Status_Flexcan_Success == FLEXCAN_WriteTxMb(flex_can, mbIdx, (const flexcan_frame_t *)(uint32_t)pTxFrame))
    {
        /*!< Wait until CAN Message send out. */
        uint32_t u32flag = 1;

        while (RESET == FLEXCAN_GetMbFlagStatus(flex_can, u32flag << mbIdx))
        {
        }

        /*!< Clean Tx Message Buffer Flag. */
        FLEXCAN_ClearMbFlag(flex_can, u32flag << mbIdx);

        /*!< After TX MB tranfered success, update the Timestamp from MB[mbIdx].CS register */
        pTxFrame->timestamp = (uint16_t)((flex_can->MB[mbIdx].CS & CAN_CS_TIME_STAMP_Msk) >> CAN_CS_TIME_STAMP_Pos);

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
  * @param flex_can： FlexCAN peripheral flex_can pointer.
  * @param mbIdx： The FlexCAN Message Buffer index.
  * @param pRxFrame： Pointer to CAN message frame structure for reception.
  * @retval Status_Flexcan_Success            - Rx Message Buffer is full and has been read successfully.
  *         Status_Flexcan_RxOverflow  - Rx Message Buffer is already overflowed and has been read successfully.
  *         Status_Flexcan_Fail               - Rx Message Buffer is empty.
  */
uint32_t FLEXCAN_TransferReceiveBlocking(Flex_CAN_TypeDef *flex_can, uint8_t mbIdx, flexcan_frame_t *pRxFrame)
{
    /*!< Wait until Rx Message Buffer non-empty. */
    uint32_t u32flag = 1;

    while (RESET == FLEXCAN_GetMbFlagStatus(flex_can, u32flag << mbIdx))
    {
    }

    /*!< Clean Rx Message Buffer Flag. */
    FLEXCAN_ClearMbFlag(flex_can, u32flag << mbIdx);

    /*!< Read Received CAN Message. */
    return (FLEXCAN_ReadRxMb(flex_can, mbIdx, pRxFrame));
}

/**
  * @brief Performs a polling receive transaction from Rx FIFO on the CAN bus.
  *
  * Note that a transfer handle does not need to be created  before calling this API.
  *
  * @param flex_can： FlexCAN peripheral flex_can pointer.
  * @param pRxFrame： Pointer to CAN message frame structure for reception.
  * @retval Status_Flexcan_Success - Read Message from Rx FIFO successfully.
  *         Status_Flexcan_Fail    - Rx FIFO is not enabled.
  */
uint32_t FLEXCAN_TransferReceiveFifoBlocking(Flex_CAN_TypeDef *flex_can, flexcan_frame_t *pRxFrame)
{
    uint32_t rxFifoStatus;

    /*!< Wait until Rx FIFO non-empty. */
    while (RESET == FLEXCAN_GetMbFlagStatus(flex_can, (uint32_t)Flexcan_RxFifoFrameAvlFlag))
    {
    }

    rxFifoStatus = FLEXCAN_ReadRxFifo(flex_can, pRxFrame);

    /*!< Clean Rx Fifo available flag. */
    FLEXCAN_ClearMbFlag(flex_can, (uint32_t)Flexcan_RxFifoFrameAvlFlag);

    return (rxFifoStatus);
}

/**
  * @brief Initializes the FlexCAN handle.
  *
  * This function initializes the FlexCAN handle, which can be used for other FlexCAN
  * transactional APIs. Usually, for a specified FlexCAN instance,
  * call this API once to get the initialized handle.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param handle: FlexCAN handle pointer.
  * @param callback: The callback function.
  * @param userData: The parameter of the callback function.
  */
void FLEXCAN_TransferCreateHandle(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_transfer_callback_t callback, void *userData)
{
    /*!< Clean FlexCAN transfer handle. */
    (void)memset(handle, 0, sizeof(*handle));

    /*!< Save the context in global variables to support the double weak mechanism. */
    /*!< s_flexcanHandle[instance] = handle; */

    /*!< Register Callback function. */
    handle->callback = callback;
    handle->userData = userData;

    /*!< s_flexcanIsr = FLEXCAN_TransferHandleIRQ; */

    /*!< We Enable Error & Status interrupt here, because this interrupt just */
    /*!< report current status of FlexCAN module through Callback function. */
    /*!< It is insignificance without a available callback function. */

    if (handle->callback != NULL)
    {
        FLEXCAN_ITConfig(
            flex_can, FLEXCAN_IT_BusOffInterrupt | FLEXCAN_IT_ErrorInterrupt |
            FLEXCAN_IT_RxWarningInterrupt | FLEXCAN_IT_TxWarningInterrupt |
            FLEXCAN_IT_WakeUpInterrupt, ENABLE);
    }
    else
    {
        FLEXCAN_ITConfig(
            flex_can, FLEXCAN_IT_BusOffInterrupt | FLEXCAN_IT_ErrorInterrupt |
            FLEXCAN_IT_RxWarningInterrupt | FLEXCAN_IT_TxWarningInterrupt |
            FLEXCAN_IT_WakeUpInterrupt, DISABLE);
    }
}

/**
  * @brief Sends a message using IRQ.
  *
  * This function sends a message using IRQ. This is a non-blocking function, which returns
  * right away. When messages have been sent out, the send callback function is called.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param handle: FlexCAN handle pointer.
  * @param pMbXfer: FlexCAN Message Buffer transfer structure.
  * @retval Status_Flexcan_Success        Start Tx Message Buffer sending process successfully.
  *         Status_Flexcan_Fail           Write Tx Message Buffer failed.
  *         Status_Flexcan_TxBusy         Tx Message Buffer is in use.
  */
uint32_t FLEXCAN_TransferSendNonBlocking(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    uint32_t status;

    /*!< Check if Message Buffer is idle. */
    if ((uint8_t)Flexcan_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
        /*!< Distinguish transmit type. */
        if ((uint32_t)Enum_Flexcan_FrameTypeRemote == pMbXfer->frame->type)
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)Flexcan_StateTxRemote;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)Flexcan_StateTxData;
        }

        if (Status_Flexcan_Success == FLEXCAN_WriteTxMb(flex_can, pMbXfer->mbIdx, (const flexcan_frame_t *)(uint32_t)pMbXfer->frame))
        {
            /*!< Enable Message Buffer Interrupt. */
            uint32_t u32mask = 1;

            FLEXCAN_MbInterruptCmd(flex_can, u32mask << pMbXfer->mbIdx, ENABLE);

            status = Status_Flexcan_Success;
        }
        else
        {
            handle->mbState[pMbXfer->mbIdx] = (uint8_t)Flexcan_StateIdle;
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
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param handle FlexCAN handle pointer.
  * @param pMbXfer: FlexCAN Message Buffer transfer structure.
  * @retval Status_Flexcan_Success        - Start Rx Message Buffer receiving process successfully.
  *         Status_Flexcan_RxBusy - Rx Message Buffer is in use.
  */
uint32_t FLEXCAN_TransferReceiveNonBlocking(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_mb_transfer_t *pMbXfer)
{
    uint32_t status;
    uint32_t u32mask = 1;

    /*!< Check if Message Buffer is idle. */
    if ((uint8_t)Flexcan_StateIdle == handle->mbState[pMbXfer->mbIdx])
    {
        handle->mbState[pMbXfer->mbIdx] = (uint8_t)Flexcan_StateRxData;

        /*!< Register Message Buffer. */
        handle->mbFrameBuf[pMbXfer->mbIdx] = pMbXfer->frame;

        /*!< Enable Message Buffer Interrupt. */
        FLEXCAN_MbInterruptCmd(flex_can, u32mask << pMbXfer->mbIdx, ENABLE);

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
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param handle: FlexCAN handle pointer.
  * @param pFifoXfer: FlexCAN Rx FIFO transfer structure. See the ref flexcan_fifo_transfer_t.
  * @retval Status_Flexcan_Success            - Start Rx FIFO receiving process successfully.
  *         Status_Flexcan_RxFifoBusy - Rx FIFO is currently in use.
  */
uint32_t FLEXCAN_TransferReceiveFifoNonBlocking(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, flexcan_fifo_transfer_t *pFifoXfer)
{
    uint32_t status;

    /*!< Check if Message Buffer is idle. */
    if ((uint8_t)Flexcan_StateIdle == handle->rxFifoState)
    {
        handle->rxFifoState = (uint8_t)Flexcan_StateRxFifo;

        /*!< Register Message Buffer. */
        handle->rxFifoFrameBuf = pFifoXfer->frame;

        /*!< Enable Message Buffer Interrupt. */
        FLEXCAN_MbInterruptCmd(flex_can, (uint32_t)Flexcan_RxFifoOverflowFlag | (uint32_t)Flexcan_RxFifoWarningFlag |
                               (uint32_t)Flexcan_RxFifoFrameAvlFlag, ENABLE);

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
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param handle: FlexCAN handle pointer.
  * @param mbIdx: The FlexCAN Message Buffer index.
  */
void FLEXCAN_TransferAbortSend(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, uint8_t mbIdx)
{
    uint16_t timestamp;

    /*!< Disable Message Buffer Interrupt. */
    uint32_t u32mask = 1;

    FLEXCAN_MbInterruptCmd(flex_can, u32mask << mbIdx, DISABLE);

    /*!< Update the TX frame 's time stamp by MB[mbIdx].cs. */
    timestamp                = (uint16_t)((flex_can->MB[mbIdx].CS & CAN_CS_TIME_STAMP_Msk) >> CAN_CS_TIME_STAMP_Pos);
    handle->timestamp[mbIdx] = timestamp;

    /*!< Clean Message Buffer. */
    FLEXCAN_TxMbConfig(flex_can, mbIdx, ENABLE);

    handle->mbState[mbIdx] = (uint8_t)Flexcan_StateIdle;
}

/**
  * @brief Aborts the interrupt driven message receive process.
  *
  * This function aborts the interrupt driven message receive process.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param handle: FlexCAN handle pointer.
  * @param mbIdx: The FlexCAN Message Buffer index.
  */
void FLEXCAN_TransferAbortReceive(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, uint8_t mbIdx)
{
    /*!< Disable Message Buffer Interrupt. */
    uint32_t u32mask = 1;

    FLEXCAN_MbInterruptCmd(flex_can, (u32mask << mbIdx), DISABLE);

    /*!< Un-register handle. */
    handle->mbFrameBuf[mbIdx] = NULL;
    handle->mbState[mbIdx]    = (uint8_t)Flexcan_StateIdle;
}

/**
  * @brief Aborts the interrupt driven message receive from Rx FIFO process.
  *
  * This function aborts the interrupt driven message receive from Rx FIFO process.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param handle: FlexCAN handle pointer.
  */
void FLEXCAN_TransferAbortReceiveFifo(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle)
{
    /*!< Check if Rx FIFO is enabled. */
    if (0U != (flex_can->MCR & CAN_MCR_RFEN_Msk))
    {
        /*!< Disable Rx Message FIFO Interrupts. */
        FLEXCAN_MbInterruptCmd(flex_can, (uint32_t)Flexcan_RxFifoOverflowFlag | (uint32_t)Flexcan_RxFifoWarningFlag |
                               (uint32_t)Flexcan_RxFifoFrameAvlFlag, DISABLE);

        /*!< Un-register handle. */
        handle->rxFifoFrameBuf = NULL;
    }

    handle->rxFifoState = (uint8_t)Flexcan_StateIdle;
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
  * @param handle: FlexCAN handle pointer.
  * @param mbIdx: The FlexCAN FD Message Buffer index.
  * @retval the index of mailbox 's timestamp stored in the handle.
  */
uint32_t FLEXCAN_GetTimeStamp(flexcan_handle_t *handle, uint8_t mbIdx)
{
    return ((uint32_t)(handle->timestamp[mbIdx]));
}

static ErrorStatus FLEXCAN_CheckUnhandleInterruptEvents(Flex_CAN_TypeDef *flex_can)
{
    uint32_t tempmask;
    uint32_t tempflag;
    ErrorStatus fgRet = ERROR;

    /*!< Checking exist error flag. */
    if (RESET == FLEXCAN_GetFlagStatus(flex_can, (FLEXCAN_FLAG_TxWarningInt | FLEXCAN_FLAG_RxWarningInt | FLEXCAN_FLAG_BusOffInt | FLEXCAN_FLAG_ErrorInt | FLEXCAN_FLAG_WakeUpInt)))
    {
        tempmask = (uint32_t)flex_can->IMASK1;
        tempflag = (uint32_t)flex_can->IFLAG1;

        fgRet = (ErrorStatus)(0U != (tempmask & tempflag));
    }
    else
    {
        fgRet = SUCCESS;
    }

    return (fgRet);
}

static uint32_t FLEXCAN_SubHandlerForDataTransfered(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle, uint32_t *pResult)
{
    uint32_t status  = Status_Flexcan_UnHandled;
    uint32_t result  = 0xFFU;
    uint32_t u32flag = 1;

    /*!< For this implementation, we solve the Message with lowest MB index first. */
    for (result = 0U; result < (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can); result++)
    {
        /*!< Get the lowest unhandled Message Buffer */
        uint32_t u32flag = 1;

        if (RESET != FLEXCAN_GetMbFlagStatus(flex_can, u32flag << result))
        {
            if (flex_can->IMASK1 & (1U << (uint8_t)result))
            {
                break;
            }
        }
    }

    /*!< find Message to deal with. */
    if (result < (uint32_t)FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(flex_can))
    {
        /*!< Solve Legacy Rx FIFO interrupt. */
        if (((uint8_t)Flexcan_StateIdle != handle->rxFifoState) && (result <= (uint32_t)CAN_IFLAG1_BUF7I_Pos))
        {
            uint32_t u32mask = 1;

            switch (u32mask << result)
            {
                case Flexcan_RxFifoOverflowFlag:
                    status = Status_Flexcan_RxFifoOverflow;
                    break;

                case Flexcan_RxFifoWarningFlag:
                    status = Status_Flexcan_RxFifoWarning;
                    break;

                case Flexcan_RxFifoFrameAvlFlag:
                    status = FLEXCAN_ReadRxFifo(flex_can, handle->rxFifoFrameBuf);

                    if (Status_Flexcan_Success == status)
                    {
                        /*!< Align the current (index 0) rxfifo timestamp to the timestamp array by handle. */
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
            /*!< Get current State of Message Buffer. */
            switch (handle->mbState[result])
            {
                /*!< Solve Rx Data Frame. */
                case (uint8_t)Flexcan_StateRxData:
                {
                    status = FLEXCAN_ReadRxMb(flex_can, (uint8_t)result, handle->mbFrameBuf[result]);

                    if (Status_Flexcan_Success == status)
                    {
                        /*!< Align the current index of RX MB timestamp to the timestamp array by handle. */
                        handle->timestamp[result] = handle->mbFrameBuf[result]->timestamp;
                        status                    = Status_Flexcan_RxIdle;
                    }
                }

                    {
                        FLEXCAN_TransferAbortReceive(flex_can, handle, (uint8_t)result);
                    }
                    break;

                /*!< Sove Rx Remote Frame.  User need to Read the frame in Mail box in time by Read from MB API. */
                case (uint8_t)Flexcan_StateRxRemote:
                    status = Status_Flexcan_RxRemote;

                    {
                        FLEXCAN_TransferAbortReceive(flex_can, handle, (uint8_t)result);
                    }
                    break;

                /*!< Solve Tx Data Frame. */
                case (uint8_t)Flexcan_StateTxData:
                    status = Status_Flexcan_TxIdle;

                    {
                        FLEXCAN_TransferAbortSend(flex_can, handle, (uint8_t)result);
                    }
                    break;

                /*!< Solve Tx Remote Frame. */
                case (uint8_t)Flexcan_StateTxRemote:
                    handle->mbState[result] = (uint8_t)Flexcan_StateRxRemote;
                    status                  = Status_Flexcan_TxSwitchToRx;
                    break;

                default:
                    status = Status_Flexcan_UnHandled;
                    break;
            }
        }

        /*!< Clear resolved Message Buffer IRQ. */
        FLEXCAN_ClearMbFlag(flex_can, u32flag << result);
    }

    *pResult = result;

    return (status);
}

/**
  * @brief FlexCAN IRQ handle function.
  *
  * This function handles the FlexCAN Error, the Message Buffer, and the Rx FIFO IRQ request.
  *
  * @param flex_can: FlexCAN peripheral Struct Point.
  * @param handle: FlexCAN handle pointer.
  */
void FLEXCAN_TransferHandleIRQ(Flex_CAN_TypeDef *flex_can, flexcan_handle_t *handle)
{
    uint32_t status;
    uint32_t result    = 0xFFU;
    uint32_t EsrStatus = 0U;

    do
    {
        /*!< To handle FlexCAN Error and Status Interrupt first. */
        if (RESET != FLEXCAN_GetFlagStatus(flex_can, (FLEXCAN_FLAG_TxWarningInt | FLEXCAN_FLAG_RxWarningInt |
                                                      FLEXCAN_FLAG_BusOffInt | FLEXCAN_FLAG_ErrorInt)))
        {
            status = Status_Flexcan_ErrorStatus;
            /*!< Clear FlexCAN Error and Status Interrupt. */
            FLEXCAN_ClearFlag(flex_can, FLEXCAN_FLAG_TxWarningInt | FLEXCAN_FLAG_RxWarningInt |
                              FLEXCAN_FLAG_BusOffInt | FLEXCAN_FLAG_ErrorInt);
            result = EsrStatus;
        }
        else if (RESET != FLEXCAN_GetFlagStatus(flex_can, FLEXCAN_FLAG_WakeUpInt))
        {
            status = Status_Flexcan_WakeUp;
            FLEXCAN_ClearFlag(flex_can, FLEXCAN_FLAG_WakeUpInt);
        }
        else
        {
            /*!< to handle real data transfer. */
            status = FLEXCAN_SubHandlerForDataTransfered(flex_can, handle, &result);
        }

        /*!< Calling Callback Function if has one. */
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

