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
  * @brief CAN Base Address Definition
  */
#define FLEX_CAN1_BASE                  (APB1PERIPH_BASE + 0xC000) /*!< Base Address: 0x4000C000 */

/*!< FLEXCAN   Typedef */
typedef struct
{
    __IO uint32_t MCR;                 /*!< offset: 0x00 Module Configuration Register, */
    __IO uint32_t CTRL1;               /*!< offset: 0x04 Control 1 register, */
    __IO uint32_t TIMER;               /*!< offset: 0x08 Free Running Timer, */
    uint32_t      RESERVED_0;
    __IO uint32_t RXMGMASK;            /*!< offset: 0x10 Rx Mailboxes Global Mask Register, */
    __IO uint32_t RX14MASK;            /*!< offset: 0x14 Rx 14 Mask register, */
    __IO uint32_t RX15MASK;            /*!< offset: 0x18 Rx 15 Mask register, */
    __IO uint32_t ECR;                 /*!< offset: 0x1C Error Counter, */
    __IO uint32_t ESR1;                /*!< offset: 0x20 Error and Status 1 register, */
    uint32_t      RESERVED_1;
    __IO uint32_t IMASK1;              /*!< offset: 0x28 Interrupt Masks 1 register, */
    uint32_t      RESERVED_2;
    __IO uint32_t IFLAG1;              /*!< offset: 0x30 Interrupt Flags 1 register, */
    __IO uint32_t CTRL2;               /*!< offset: 0x34 Control 2 register, */
    __I  uint32_t ESR2;                /*!< offset: 0x38 Error and Status 2 register, */
    uint32_t      RESERVED_3[2];
    __I  uint32_t CRCR;                /*!< offset: 0x44 CRC Register, */
    __IO uint32_t RXFGMASK;            /*!< offset: 0x48 Rx FIFO Global Mask register, */
    __I  uint32_t RXFIR;               /*!< offset: 0x4C Rx FIFO Information Register, */
    __IO uint32_t CBT;                 /*!< offset: 0x50 CAN Bit Timing Register, */
    uint32_t      RESERVED_4[11];

    struct                             /*!< offset: 0x80 array step: 0x10 x 128bit */
    {
        __IO uint32_t CS;              /*!< Message Buffer 0 CS Register..Message Buffer 15 CS Register, array offset: 0x80, array step: 0x10 */
        __IO uint32_t ID;              /*!< Message Buffer 0 ID Register..Message Buffer 15 ID Register, array offset: 0x84, array step: 0x10 */
        __IO uint32_t WORD0;           /*!< Message Buffer 0 WORD0 Register..Message Buffer 15 WORD0 Register, array offset: 0x88, array step: 0x10 */
        __IO uint32_t WORD1;           /*!< Message Buffer 0 WORD1 Register..Message Buffer 15 WORD1 Register, array offset: 0x8C, array step: 0x10 */
    }             MB[16];

    uint32_t      RESERVED_5[448];
    __IO uint32_t RXIMR[16];           /*!< offset: 0x880  Rx Individual Mask Registers */
} Flex_CAN_TypeDef;

/**
  * @brief CAN type pointer Definition
  */
#define FLEX_CAN1                       ((Flex_CAN_TypeDef *)FLEX_CAN1_BASE)

/**
  * @brief  CAN_MCR Control Register 1
  */
#define CAN_MCR_MDIS_Pos                (31)
#define CAN_MCR_MDIS_Msk                (0x1U << CAN_MCR_MDIS_Pos)
#define CAN_MCR_FRZ_Pos                 (30)
#define CAN_MCR_FRZ_Msk                 (0x1U << CAN_MCR_FRZ_Pos)
#define CAN_MCR_RFEN_Pos                (29)
#define CAN_MCR_RFEN_Msk                (0x1U << CAN_MCR_RFEN_Pos)
#define CAN_MCR_HALT_Pos                (28)
#define CAN_MCR_HALT_Msk                (0x1U << CAN_MCR_HALT_Pos)
#define CAN_MCR_NOTRDY_Pos              (27)
#define CAN_MCR_NOTRDY_Msk              (0x1U << CAN_MCR_NOTRDY_Pos)
#define CAN_MCR_WAKMSK_Pos              (26U)
#define CAN_MCR_WAKMSK_Msk              (0x1U << CAN_MCR_WAKMSK_Pos)
#define CAN_MCR_SOFTRST_Pos             (25)
#define CAN_MCR_SOFTRST_Msk             (0x1U << CAN_MCR_SOFTRST_Pos)
#define CAN_MCR_FRZACK_Pos              (24)
#define CAN_MCR_FRZACK_Msk              (0x1U << CAN_MCR_FRZACK_Pos)
#define CAN_MCR_SUPV_Pos                (23)
#define CAN_MCR_SUPV_Msk                (0x1U << CAN_MCR_SUPV_Pos)
#define CAN_MCR_SLFWAK_Pos              (22)
#define CAN_MCR_SLFWAK_Msk              (0x01U << CAN_MCR_SLFWAK_Pos)
#define CAN_MCR_WRNEN_Pos               (21)
#define CAN_MCR_WRNEN_Msk               (0x1U << CAN_MCR_WRNEN_Pos)
#define CAN_MCR_LPMACK_Pos              (20)
#define CAN_MCR_LPMACK_Msk              (0x1U << CAN_MCR_LPMACK_Pos)
#define CAN_MCR_WAKSRC_Pos              (19)
#define CAN_MCR_WAKSRC_Msk              (0x01U << CAN_MCR_WAKSRC_Pos)
#define CAN_MCR_SRXDIS_Pos              (17)
#define CAN_MCR_SRXDIS_Msk              (0x1U << CAN_MCR_SRXDIS_Pos)
#define CAN_MCR_IRMQ_Pos                (16)
#define CAN_MCR_IRMQ_Msk                (0x1U << CAN_MCR_IRMQ_Pos)
#define CAN_MCR_DMA_Pos                 (15)
#define CAN_MCR_DMA_Msk                 (0x1U << CAN_MCR_DMA_Pos)
#define CAN_MCR_LPRIOEN_Pos             (13)
#define CAN_MCR_LPRIOEN_Msk             (0x1U << CAN_MCR_LPRIOEN_Pos)
#define CAN_MCR_AEN_Pos                 (12)
#define CAN_MCR_AEN_Msk                 (0x1U << CAN_MCR_AEN_Pos)
#define CAN_MCR_FDEN_Pos                (11)
#define CAN_MCR_FDEN_Msk                (0x1U << CAN_MCR_FDEN_Pos)
#define CAN_MCR_IDAM_Pos                (8)
#define CAN_MCR_IDAM_Msk                (0x3U << CAN_MCR_IDAM_Pos)
#define CAN_MCR_MAXMB_Pos               (0)
#define CAN_MCR_MAXMB_Msk               (0x7FU << CAN_MCR_MAXMB_Pos)

/**
  * @brief  CAN_CTRL1 Control Register 1
  */
#define CAN_CTRL1_PROPSEG_Pos           (0)
#define CAN_CTRL1_PROPSEG_Msk           (0x07U << CAN_CTRL1_PROPSEG_Pos)  /*!< Propagation Segment */

#define CAN_CTRL1_LOM_Pos               (3)
#define CAN_CTRL1_LOM_Msk               (0x01U << CAN_CTRL1_LOM_Pos)      /*!< Listen-Only mode */
#define CAN_CTRL1_LBUF_Pos              (4)
#define CAN_CTRL1_LBUF_Msk              (0x01U << CAN_CTRL1_LBUF_Pos)     /*!< Lowest buffer transmission */
#define CAN_CTRL1_TSYN_Pos              (5)
#define CAN_CTRL1_TSYN_Msk              (0x01U << CAN_CTRL1_TSYN_Pos)     /*!< Timer Sync */
#define CAN_CTRL1_BOFFREC_Pos           (6)
#define CAN_CTRL1_BOFFREC_Msk           (0x01U << CAN_CTRL1_BOFFREC_Pos)  /*!< Defines how the FlexCAN recovers from Bus Off state */
#define CAN_CTRL1_SMP_Pos               (7)
#define CAN_CTRL1_SMP_Msk               (0x01U << CAN_CTRL1_SMP_Pos)      /*!< The sampling mode of CAN bits at the Rx input */

#define CAN_CTRL1_RWRNMASK_Pos          (10)
#define CAN_CTRL1_RWRNMASK_Msk          (0x01U << CAN_CTRL1_RWRNMASK_Pos) /*!< Rx Warning Interrupt Mask */
#define CAN_CTRL1_TWRNMASK_Pos          (11)
#define CAN_CTRL1_TWRNMASK_Msk          (0x01U << CAN_CTRL1_TWRNMASK_Pos) /*!< Tx Warning Interrupt Mask */
#define CAN_CTRL1_LPB_Pos               (12)
#define CAN_CTRL1_LPB_Msk               (0x01U << CAN_CTRL1_LPB_Pos)      /*!< Loop-Back */
#define CAN_CTRL1_CLKSRC_Pos            (13)
#define CAN_CTRL1_CLKSRC_Msk            (0x01U << CAN_CTRL1_CLKSRC_Pos)   /*!< Selects the clock source */
#define CAN_CTRL1_ERRMSK_Pos            (14)
#define CAN_CTRL1_ERRMSK_Msk            (0x01U << CAN_CTRL1_ERRMSK_Pos)   /*!< Error Interrupt Mask */
#define CAN_CTRL1_BOFFMSK_Pos           (15)
#define CAN_CTRL1_BOFFMSK_Msk           (0x01U << CAN_CTRL1_BOFFMSK_Pos)  /*!< Bus Off Interrupt Mask */
#define CAN_CTRL1_PSEG2_Pos             (16)
#define CAN_CTRL1_PSEG2_Msk             (0x07U << CAN_CTRL1_PSEG2_Pos)    /*!< Phase Segment 2 */
#define CAN_CTRL1_PSEG1_Pos             (19)
#define CAN_CTRL1_PSEG1_Msk             (0x07U << CAN_CTRL1_PSEG1_Pos)    /*!< Phase Segment 1 */
#define CAN_CTRL1_RJW_Pos               (22)
#define CAN_CTRL1_RJW_Msk               (0x03U << CAN_CTRL1_RJW_Pos)      /*!< Resync Jump Width */
#define CAN_CTRL1_PRESDIV_Pos           (24)
#define CAN_CTRL1_PRESDIV_Msk           (0xFFU << CAN_CTRL1_PRESDIV_Pos)  /*!< Prescaler Division Factor */

/**
  * @brief  CAN_TIMER Free Running Timer Register
  */
#define CAN_TIMER_TIMER_Pos             (0)
#define CAN_TIMER_TIMER_Msk             (0x0FFFFU << CAN_TIMER_TIMER_Pos) /*!< Timer Value */

/**
  * @brief  CAN_RXMGMASK Rx Mailboxes Global Mask Register
  */
#define CAN_RXMGMASK_MG_Pos             (0)
#define CAN_RXMGMASK_MG_Msk             (0xFFFFFFFFU << CAN_RXMGMASK_MG_Pos) /*!< Rx Mailboxes Global Mask Bits */

/**
  * @brief  CAN_RX14MASK Rx 14 Mask Register
  */
#define CAN_RX14MASK_RX14M_Pos          (0)
#define CAN_RX14MASK_RX14M_Msk          (0xFFFFFFFFU << CAN_RX14MASK_RX14M_Pos) /*!< Rx MB14 Mask Bits */

/**
  * @brief  CAN_RX15MASK Rx 15 Mask Register
  */
#define CAN_RX15MASK_RX15M_Pos          (0)
#define CAN_RX15MASK_RX15M_Msk          (0xFFFFFFFFU << CAN_RX15MASK_RX15M_Pos) /*!< Rx MB15 Mask Bits */

/**
  * @brief  CAN_ECR Error Counter Register
  */
#define CAN_ECR_TXERRCNT_Pos            (0)
#define CAN_ECR_TXERRCNT_Msk            (0xFFU << CAN_ECR_TXERRCNT_Pos) /*!< Transmit Error Counter */
#define CAN_ECR_RXERRCNT_Pos            (8)
#define CAN_ECR_RXERRCNT_Msk            (0xFFU << CAN_ECR_RXERRCNT_Pos) /*!< Receive Error Counter */

/**
  * @brief  CAN_ESR1 Error and Status Register 1
  */
#define CAN_ESR1_WAKINT_Pos             (0U)
#define CAN_ESR1_WAKINT_Msk             (0x01U << CAN_ESR1_WAKINT_Pos)
#define CAN_ESR1_ERRINT_Pos             (1)
#define CAN_ESR1_ERRINT_Msk             (0x01U << CAN_ESR1_ERRINT_Pos)      /*!< Error Interrupt */
#define CAN_ESR1_BOFFINT_Pos            (2)
#define CAN_ESR1_BOFFINT_Msk            (0x01U << CAN_ESR1_BOFFINT_Pos)     /*!< Bus Off Interrupt */
#define CAN_ESR1_RX_Pos                 (3)
#define CAN_ESR1_RX_Msk                 (0x01U << CAN_ESR1_RX_Pos)          /*!< Receiving a Message */
#define CAN_ESR1_FLTCONF_Pos            (4)
#define CAN_ESR1_FLTCONF_Msk            (0x03U << CAN_ESR1_FLTCONF_Pos)     /*!< Fault State of the FlexCAN module */
#define CAN_ESR1_TX_Pos                 (6)
#define CAN_ESR1_TX_Msk                 (0x01U << CAN_ESR1_TX_Pos)          /*!< Transmitting a Message */
#define CAN_ESR1_IDLE_Pos               (7)
#define CAN_ESR1_IDLE_Msk               (0x01U << CAN_ESR1_IDLE_Pos)        /*!< IDLE STATE */
#define CAN_ESR1_RXWRN_Pos              (8)
#define CAN_ESR1_RXWRN_Msk              (0x01U << CAN_ESR1_RXWRN_Pos)       /*!< Rx Error Warning */
#define CAN_ESR1_TXWRN_Pos              (9)
#define CAN_ESR1_TXWRN_Msk              (0x01U << CAN_ESR1_TXWRN_Pos)       /*!< Tx Error Warning */
#define CAN_ESR1_STFERR_Pos             (10)
#define CAN_ESR1_STFERR_Msk             (0x01U << CAN_ESR1_STFERR_Pos)      /*!< Stuffing Error */
#define CAN_ESR1_FRMERR_Pos             (11)
#define CAN_ESR1_FRMERR_Msk             (0x01U << CAN_ESR1_FRMERR_Pos)      /*!< Form Error */
#define CAN_ESR1_CRCERR_Pos             (12)
#define CAN_ESR1_CRCERR_Msk             (0x01U << CAN_ESR1_CRCERR_Pos)      /*!< Cyclic Redundancy Check Error */
#define CAN_ESR1_ACKERR_Pos             (13)
#define CAN_ESR1_ACKERR_Msk             (0x01U << CAN_ESR1_ACKERR_Pos)      /*!< Acknowledge Error */
#define CAN_ESR1_BIT0ERR_Pos            (14)
#define CAN_ESR1_BIT0ERR_Msk            (0x01U << CAN_ESR1_BIT0ERR_Pos)     /*!< Bit0 Error */
#define CAN_ESR1_BIT1ERR_Pos            (15)
#define CAN_ESR1_BIT1ERR_Msk            (0x01U << CAN_ESR1_BIT1ERR_Pos)     /*!< Bit1 Error */
#define CAN_ESR1_RWRNINT_Pos            (16)
#define CAN_ESR1_RWRNINT_Msk            (0x01U << CAN_ESR1_RWRNINT_Pos)     /*!< Rx Warning Interrupt Flag */
#define CAN_ESR1_TWRNINT_Pos            (17)
#define CAN_ESR1_TWRNINT_Msk            (0x01U << CAN_ESR1_TWRNINT_Pos)     /*!< Tx Warning Interrupt Flag */
#define CAN_ESR1_SYNCH_Pos              (18)
#define CAN_ESR1_SYNCH_Msk              (0x01U << CAN_ESR1_SYNCH_Pos)       /*!< CAN Synchronization Status */
#define CAN_ESR1_BOFFDONEINT_Pos        (19)
#define CAN_ESR1_BOFFDONEINT_Msk        (0x01U << CAN_ESR1_BOFFDONEINT_Pos) /*!< Bus Off Done Interrupt */

#define CAN_ESR1_ERROVR_Pos             (21)
#define CAN_ESR1_ERROVR_Msk             (0x01U << CAN_ESR1_ERROVR_Pos)      /*!< Error Overrun */

/**
  * @brief  CAN_IMASK1 Interrupt Mask Register 1
  */
#define CAN_IMASK1_BUF15TO0M_Pos        (0)
#define CAN_IMASK1_BUF15TO0M_Msk        (0xFFFFU << CAN_IMASK1_BUF15TO0M_Pos) /*!< MBi Mask Bits - Each bit enables or disables the corresponding interrupt for MB15 to MB0 */

/**
  * @brief  CAN_IFLAG1 Interrupt Flag Register 1
  */
#define CAN_IFLAG1_BUF0I_Pos            (0)
#define CAN_IFLAG1_BUF0I_Msk            (0x01U << CAN_IFLAG1_BUF0I_Pos)     /*!< MB0 Interrupt or Clear FIFO bit */
#define CAN_IFLAG1_BUF4TO1I_Pos         (1)
#define CAN_IFLAG1_BUF4TO1I_Msk         (0x0FU << CAN_IFLAG1_BUF4TO1I_Pos)  /*!< MB4~1 Interrupt or Reserved */
#define CAN_IFLAG1_BUF4TO1I_1           (0x01U << CAN_IFLAG1_BUF4TO1I_Pos)  /*!< MB1 Interrupt or Reserved */
#define CAN_IFLAG1_BUF4TO1I_2           (0x02U << CAN_IFLAG1_BUF4TO1I_Pos)  /*!< MB2 Interrupt or Reserved */
#define CAN_IFLAG1_BUF4TO1I_3           (0x04U << CAN_IFLAG1_BUF4TO1I_Pos)  /*!< MB3 Interrupt or Reserved */
#define CAN_IFLAG1_BUF4TO1I_4           (0x08U << CAN_IFLAG1_BUF4TO1I_Pos)  /*!< MB4 Interrupt or Reserved */
#define CAN_IFLAG1_BUF5I_Pos            (5)
#define CAN_IFLAG1_BUF5I_Msk            (0x01U << CAN_IFLAG1_BUF5I_Pos)     /*!< MB5 Interrupt or Frames Available in Rx FIFO */
#define CAN_IFLAG1_BUF6I_Pos            (6)
#define CAN_IFLAG1_BUF6I_Msk            (0x01U << CAN_IFLAG1_BUF6I_Pos)     /*!< MB6 Interrupt or Rx FIFO Warning */
#define CAN_IFLAG1_BUF7I_Pos            (7)
#define CAN_IFLAG1_BUF7I_Msk            (0x01U << CAN_IFLAG1_BUF7I_Pos)     /*!< MB7 Interrupt or Rx FIFO Overflow */
#define CAN_IFLAG1_BUF15TO8I_Pos        (8)
#define CAN_IFLAG1_BUF15TO8I_Msk        (0xFFU << CAN_IFLAG1_BUF15TO8I_Pos) /*!< MBi Interrupt Flag Bits - Each bit flags the corresponding Message Buffer Interrupt for MB15 to MB8 */
#define CAN_IFLAG1_BUF15TO8I_8          (0x01U << CAN_IFLAG1_BUF15TO8I_Pos) /*!< MB8 Interrupt Flag Bit */
#define CAN_IFLAG1_BUF15TO8I_9          (0x02U << CAN_IFLAG1_BUF15TO8I_Pos) /*!< MB9 Interrupt Flag Bit */
#define CAN_IFLAG1_BUF15TO8I_10         (0x04U << CAN_IFLAG1_BUF15TO8I_Pos) /*!< MB10 Interrupt Flag Bit */
#define CAN_IFLAG1_BUF15TO8I_11         (0x08U << CAN_IFLAG1_BUF15TO8I_Pos) /*!< MB11 Interrupt Flag Bit */
#define CAN_IFLAG1_BUF15TO8I_12         (0x10U << CAN_IFLAG1_BUF15TO8I_Pos) /*!< MB12 Interrupt Flag Bit */
#define CAN_IFLAG1_BUF15TO8I_13         (0x20U << CAN_IFLAG1_BUF15TO8I_Pos) /*!< MB13 Interrupt Flag Bit */
#define CAN_IFLAG1_BUF15TO8I_14         (0x40U << CAN_IFLAG1_BUF15TO8I_Pos) /*!< MB14 Interrupt Flag Bit */
#define CAN_IFLAG1_BUF15TO8I_15         (0x80U << CAN_IFLAG1_BUF15TO8I_Pos) /*!< MB15 Interrupt Flag Bit */

/**
  * @brief  CAN_CTRL2 Control Register 2
  */
#define CAN_CTRL2_EACEN_Pos             (16)
#define CAN_CTRL2_EACEN_Msk             (0x01U << CAN_CTRL2_EACEN_Pos)       /*!< Entire Frame Arbitration Field Comparison Enable For Rx Mailboxes */
#define CAN_CTRL2_RRS_Pos               (17)
#define CAN_CTRL2_RRS_Msk               (0x01U << CAN_CTRL2_RRS_Pos)         /*!< Remote Request Storing */
#define CAN_CTRL2_MRP_Pos               (18)
#define CAN_CTRL2_MRP_Msk               (0x01U << CAN_CTRL2_MRP_Pos)         /*!< Mailboxes Reception Priority */
#define CAN_CTRL2_TASD_Pos              (19)
#define CAN_CTRL2_TASD_Msk              (0x1FU << CAN_CTRL2_TASD_Pos)        /*!< Tx Arbitration Start Delay */
#define CAN_CTRL2_RFFN_Pos              (24)
#define CAN_CTRL2_RFFN_Msk              (0x0FU << CAN_CTRL2_RFFN_Pos)        /*!< The number of Rx FIFO filters */

#define CAN_CTRL2_BOFFDONEMSK_Pos       (30)
#define CAN_CTRL2_BOFFDONEMSK_Msk       (0x01U << CAN_CTRL2_BOFFDONEMSK_Pos) /*!< Bus Off Done Interrupt Mask */

/**
  * @brief  CAN_ESR2 Error and Status Register 2
  */
#define CAN_ESR2_IMB_Pos                (13)
#define CAN_ESR2_IMB_Msk                (0x01U << CAN_ESR2_IMB_Pos)  /*!< Inactive Mailbox */
#define CAN_ESR2_VPS_Pos                (14)
#define CAN_ESR2_VPS_Msk                (0x01U << CAN_ESR2_VPS_Pos)  /*!< Indicates whether CAN_ESR2.IMB and CAN_ESR2.LPTM contents are currently valid or not */

#define CAN_ESR2_LPTM_Pos               (16)
#define CAN_ESR2_LPTM_Msk               (0x7FU << CAN_ESR2_LPTM_Pos) /*!< Lowest Priority Tx Mailbox */

/**
  * @brief  CAN_CRCR CRC Register
  */
#define CAN_CRCR_TXCRC_Pos              (0)
#define CAM_CRCR_TXCRC_Msk              (0x7FFFU << CAN_CRCR_TXCRC_Pos) /*!< Transmitted CRC Value */

#define CAN_CRCR_MBCRC_Pos              (16)
#define CAN_CRCR_MBCRC_Msk              (0x7FU << CAN_CRCR_MBCRC_Pos)   /*!< CRC Mailbox */

/**
  * @brief  CAN_RXFGMASK Rx FIFO Global Mask Register
  */
#define CAN_RXFGMASK_FGM_Pos            (0)
#define CAN_RXFGMASK_FGM_Msk            (0xFFFFFFFFU << CAN_RXFGMASK_FGM_Pos) /*!< Rx FIFO Global Mask Bits */

/**
  * @brief  CAN_RXFIR Rx FIFO Information Register
  */
#define CAN_RXFIR_IDHIT_Pos             (0)
#define CAN_RXFIR_IDHIT_Msk             (0x1FFU << CAN_RXFIR_IDHIT_Pos) /*!< ID Acceptance Filter Hit Indicator */

/**
  * @brief  CAN_CBT CAN Bit Timing Register
  */
#define CAN_CBT_EPSEG2_Pos              (0)
#define CAN_CBT_EPSEG2_Msk              (0x1FU << CAN_CBT_EPSEG2_Pos)    /*!< Extended Phase Segment 2 */
#define CAN_CBT_EPSEG1_Pos              (5)
#define CAN_CBT_EPSEG1_Msk              (0x1FU << CAN_CBT_EPSEG1_Pos)    /*!< Extended Phase Segment 1 */
#define CAN_CBT_EPROPSEG_Pos            (10)
#define CAN_CBT_EPROPSEG_Msk            (0x3FU << CAN_CBT_EPROPSEG_Pos)  /*!< Extended Propagation Segment */
#define CAN_CBT_ERJW_Pos                (16)
#define CAN_CBT_ERJW_Msk                (0x1FU << CAN_CBT_ERJW_Pos)      /*!< Extended Resync Jump Width */
#define CAN_CBT_EPRESDIV_Pos            (21)
#define CAN_CBT_EPRESDIV_Msk            (0x3FFU << CAN_CBT_EPRESDIV_Pos) /*!< Extended Prescaler Division Factor */
#define CAN_CBT_BTF_Pos                 (31)
#define CAN_CBT_BTF_Msk                 (0x01U << CAN_CBT_BTF_Pos)       /*!< Bit Timing Format */

/**
  * @brief  CAN_RXIMR0 Rx Individual Mask Registers
  */
#define CAN_RXIMR0_MI_Pos               (0)
#define CAN_RXIMR0_MI_Msk               (0xFFFFFFFFU << CAN_RXIMR0_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR1 Rx Individual Mask Registers
  */
#define CAN_RXIMR1_MI_Pos               (0)
#define CAN_RXIMR1_MI_Msk               (0xFFFFFFFFU << CAN_RXIMR1_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR2 Rx Individual Mask Registers
  */
#define CAN_RXIMR2_MI_Pos               (0)
#define CAN_RXIMR2_MI_Msk               (0xFFFFFFFFU << CAN_RXIMR2_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR3 Rx Individual Mask Registers
  */
#define CAN_RXIMR3_MI_Pos               (0)
#define CAN_RXIMR3_MI_Msk               (0xFFFFFFFFU << CAN_RXIMR3_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR4 Rx Individual Mask Registers
  */
#define CAN_RXIMR4_MI_Pos               (0)
#define CAN_RXIMR4_MI_Msk               (0xFFFFFFFFU << CAN_RXIMR4_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR5 Rx Individual Mask Registers
  */
#define CAN_RXIMR5_MI_Pos               (0)
#define CAN_RXIMR5_MI_Msk               (0xFFFFFFFFU << CAN_RXIMR5_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR6 Rx Individual Mask Registers
  */
#define CAN_RXIMR6_MI_Pos               (0)
#define CAN_RXIMR6_MI_Msk               (0xFFFFFFFFU << CAN_RXIMR6_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR7 Rx Individual Mask Registers
  */
#define CAN_RXIMR7_MI_Pos               (0)
#define CAN_RXIMR7_MI_Msk               (0xFFFFFFFFU << CAN_RXIMR7_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR8 Rx Individual Mask Registers
  */
#define CAN_RXIMR8_MI_Pos               (0)
#define CAN_RXIMR8_MI_Msk               (0xFFFFFFFFU << CAN_RXIMR8_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR9 Rx Individual Mask Registers
  */
#define CAN_RXIMR9_MI_Pos               (0)
#define CAN_RXIMR9_MI_Msk               (0xFFFFFFFFU << CAN_RXIMR9_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR10 Rx Individual Mask Registers
  */
#define CAN_RXIMR10_MI_Pos              (0)
#define CAN_RXIMR10_MI_Msk              (0xFFFFFFFFU << CAN_RXIMR10_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR11 Rx Individual Mask Registers
  */
#define CAN_RXIMR11_MI_Pos              (0)
#define CAN_RXIMR11_MI_Msk              (0xFFFFFFFFU << CAN_RXIMR11_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR12 Rx Individual Mask Registers
  */
#define CAN_RXIMR12_MI_Pos              (0)
#define CAN_RXIMR12_MI_Msk              (0xFFFFFFFFU << CAN_RXIMR12_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR13 Rx Individual Mask Registers
  */
#define CAN_RXIMR13_MI_Pos              (0)
#define CAN_RXIMR13_MI_Msk              (0xFFFFFFFFU << CAN_RXIMR13_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR14 Rx Individual Mask Registers
  */
#define CAN_RXIMR14_MI_Pos              (0)
#define CAN_RXIMR14_MI_Msk              (0xFFFFFFFFU << CAN_RXIMR14_MI_Pos) /*!< Individual Mask Bits */

/**
  * @brief  CAN_RXIMR15 Rx Individual Mask Registers
  */
#define CAN_RXIMR15_MI_Pos              (0)
#define CAN_RXIMR15_MI_Msk              (0xFFFFFFFFU << CAN_RXIMR15_MI_Pos) /*!< Individual Mask Bits */

/** @name CS - Message Buffer 0 CS Register..Message Buffer 63 CS Register
  * @{
  */
#define CAN_CS_TIME_STAMP_Msk           (0xFFFFU)
#define CAN_CS_TIME_STAMP_Pos           (0U)
#define CAN_CS_DLC_Msk                  (0xF0000U)
#define CAN_CS_DLC_Pos                  (16U)
#define CAN_CS_RTR_Msk                  (0x100000U)
#define CAN_CS_RTR_Pos                  (20U)
#define CAN_CS_IDE_Msk                  (0x200000U)
#define CAN_CS_IDE_Pos                  (21U)
#define CAN_CS_SRR_Msk                  (0x400000U)
#define CAN_CS_SRR_Pos                  (22U)
#define CAN_CS_CODE_Msk                 (0xF000000U)
#define CAN_CS_CODE_Pos                 (24U)
/**
  * @}
  */

/** @name ID - Message Buffer 0 ID Register..Message Buffer 63 ID Register
  * @{
  */
#define CAN_ID_EXT_Msk                  (0x3FFFFU)
#define CAN_ID_EXT_Pos                  (0U)
#define CAN_ID_STD_Msk                  (0x1FFC0000U)
#define CAN_ID_STD_Pos                  (18U)
#define CAN_ID_PRIO_Msk                 (0xE0000000U)
#define CAN_ID_PRIO_Pos                 (29U)
/**
  * @}
  */

/**
  * @brief  CAN_FDCTRL CAN FD Control Registers
  */
#define CAN_FDCTRL_FDRATE_Pos           (31)
#define CAN_FDCTRL_FDRATE_Msk           (0x1U << CAN_FDCTRL_FDRATE_Pos)
#define CAN_FDCTRL_MBDSR0_Pos           (16)
#define CAN_FDCTRL_MBDSR0_Msk           (0x3U << CAN_FDCTRL_MBDSR0_Pos)
#define CAN_FDCTRL_TDCEN_Pos            (15)
#define CAN_FDCTRL_TDCEN_Msk            (0x1U << CAN_FDCTRL_TDCEN_Pos)
#define CAN_FDCTRL_TDCFAIL_Pos          (14)
#define CAN_FDCTRL_TDCFAIL_Msk          (0x1U << CAN_FDCTRL_TDCFAIL_Pos)
#define CAN_FDCTRL_TDCOFF_Pos           (8)
#define CAN_FDCTRL_TDCOFF_Msk           (0x1FU << CAN_FDCTRL_TDCOFF_Pos)
#define CAN_FDCTRL_TDCVAL_Pos           (0)
#define CAN_FDCTRL_TDCVAL_Msk           (0x3FU << CAN_FDCTRL_TDCVAL_Pos)

/**
  * @brief  CAN_FDCBT CAN FD Control Registers
  */
#define CAN_FDCBT_FPRESDIV_Pos          (20)
#define CAN_FDCBT_FPRESDIV_Msk          (0x3FFU << CAN_FDCBT_FPRESDIV_Pos)
#define CAN_FDCBT_FRJW_Pos              (16)
#define CAN_FDCBT_FRJW_Msk              (0x7U << CAN_FDCBT_FRJW_Pos)
#define CAN_FDCBT_FPROPSEG_Pos          (10)
#define CAN_FDCBT_FPROPSEG_Msk          (0x1FU << CAN_FDCBT_FPROPSEG_Pos)
#define CAN_FDCBT_FPSEG1_Pos            (5)
#define CAN_FDCBT_FPSEG1_Msk            (0x7U << CAN_FDCBT_FPSEG1_Pos)
#define CAN_FDCBT_FPSEG2_Pos            (0)
#define CAN_FDCBT_FPSEG2_Msk            (0x7U << CAN_FDCBT_FPSEG2_Pos)

/**
  * @brief  CAN_FDCRC CAN FD Control Registers
  */
#define CAN_FDCRC_FDMBCRC_Pos           (24)
#define CAN_FDCRC_FDMBCRC_Msk           (0x7FU << CAN_FDCRC_FDMBCRC_Pos)
#define CAN_FDCRC_FDTXCRC_Pos           (0)
#define CAN_FDCRC_FDTXCRC_Msk           (0x1FFFFFU << CAN_FDCRC_FDTXCRC_Pos)

/**
  * @brief  CAN_ERFCR Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFCR_ERFEN_Pos             (31)
#define CAN_ERFCR_ERFEN_Msk             (0x1U << CAN_ERFCR_ERFEN_Pos)

#define CAN_ERFCR_DMALW_Pos             (26)
#define CAN_ERFCR_DMALW_Msk             (0x1FU << CAN_ERFCR_DMALW_Pos)

#define CAN_ERFCR_NEXIF_Pos             (16)
#define CAN_ERFCR_NEXIF_Msk             (0x1U << CAN_ERFCR_NEXIF_Pos)

#define CAN_ERFCR_ERFWM_Pos             (0)
#define CAN_ERFCR_ERFWM_Msk             (0x7U << CAN_ERFCR_ERFWM_Pos)

/**
  * @brief  CAN_ERFIER Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFIER_ERFUFWIE_Pos         (31)
#define CAN_ERFIER_ERFUFWIE_Msk         (0x1U << CAN_ERFIER_ERFUFWIE_Pos)

#define CAN_ERFIER_ERFOVFIE_Pos         (30)
#define CAN_ERFIER_ERFOVFIE_Msk         (0x1U << CAN_ERFIER_ERFOVFIE_Pos)

#define CAN_ERFIER_ERFWMIIE_Pos         (29)
#define CAN_ERFIER_ERFWMIIE_Msk         (0x1U << CAN_ERFIER_ERFWMIIE_Pos)

#define CAN_ERFIER_ERFDAIE_Pos          (28)
#define CAN_ERFIER_ERFDAIE_Msk          (0x1U << CAN_ERFIER_ERFDAIE_Pos)

/**
  * @brief  CAN_ERFSR Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFSR_ERFUFW_Pos            (31)
#define CAN_ERFSR_ERFUFW_Msk            (0x1U << CAN_ERFSR_ERFUFW_Pos)

#define CAN_ERFSR_ERFOVF_Pos            (30)
#define CAN_ERFSR_ERFOVF_Msk            (0x1U << CAN_ERFSR_ERFOVF_Pos)

#define CAN_ERFSR_ERFWMI_Pos            (29)
#define CAN_ERFSR_ERFWMI_Msk            (0x1U << CAN_ERFSR_ERFWMI_Pos)

#define CAN_ERFSR_ERFDA_Pos             (28)
#define CAN_ERFSR_ERFDA_Msk             (0x1U << CAN_ERFSR_ERFDA_Pos)

#define CAN_ERFSR_ERFCLR_Pos            (27)
#define CAN_ERFSR_ERFCLR_Msk            (0x1U << CAN_ERFSR_ERFCLR_Pos)

#define CAN_ERFSR_ERFE_Pos              (17)
#define CAN_ERFSR_ERFE_Msk              (0x1U << CAN_ERFSR_ERFE_Pos)

#define CAN_ERFSR_ERFF_Pos              (16)
#define CAN_ERFSR_ERFF_Msk              (0x1U << CAN_ERFSR_ERFF_Pos)

#define CAN_ERFSR_ERFEL_Pos             (0)
#define CAN_ERFSR_ERFEL_Msk             (0x7U << CAN_ERFSR_ERFEL_Pos)

/**
  * @brief  CAN_ERFFEL0 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL0_FEL_Pos             (0)
#define CAN_ERFFEL0_FEL_Msk             (0xFFFFFFFFU << CAN_ERFFEL0_FEL_Pos)

/**
  * @brief  CAN_ERFFEL1 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL1_FEL_Pos             (0)
#define CAN_ERFFEL1_FEL_Msk             (0xFFFFFFFFU << CAN_ERFFEL1_FEL_Pos)

/**
  * @brief  CAN_ERFFEL2 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL2_FEL_Pos             (0)
#define CAN_ERFFEL2_FEL_Msk             (0xFFFFFFFFU << CAN_ERFFEL2_FEL_Pos)

/**
  * @brief  CAN_ERFFEL3 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL3_FEL_Pos             (0)
#define CAN_ERFFEL3_FEL_Msk             (0xFFFFFFFFU << CAN_ERFFEL3_FEL_Pos)

/**
  * @brief  CAN_ERFFEL4 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL4_FEL_Pos             (0)
#define CAN_ERFFEL4_FEL_Msk             (0xFFFFFFFFU << CAN_ERFFEL4_FEL_Pos)

/**
  * @brief  CAN_ERFFEL5 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL5_FEL_Pos             (0)
#define CAN_ERFFEL5_FEL_Msk             (0xFFFFFFFFU << CAN_ERFFEL5_FEL_Pos)

/**
  * @brief  CAN_ERFFEL6 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL6_FEL_Pos             (0)
#define CAN_ERFFEL6_FEL_Msk             (0xFFFFFFFFU << CAN_ERFFEL6_FEL_Pos)

/**
  * @brief  CAN_ERFFEL7 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL7_FEL_Pos             (0)
#define CAN_ERFFEL7_FEL_Msk             (0xFFFFFFFFU << CAN_ERFFEL7_FEL_Pos)

/**
  * @brief  CAN_ERFFEL8 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL8_FEL_Pos             (0)
#define CAN_ERFFEL8_FEL_Msk             (0xFFFFFFFFU << CAN_ERFFEL8_FEL_Pos)

/**
  * @brief  CAN_ERFFEL9 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL9_FEL_Pos             (0)
#define CAN_ERFFEL9_FEL_Msk             (0xFFFFFFFFU << CAN_ERFFEL9_FEL_Pos)

/**
  * @brief  CAN_ERFFEL10 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL10_FEL_Pos            (0)
#define CAN_ERFFEL10_FEL_Msk            (0xFFFFFFFFU << CAN_ERFFEL10_FEL_Pos)

/**
  * @brief  CAN_ERFFEL11 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL11_FEL_Pos            (0)
#define CAN_ERFFEL11_FEL_Msk            (0xFFFFFFFFU << CAN_ERFFEL11_FEL_Pos)

/**
  * @brief  CAN_ERFFEL12 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL12_FEL_Pos            (0)
#define CAN_ERFFEL12_FEL_Msk            (0xFFFFFFFFU << CAN_ERFFEL12_FEL_Pos)

/**
  * @brief  CAN_ERFFEL13 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL13_FEL_Pos            (0)
#define CAN_ERFFEL13_FEL_Msk            (0xFFFFFFFFU << CAN_ERFFEL13_FEL_Pos)

/**
  * @brief  CAN_ERFFEL14 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL14_FEL_Pos            (0)
#define CAN_ERFFEL14_FEL_Msk            (0xFFFFFFFFU << CAN_ERFFEL14_FEL_Pos)

/**
  * @brief  CAN_ERFFEL15 Enhanced Rx FIFO Control Registers
  */
#define CAN_ERFFEL15_FEL_Pos            (0)
#define CAN_ERFFEL15_FEL_Msk            (0xFFFFFFFFU << CAN_ERFFEL15_FEL_Pos)

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif

