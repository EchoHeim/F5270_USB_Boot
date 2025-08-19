/***********************************************************************************************************************
    @file    platform.c
    @author  FAE Team
    @date    1-Sep-2023
    @brief   THIS FILE PROVIDES ALL THE SYSTEM FUNCTIONS.
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

/* Define to prevent recursive inclusion */
#define _PLATFORM_C_

/* Files include */
#include <stdio.h>
#include "platform.h"

/**
  * @addtogroup MM32F5270_LibSamples
  * @{
  */

/**
  * @addtogroup ADC
  * @{
  */

/**
  * @addtogroup ADC_AnalogWatchdog
  * @{
  */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/

/***********************************************************************************************************************
  * @brief  Initialize SysTick for delay function
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void PLATFORM_InitDelay(void)
{
    RCC_ClocksTypeDef  RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    if (SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000))
    {
        while (1)
        {
        }
    }

    NVIC_SetPriority(SysTick_IRQn, 0x0);
}

/***********************************************************************************************************************
  * @brief  Millisecond delay
  * @note   none
  * @param  Millisecond: delay time unit
  * @retval none
  *********************************************************************************************************************/
void PLATFORM_DelayMS(uint32_t Millisecond)
{
    PLATFORM_DelayTick = Millisecond;

    while (0 != PLATFORM_DelayTick)
    {
    }
}

/***********************************************************************************************************************
  * @brief  Initialize console for printf
  * @note   none
  * @param  Baudrate : UART2 communication baudrate
  * @retval none
  *********************************************************************************************************************/
void PLATFORM_InitConsole(uint32_t Baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    UART_InitTypeDef UART_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);

    UART_StructInit(&UART_InitStruct);
    UART_InitStruct.BaudRate      = Baudrate;
    UART_InitStruct.WordLength    = UART_WordLength_8b;
    UART_InitStruct.StopBits      = UART_StopBits_1;
    UART_InitStruct.Parity        = UART_Parity_No;
    UART_InitStruct.HWFlowControl = UART_HWFlowControl_None;
    UART_InitStruct.Mode          = UART_Mode_Tx;
    UART_Init(UART2, &UART_InitStruct);

    UART_Cmd(UART2, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

#if   defined (__ICCARM__)

#if   (__VER__ >= 9030001)

/* Files include */
#include <stddef.h>
#include <LowLevelIOInterface.h>

/***********************************************************************************************************************
  * @brief  redefine __write function
  * @note   for printf
  * @param  handle
  * @param  *buf
  * @param  bufSize
  * @retval nChars
  *********************************************************************************************************************/
size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
    size_t nChars = 0;

    /* Check for the command to flush all handles */
    if (-1 == handle)
    {
        return (0);
    }

    /* Check for stdout and stderr (only necessary if FILE descriptors are enabled.) */
    if ((_LLIO_STDOUT != handle) && (_LLIO_STDERR != handle))
    {
        return (-1);
    }

    for (/* Empty */; bufSize > 0; --bufSize)
    {
        UART_SendData(UART2, *buf);

        while (RESET == UART_GetFlagStatus(UART2, UART_FLAG_TXC))
        {
        }

        ++buf;
        ++nChars;
    }

    return (nChars);
}

#else

/***********************************************************************************************************************
  * @brief  redefine fputc function
  * @note   for printf
  * @param  ch
  * @param  f
  * @retval ch
  *********************************************************************************************************************/
int fputc(int ch, FILE *f)
{
    UART_SendData(UART2, (uint8_t)ch);

    while (RESET == UART_GetFlagStatus(UART2, UART_FLAG_TXC))
    {
    }

    return (ch);
}

#endif

#elif defined (__GNUC__)

/***********************************************************************************************************************
  * @brief  redefine fputc function
  * @note   for printf
  * @param  ch
  * @param  f
  * @retval ch
  *********************************************************************************************************************/
int fputc(int ch, FILE *f)
{
    UART_SendData(UART2, (uint8_t)ch);

    while (RESET == UART_GetFlagStatus(UART2, UART_FLAG_TXC))
    {
    }

    return (ch);
}

#else

/***********************************************************************************************************************
  * @brief  redefine fputc function
  * @note   for printf
  * @param  ch
  * @param  f
  * @retval ch
  *********************************************************************************************************************/
int fputc(int ch, FILE *f)
{
    UART_SendData(UART2, (uint8_t)ch);

    while (RESET == UART_GetFlagStatus(UART2, UART_FLAG_TXC))
    {
    }

    return (ch);
}

#endif

/***********************************************************************************************************************
  * @brief  Initialize LED GPIO pin
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void PLATFORM_InitLED(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    PLATFORM_LED_Enable(LED1, ENABLE);
    PLATFORM_LED_Enable(LED2, ENABLE);
}

/***********************************************************************************************************************
  * @brief  LED on or off
  * @note   none
  * @param  LEDn : LED index
  * @arg    LED1, LED2, LED3, LED4
  * @param  State
  * @arg    ENABLE, DISABLE
  * @retval none
  *********************************************************************************************************************/
void PLATFORM_LED_Enable(LEDn_TypeDef LEDn, FunctionalState State)
{
    switch (LEDn)
    {
        case LED1:
            GPIO_WriteBit(GPIOB, GPIO_Pin_15, (ENABLE == State) ? Bit_RESET : Bit_SET);
            break;

        case LED2:
            GPIO_WriteBit(GPIOB, GPIO_Pin_14, (ENABLE == State) ? Bit_RESET : Bit_SET);
            break;

        default:
            break;
    }
}

/***********************************************************************************************************************
  * @brief  LED toggle display
  * @note   none
  * @param  LEDn : LED index
  * @arg    LED1, LED2, LED3, LED4
  * @retval none
  *********************************************************************************************************************/
void PLATFORM_LED_Toggle(LEDn_TypeDef LEDn)
{
    switch (LEDn)
    {
        case LED1:
            GPIO_WriteBit(GPIOB, GPIO_Pin_15, GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15) ? Bit_RESET : Bit_SET);
            break;

        case LED2:
            GPIO_WriteBit(GPIOB, GPIO_Pin_14, GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_14) ? Bit_RESET : Bit_SET);
            break;

        default:
            break;
    }
}

/***********************************************************************************************************************
  * @brief  Initialize Platform
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void PLATFORM_Init(void)
{
    PLATFORM_InitDelay();

    PLATFORM_InitConsole(115200);

    PLATFORM_InitLED();
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

/********************************************** (C) Copyright MindMotion **********************************************/
