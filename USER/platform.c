/***********************************************************************************************************************
    @file    platform.c
    @author  FAE Team
    @date    19-Jun-2023
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
 * @addtogroup UID
 * @{
 */

/**
 * @addtogroup UID_ReadUID
 * @{
 */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/

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

#if defined(__ICCARM__)

#if (__VER__ >= 9030001)

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

#elif defined(__GNUC__)

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

/********************************************** (C) Copyright MindMotion **********************************************/
