/***********************************************************************************************************************
    @file    main.c
    @author  FD Team
    @date    12-March-2024
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
#define _MAIN_C_

/* Files include */
#include "hal_conf.h"
#include "tusb.h"
#include "board_api.h"

void led_blinking_task(void);
extern void cdc_app_task(void);
extern void hid_app_task(void);
void USB_HostClockInit(void);

int fputc(int ch, FILE *f)
{
    UART_SendData(UART2, (uint8_t)ch);

    while (RESET == UART_GetFlagStatus(UART2, UART_FLAG_TXC))
        ;

    return (ch);
}

void PLATFORM_InitConsole(uint32_t Baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    UART_InitTypeDef UART_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);

    UART_StructInit(&UART_InitStruct);
    UART_InitStruct.BaudRate = Baudrate;
    UART_InitStruct.WordLength = UART_WordLength_8b;
    UART_InitStruct.StopBits = UART_StopBits_1;
    UART_InitStruct.Parity = UART_Parity_No;
    UART_InitStruct.HWFlowControl = UART_HWFlowControl_None;
    UART_InitStruct.Mode = UART_Mode_Tx;
    UART_Init(UART2, &UART_InitStruct);

    UART_Cmd(UART2, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void PLATFORM_InitLED(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
}

/***********************************************************************************************************************
 * @brief  This function is main entrance
 * @note   main
 * @param  none
 * @retval none
 *********************************************************************************************************************/
int main(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    while (SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000))
        ;

    NVIC_SetPriority(SysTick_IRQn, 0x0);

    PLATFORM_InitLED();
    PLATFORM_InitConsole(115200);

    //--------------------------------------
    USB_HostClockInit();
    // init host stack on configured roothub port
    tuh_init(BOARD_TUH_RHPORT);

    if (board_init_after_tusb)
    {
        board_init_after_tusb();
    }

    while (1)
    {
        // tinyusb host task
        tuh_task();

        led_blinking_task();
        cdc_app_task();
        hid_app_task();
    }
}

void led_blinking_task(void)
{
    const uint32_t interval_ms = 1000;
    static uint32_t start_ms = 0;

    static bool led_state = false;

    // Blink every interval ms
    if (board_millis() - start_ms < interval_ms)
        return; // not enough time
    start_ms += interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}

/********************************************** (C) Copyright MindMotion **********************************************/
