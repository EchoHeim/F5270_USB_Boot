/***********************************************************************************************************************
    @file    tinyusb_device_dual_cdc.c
    @author  FD Team
    @date    15-March-2024
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
#define _TINYUSB_DEVICE_DUAL_CDC_C_

/* Files include */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "platform.h"

#include "board_api.h"
#include "tusb.h"
#include "tinyusb_host_cdc_msc_hid.h"
/**
  * @addtogroup MM32F5270_TinyUSB
  * @{
  */

/**
  * @addtogroup TinyUSB_Host
  * @{
  */

/**
  * @addtogroup TinyUSB_Host_CDC_MSC_HID
  * @{
  */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/
//------------- prototypes -------------//
void led_blinking_task(void);

extern void cdc_app_task(void);
extern void hid_app_task(void);

/* Private functions **************************************************************************************************/
/***********************************************************************************************************************
  * @brief TinyUSB Host Configure
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void TinyUSB_Host_Configure(void)
{
    USB_HostClockInit();

    // init host stack on configured roothub port
    tuh_init(BOARD_TUH_RHPORT);
}

/***********************************************************************************************************************
  * @brief  TinyUSB Device Dual CDC Sample.
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void TinyUSB_Host_CDC_MSC_HID_Sample(void)
{
    printf("\r\nTest %s\r\n", __FUNCTION__);

    TinyUSB_Host_Configure();

    if (board_init_after_tusb) {
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

/***********************************************************************************************************************
  * @brief  TinyUSB Mount Callbacks.
  * @note   none
  * @param  dev_addr: device address.
  * @retval none
  *********************************************************************************************************************/
void tuh_mount_cb(uint8_t dev_addr)
{
  // application set-up
  printf("A device with address %d is mounted\r\n", dev_addr);
}

/***********************************************************************************************************************
  * @brief  TinyUSB Umount Callbacks.
  * @note   none
  * @param  dev_addr: device address.
  * @retval none
  *********************************************************************************************************************/
void tuh_umount_cb(uint8_t dev_addr)
{
  // application tear-down
  printf("A device with address %d is unmounted \r\n", dev_addr);
}

/***********************************************************************************************************************
  * @brief  TinyUSB Blinking Task.
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void led_blinking_task(void)
{
  const uint32_t interval_ms = 1000;
  static uint32_t start_ms = 0;

  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
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
