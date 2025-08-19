/***********************************************************************************************************************
    @file    tuh_hcd_port.c
    @author  FD Team
    @date    13-March-2024
    @brief   THIS FILE PROVIDES ALL THE SYSTEM FUNCTIONS.
  **********************************************************************************************************************
    @attention

    <h2><center>&copy; Copyright(c) <2024> <MindMotion></center></h2>

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
/* Files include */
#include "tusb_config.h"
#ifdef BOARD_TUH_RHPORT


#include "mm32_device.h"
#include "hal_usbfs.h"
#include "host/hcd.h"
#include "tusb.h"
#include "stdlib.h"

extern uint32_t SystemCoreClock;

/* Private macro ******************************************************************************************************/
/* USB. */
#define BOARD_USB_PORT          USB_FS
#define BOARD_USB_IRQn          USB_FS_IRQn
#define BOARD_USB_IRQHandler    USB_FS_IRQHandler
#define BOARD_USB_SOFTHRESHOLD  255u
#define CLOCK_SYS_FREQ          SystemCoreClock

/* Private variables **************************************************************************************************/
/* the marco of conversion between dev_addr, ep_addr & dev_ep_addr. */
#define TUH_HCD_PORT_GET_DEV_ADDR(dev_ep_addr)         (dev_ep_addr >> 8u) /* dev_ep_addr -> dev_addr. */
#define TUH_HCD_PORT_GET_EP_ADDR(dev_ep_addr)          (dev_ep_addr & 0xFF) /* dev_ep_addr -> ep_addr. */
#define TUH_HCD_PORT_GET_DEV_EP_ADDR(dev_addr,ep_addr) ((dev_addr << 8u) | ep_addr) /* dev_addr + ep_addr -> dev_ep_addr. */
/* the marco of xfer task deque size. */
#define TUH_HCD_PORT_XFER_TASK_DEQUE_SIZE               (CFG_TUH_ENDPOINT_MAX * 2u)
/* the max number of support endpoint. */
#define TUH_HCD_PORT_MAX_EP_NUM                         (CFG_TUH_ENDPOINT_MAX * 2u)
/* the timeout value that xfer data. */
#define TUH_HCD_PORT_XFER_TIMEOUT                       1000000u
/* the buf size of xfer data. */
#define TUH_HCD_PORT_XFER_BUF_SIZE                      64u
/* xfer data need to wait 100ms after bus reset. */
#define TUH_HCD_PORT_FIRST_XFER_DATA_CNT_DOWN           100u;


/* xfer task deque node. */
typedef struct
{
    uint32_t  dev_ep_addr; /* record the dev_addr & ep_addr. */
    uint32_t  max_packet_size; /* record the endpoint max max packet size. */
    uint8_t * buf; /* the xfer data addr. */
    uint32_t  len; /* data size. */
    uint32_t  remaining; /* remaining data size. */
    USBFS_TokenPid_Type token; /* xfer token. */
    uint32_t  data_toggle; /* data0 or data1 that xfer tag. */
} xfer_task_node_t;

/* endpoint status. */
typedef struct
{
    uint32_t dev_ep_addr; /* the index of endpoint, record dev_addr & ep_addr. */
    uint32_t max_packet_size; /* record the max packet size that the endpoint supported. */
    uint32_t data_toggle; /* data toggle, the value is 0 or 1. */
} ep_status_t;

/* xfer task deque support. */
static xfer_task_node_t xfer_task_deque[TUH_HCD_PORT_XFER_TASK_DEQUE_SIZE] = {0u}; /* xfer task deque buf. */
static uint32_t xfer_task_head = 0u; /* record deque head. */
static uint32_t xfer_task_tail = 0u; /* record deque tail. */
static uint32_t xfer_task_cnt  = 0u; /* record xfer task num. */

/* endpoint status table. */
static ep_status_t ep_tbl[TUH_HCD_PORT_MAX_EP_NUM]; /* the table that record the ep_status. */
static uint32_t ep_count = 0u; /* record the how many endpoint status in ep_tbl[]. */

/* usb buf descriptor table. */
__attribute__ ((aligned(512))) static USBFS_BufDespTable_Type usb_bdt = {0u};

/* speed status. */
volatile static tusb_speed_t device_speed = TUSB_SPEED_FULL;

/* xfer_buf. */
__attribute__ ((aligned(4))) static uint8_t usb_xfer_buf[TUH_HCD_PORT_XFER_BUF_SIZE] = {0u};

/* usb first xfer delay. */
volatile uint32_t usb_first_xfer_cnt = 0u;

/* atached. */
volatile bool usb_attached = false;

/*
 * deque func.
 */

/* get deque head node & delete the node in deque. */
bool xfer_task_pop_head(xfer_task_node_t * task)
{
    if (0u == xfer_task_cnt)
    {
        return false; /* no xfer_task. */
    }

    /* pop head: read head, head--, cnt--. */
    memcpy(task, &xfer_task_deque[xfer_task_head], sizeof(xfer_task_node_t)); /* read head. */
    if (xfer_task_head == 0u) /* head--. */
    {
        xfer_task_head = TUH_HCD_PORT_XFER_TASK_DEQUE_SIZE - 1u;
    }
    else
    {
        xfer_task_head--;
    }
    xfer_task_cnt--; /* cnt--. */

    return true;
}

/* put node in deque head. */
bool xfer_task_push_head(xfer_task_node_t * task)
{
    if (TUH_HCD_PORT_XFER_TASK_DEQUE_SIZE == xfer_task_cnt)
    {
        return false; /* queue full. */
    }

    /* push head: head++, write head, cnt++ */

    if (xfer_task_head == TUH_HCD_PORT_XFER_TASK_DEQUE_SIZE - 1u) /* head++. */
    {
        xfer_task_head = 0u;
    }
    else
    {
        xfer_task_head++;
    }
    memcpy(&xfer_task_deque[xfer_task_head], task, sizeof(xfer_task_node_t)); /* write tail. */
    xfer_task_cnt++;/* cnt++. */

    return true;
}

/* put node in deque tail. */
bool xfer_task_push_tail(xfer_task_node_t * task)
{
    if (TUH_HCD_PORT_XFER_TASK_DEQUE_SIZE == xfer_task_cnt)
    {
        return false; /* queue full. */
    }

    /* push tail: write tail, tail--, cnt++ */

    memcpy(&xfer_task_deque[xfer_task_tail], task, sizeof(xfer_task_node_t)); /* write tail. */
    if (xfer_task_tail == 0u) /* head++. */
    {
        xfer_task_tail = TUH_HCD_PORT_XFER_TASK_DEQUE_SIZE - 1u;
    }
    else
    {
        xfer_task_tail--;
    }
    xfer_task_cnt++;/* cnt++. */

    return true;
}

/* reset the deque, delete all node. */
void xfer_task_reset(void)
{
    xfer_task_head = 0u;
    xfer_task_tail = 0u;
    xfer_task_cnt  = 0u;
}

/*
 * endpoint database.
 */

/* get endpoint status. */
ep_status_t * ep_get_status(uint32_t dev_ep_addr)
{
    ep_status_t * ep_status = NULL;
    for(uint32_t i = 0u; i < ep_count; i++)
    {
        if(dev_ep_addr == ep_tbl[i].dev_ep_addr)
        {
            ep_status = &ep_tbl[i];
            break;
        }
    }
    return ep_status;
}

/* add new or modify endpoint status. */
bool ep_set_status(ep_status_t * status)
{
    ep_status_t * ep_status = ep_get_status(status->dev_ep_addr);
    if (NULL == ep_status)
    {
        if (ep_count == TUH_HCD_PORT_MAX_EP_NUM)
        {
            return false;
        }
        ep_status = &ep_tbl[ep_count];
        ep_count++;
    }
    memcpy(ep_status, status, sizeof(ep_status_t));
    return true;
}

/* clear all endpoint status. */
void ep_reset(void)
{
    ep_count = 0u;
}

/*
 * process.
 */

void xfer_start(xfer_task_node_t * task)
{
    USBFS_BufDesp_Type * bd = NULL;

    /* set endpoint status & select bd. */
    switch(task->token)
    {
        case USBFS_TokenPid_IN:
            USBFS_EndPointCmd(BOARD_USB_PORT, 0u, USBFS_EndPointMode_Bulk, ENABLE);
            bd = &usb_bdt.Table[0u][0u][0u];
            break;
        case USBFS_TokenPid_OUT:
            USBFS_EndPointCmd(BOARD_USB_PORT, 0u, USBFS_EndPointMode_Bulk, ENABLE);
            bd = &usb_bdt.Table[0u][1u][0u];
            break;
        case USBFS_TokenPid_SETUP:
            USBFS_EndPointCmd(BOARD_USB_PORT, 0u, USBFS_EndPointMode_Control, ENABLE);
            bd = &usb_bdt.Table[0u][1u][0u];
            break;
        default:
            break;
    }

    /* calc xfer size. */
    uint32_t xfer_size = task->remaining;
    if (xfer_size > task->max_packet_size)
    {
        xfer_size = task->max_packet_size;
    }

    /* calc xfer buf start. */
    uint8_t * xfer_buf = task->buf + (task->len - task->remaining);
    uint32_t  ep_addr = TUH_HCD_PORT_GET_EP_ADDR(task->dev_ep_addr);

    /* xfer data  */
    if (task->token == USBFS_TokenPid_OUT || task->token == USBFS_TokenPid_SETUP)
    {
        memcpy(usb_xfer_buf, xfer_buf, xfer_size);
    }

    /* ready to xfer. */
    
    USBFS_SetDeviceAddr(USB_FS, TUH_HCD_PORT_GET_DEV_ADDR(task->dev_ep_addr));
    USBFS_BufDesp_Reset(bd);
    USBFS_BufDesp_Xfer(bd, task->data_toggle, usb_xfer_buf, xfer_size);
    for(uint32_t i = 0u; i < TUH_HCD_PORT_XFER_TIMEOUT; i++)
    {
        if (USBFS_SetToken(BOARD_USB_PORT, tu_edpt_number(ep_addr), task->token))
        {
            break;
        }
    }
}

void process_softok(void)
{
    xfer_task_node_t task;
    if (xfer_task_pop_head(&task)) /* have xfer task. */
    {
        if (usb_first_xfer_cnt > 0) /* xfer first paxket need to wait a moment. */
        {
            usb_first_xfer_cnt--;
            xfer_task_push_head(&task);
            return;
        }
        xfer_start(&task);
        xfer_task_push_head(&task);
        USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_SOFTOK, DISABLE);
        USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_TOKENDONE, ENABLE);
    }
}

/* process when token xfer done. */
void process_token_done(void)
{
    xfer_task_node_t task;
    if (!xfer_task_pop_head(&task)) /* get xfer task. */
    {
        return; /* error. */
    }

    USBFS_BufDesp_Type * bd = USBFS_GetBufDesp(BOARD_USB_PORT); /* get xfer bd. */

    if (USBFS_BufDesp_GetTokenPid(bd) == USBFS_TokenPid_NAK) /* nak, do next xfer task. */
    {
        xfer_task_push_tail(&task);
        USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_TOKENDONE, DISABLE);
        USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_SOFTOK, ENABLE);
        USBFS_ClearITPendingBit(BOARD_USB_PORT, USBFS_IT_SOFTOK);
        return;
    }

    if (false == usb_attached) /* attached test. */
    {
        if (task.token == USBFS_TokenPid_SETUP) /* test SETUP packet. */
        {
            task.data_toggle = 1u;
            task.dev_ep_addr = 0x80;
            task.token = USBFS_TokenPid_IN;
            xfer_task_push_tail(&task);
        }
        else if (task.token == USBFS_TokenPid_IN) /* test IN packet. */
        {
            task.data_toggle = 1u;
            task.len = 0u;
            task.remaining = 0u;
            task.dev_ep_addr = 0x00;
            task.token = USBFS_TokenPid_OUT;
            xfer_task_push_tail(&task);
        }
        else if (task.token == USBFS_TokenPid_OUT) /* test OUT packet. */
        {
            hcd_event_device_attach(BOARD_TUH_RHPORT, true);
            usb_attached = true;
        }
        USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_TOKENDONE, DISABLE);
        USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_SOFTOK, ENABLE);
        USBFS_ClearITPendingBit(BOARD_USB_PORT, USBFS_IT_SOFTOK);
        return;
    }

    /* move xfer context. */
    if (task.token == USBFS_TokenPid_IN)
    {
        /* calc xfer size. */
        uint32_t xfer_size = task.remaining;
        if (xfer_size > task.max_packet_size)
        {
            xfer_size = task.max_packet_size;
        }

        /* calc xfer buf start. */
        uint8_t * xfer_buf = task.buf + (task.len - task.remaining);
        memcpy(xfer_buf, usb_xfer_buf, xfer_size);
    }

    if (task.remaining > task.max_packet_size) /* have remain, xfer next packet. */
    {
        task.remaining -= task.max_packet_size;
        task.data_toggle ^= 1u;
        xfer_start(&task);
        xfer_task_push_head(&task);
    }
    else /* xfer done. */
    {
        ep_status_t * ep = ep_get_status(task.dev_ep_addr);
        ep->data_toggle = task.data_toggle ^= 1u;
        ep_set_status(ep); /* save ep status. */
        /* send xfer done event. */
        hcd_event_xfer_complete(TUH_HCD_PORT_GET_DEV_ADDR(task.dev_ep_addr),
                    TUH_HCD_PORT_GET_EP_ADDR(task.dev_ep_addr),
                    task.len,
                    XFER_RESULT_SUCCESS, true);
        USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_TOKENDONE, DISABLE);
        USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_SOFTOK, ENABLE);
        USBFS_ClearITPendingBit(BOARD_USB_PORT, USBFS_IT_SOFTOK);
    }
}

void process_attach(void)
{
    USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_ATTACH, DISABLE);
    USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_SOFTOK | USBFS_IT_RESET, ENABLE);
    USBFS_ClearITPendingBit(BOARD_USB_PORT, USBFS_IT_TOKENDONE | USBFS_IT_SOFTOK | USBFS_IT_RESET | USBFS_IT_ATTACH);

    xfer_task_reset();
    ep_reset();

    /* get speed. */
    if (USBFS_BusSignalStatus_J == USBFS_GetBusSignalStatus(BOARD_USB_PORT) ) /* full speed. */
    {
        USBFS_LowSpeedCmd(BOARD_USB_PORT, DISABLE);
        device_speed = TUSB_SPEED_FULL;
    }
    else /* low speed. */
    {
        USBFS_LowSpeedCmd(BOARD_USB_PORT, ENABLE);
        device_speed = TUSB_SPEED_LOW;
    }

    /* attach tast. */
    usb_first_xfer_cnt = TUH_HCD_PORT_FIRST_XFER_DATA_CNT_DOWN;
    usb_attached = false;
    /* bus reset signal. */
    USBFS_Cmd(BOARD_USB_PORT, DISABLE);
    USBFS_BusResetCmd(BOARD_USB_PORT, ENABLE);
    for (uint32_t i = CLOCK_SYS_FREQ / 100u; i > 0; i--)
    {
    }
    USBFS_BusResetCmd(BOARD_USB_PORT, DISABLE);
    USBFS_Cmd(BOARD_USB_PORT, ENABLE);

    /* build SETUP packet. */
    memset(usb_xfer_buf, 0u, 8u);
    usb_xfer_buf[0] = 0x80;
    usb_xfer_buf[1] = 0x06;
    usb_xfer_buf[3] = 0x01;
    usb_xfer_buf[6] = 0x08;

    /* build xfer task. */
    xfer_task_node_t task;
    task.buf = usb_xfer_buf;
    task.data_toggle = 0;
    task.dev_ep_addr = 0x00;
    task.len = 8u;
    task.max_packet_size = 8u;
    task.remaining = 8u;
    task.token = USBFS_TokenPid_SETUP;
    xfer_task_push_tail(&task);

    /* prepare to send packet. */
    USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_SOFTOK, ENABLE);
    USBFS_ClearITPendingBit(BOARD_USB_PORT, USBFS_IT_SOFTOK);
}

void process_detach(void)
{
    usb_attached = false;
    xfer_task_reset();
    ep_reset();
    device_speed = TUSB_SPEED_FULL;
    USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_TOKENDONE | USBFS_IT_SOFTOK | USBFS_IT_RESET, DISABLE);
    USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_ATTACH, ENABLE); /* wait device attached. */
    USBFS_ClearITPendingBit(BOARD_USB_PORT, USBFS_IT_TOKENDONE | USBFS_IT_SOFTOK | USBFS_IT_RESET | USBFS_IT_ATTACH);

    USBFS_Cmd(BOARD_USB_PORT, DISABLE); /* stop send SOF token. */
    USBFS_SetDeviceAddr(BOARD_USB_PORT, 0x00); /* set usb addr is 0x00, to xfer data when device attached. */
    USBFS_EndPointCmd(BOARD_USB_PORT, 0, USBFS_EndPointMode_NULL, DISABLE); /* disable ep0. */
    USBFS_LowSpeedCmd(BOARD_USB_PORT, DISABLE); /* disable low speed mode. */
    memset(&usb_bdt, 0, sizeof(usb_bdt));
    hcd_event_device_remove(BOARD_TUH_RHPORT, true);
}

/*
 * Controller API.
 */

/* Initialize controller to host mode. */
bool hcd_init(uint8_t rhport)
{
    (void)rhport;

    /* init usb host module. */
    USBFS_Host_Init_Type usb_init;
    usb_init.BufDespTable_Addr = (uint32_t)(&usb_bdt);
    usb_init.SofThreshold      = BOARD_USB_SOFTHRESHOLD;
    usb_init.NakRetry          = false;
    USBFS_InitHost(BOARD_USB_PORT, &usb_init);

    USBFS_OddEvenResetCmd(BOARD_USB_PORT, ENABLE); /* only use even buf desp, this example will not usb odd buf desp xfer data. */
    USBFS_SetDeviceAddr(BOARD_USB_PORT, 0x00); /* set usb addr is 0x00, to xfer data when device attached. */

    /* enable interrupt, but not use NVIC_EnableIRQ(). */
    NVIC_ClearPendingIRQ(BOARD_USB_IRQn);
    USBFS_ITConfig(BOARD_USB_PORT, USBFS_IT_ATTACH | USBFS_IT_ERROR, ENABLE);
    USBFS_ErrITConfig(BOARD_USB_PORT, (USBFS_IT_ERR_PID \
                                | USBFS_IT_ERR_EOF   \
                                | USBFS_IT_ERR_CRC16 \
                                | USBFS_IT_ERR_DFN8  \
                                | USBFS_IT_ERR_BTO   \
                                | USBFS_IT_ERR_DMA   \
                                | USBFS_IT_ERR_BTS), ENABLE);
    return true;
}

/* Enable USB interrupt. */
void hcd_int_enable (uint8_t rhport)
{
    (void)rhport;

    NVIC_EnableIRQ(BOARD_USB_IRQn);
}

/* Disable USB interrupt. */
void hcd_int_disable(uint8_t rhport)
{
    (void)rhport;

    NVIC_DisableIRQ(BOARD_USB_IRQn);
}

/* Get frame number (1ms). */
uint32_t hcd_frame_number(uint8_t rhport)
{
    (void)rhport;

    return USBFS_GetFrameNumber(BOARD_USB_PORT);
}

/*
 * Port API.
 */

/* Get the current connect status of roothub port. */
bool hcd_port_connect_status(uint8_t rhport)
{
    (void)rhport;

    if (0 != USBFS_GetITStatus(BOARD_USB_PORT, USBFS_IT_ATTACH))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* Reset USB bus on the port. */
void hcd_port_reset(uint8_t rhport)
{
    (void)rhport;

    ep_reset();
    xfer_task_reset();
    usb_first_xfer_cnt = TUH_HCD_PORT_FIRST_XFER_DATA_CNT_DOWN;
    USBFS_Cmd(BOARD_USB_PORT, DISABLE);
    USBFS_BusResetCmd(BOARD_USB_PORT, ENABLE);

    for (uint32_t i = CLOCK_SYS_FREQ / 100u; i > 0; i--)
    {
    }

    USBFS_BusResetCmd(BOARD_USB_PORT, DISABLE);
    USBFS_Cmd(BOARD_USB_PORT, ENABLE);
}

/* TODO implement later. */
void hcd_port_reset_end(uint8_t rhport)
{
    (void)rhport;
}

/* Get port link speed. */
tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
    (void)rhport;

    return device_speed;
}

/* HCD closes all opened endpoints belong to this device. */
void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
    xfer_task_reset();
    ep_reset();
}

/*
 * Endpoints API.
 */

/* Open an endpoint. */
bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
    (void)rhport;

    NVIC_DisableIRQ(BOARD_USB_IRQn);
    ep_status_t ep     = {0u};
    ep.max_packet_size = ep_desc->wMaxPacketSize;
    ep.dev_ep_addr     = TUH_HCD_PORT_GET_DEV_EP_ADDR(dev_addr, ep_desc->bEndpointAddress);

    if (tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN)
    {
        ep.data_toggle = 1u;
    }
    else
    {
        ep.data_toggle = 0u;
    }

    if (0u == ep_desc->bEndpointAddress)
    {
        ep.dev_ep_addr = TUH_HCD_PORT_GET_DEV_EP_ADDR(dev_addr, 0x00);
        ep_set_status(&ep);
        ep.dev_ep_addr = TUH_HCD_PORT_GET_DEV_EP_ADDR(dev_addr, 0x80);
        ep_set_status(&ep);
        return true;
    }
    else if (ep_set_status(&ep))
    {
        return true;
    }
    else
    {
        return false;
    }
    NVIC_EnableIRQ(BOARD_USB_IRQn);
}

/* Submit a transfer, when complete hcd_event_xfer_complete() must be invoked. */
bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t * buffer, uint16_t buflen)
{
    (void)rhport;

    NVIC_DisableIRQ(BOARD_USB_IRQn);
    xfer_task_node_t task;
    task.dev_ep_addr = TUH_HCD_PORT_GET_DEV_EP_ADDR(dev_addr, ep_addr);
    task.buf      = (uint8_t *)buffer;
    if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN)
    {
        task.token = USBFS_TokenPid_IN;
    }
    else
    {
        task.token = USBFS_TokenPid_OUT;
    }
    ep_status_t * ep = ep_get_status(task.dev_ep_addr);
    task.max_packet_size = ep->max_packet_size;
    task.data_toggle     = ep->data_toggle;
    task.len       = buflen;
    task.remaining = buflen;

    bool ret = false;
    if (xfer_task_push_tail(&task) )
    {
        ret = true;
    }
    NVIC_EnableIRQ(BOARD_USB_IRQn);
    return ret;
}

/* Submit a special transfer to send 8-byte Setup Packet, when complete hcd_event_xfer_complete() must be invoked. */
bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
    (void)rhport;

    NVIC_DisableIRQ(BOARD_USB_IRQn);
    xfer_task_node_t task;
    task.dev_ep_addr = TUH_HCD_PORT_GET_DEV_EP_ADDR(dev_addr, 0u);
    task.max_packet_size = 8u;
    task.buf      = (uint8_t *)setup_packet;
    task.token    = USBFS_TokenPid_SETUP;
    task.data_toggle   = 0u;
    task.len      = 8u;
    task.remaining = 8u;

    bool ret = false;
    if (xfer_task_push_tail(&task) )
    {
        ret = true;
    }
    NVIC_EnableIRQ(BOARD_USB_IRQn);
    return ret;
}

/*
 * USB interrupt handler.
 */
void BOARD_USB_IRQHandler(void)
{
    uint32_t flag = USBFS_GetEnabledITStatus(BOARD_USB_PORT);
//    flag &= ;

    /* device attached. */
    if (0u != (flag & USBFS_IT_ATTACH))
    {
        process_attach();
    }

    /* sof token, prepare to xfer packet. */
    if (0u != (flag & USBFS_IT_SOFTOK))
    {
        process_softok();
    }

    /* xfer a token done. */
    if (0u != (flag & USBFS_IT_TOKENDONE))
    {
        process_token_done();
    }

    /* device detached. */
    if (0u != (flag & USBFS_IT_RESET))
    {
        tuh_task(); /* clear all event. */
        process_detach(); /* do detache process. */
    }

    if (0u != (flag & USBFS_IT_ERROR))
    {
        tuh_task(); /* clear all event. */
        process_detach(); /* do detache process. */
        USBFS_ClearErrITPendingBit(BOARD_USB_PORT, (USBFS_IT_ERR_PID \
                                | USBFS_IT_ERR_EOF   \
                                | USBFS_IT_ERR_CRC16 \
                                | USBFS_IT_ERR_DFN8  \
                                | USBFS_IT_ERR_BTO   \
                                | USBFS_IT_ERR_DMA   \
                                | USBFS_IT_ERR_BTS)); /* clear all err interrut. */
    }

    USBFS_ClearITPendingBit(BOARD_USB_PORT, flag);
}

bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
    (void) rhport;
    (void) dev_addr;
    (void) ep_addr;
    // TODO not implemented yet
    return false;
}

/***********************************************************************************************************************
  * @brief  USB Host Clock Init.
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void USB_HostClockInit(void)
{
    /* Enable HSE */
    MODIFY_REG(RCC->CR, RCC_CR_HSEON_Msk, 1 << RCC_CR_HSEON_Pos);
    
    // Config PLL2 clock param.
    RCC_PLL2Config(1, 0, 7, 1); // PLL2 = HSE(12) * (7+1) / (1+1) = 48MHz
    MODIFY_REG(RCC->PLL2CFGR, RCC_PLL2CFGR_PLL2_ICTRL_Msk, 3 << RCC_PLL2CFGR_PLL2_ICTRL_Pos);

    // Enable PLL2 clock.
    RCC_PLL2Cmd(ENABLE);
    while((RCC->CR & RCC_CR_PLL2RDY_Msk) == 0)
    {
    }

    // Select PLL2 as USB clock source.
    MODIFY_REG(RCC->CFGR, RCC_CFGR_USBCLKSEL_Msk, 1 << RCC_CFGR_USBCLKSEL_Pos);

    // Enable USB Clock.
    MODIFY_REG(RCC->CFGR, RCC_CFGR_USBPRE_Msk, 0 << RCC_CFGR_USBPRE_Pos);

    // Select HSE as system clock.
    RCC_SYSCLKConfig(2);
    
    // Enable USB_FS Periph Clock.
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USB_FS, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    // sys time init;
#if CFG_TUSB_OS == OPT_OS_NONE
    RCC_ClocksTypeDef sys_clk;
    RCC_GetClocksFreq(&sys_clk);
    SysTick_Config(sys_clk.SYSCLK_Frequency / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
#endif

    // PB0-KEY1
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    gpio_init.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &gpio_init);

    // PB1-KEY2
    gpio_init.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &gpio_init);
}


// Turn LED on or off
void board_led_write(bool state)
{
  if (state)
  {
      GPIO_SetBits(GPIOB, GPIO_Pin_14|GPIO_Pin_15);
  }
  else
  {
      GPIO_ResetBits(GPIOB, GPIO_Pin_14|GPIO_Pin_15);
  };
}

// Get the current state of button
// a '1' means active (pressed), a '0' means inactive.
uint32_t board_button_read(void)
{
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == RESET)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/***********************************************************************************************************************
  * @brief  Blinking Task.
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
#if CFG_TUSB_OS == OPT_OS_NONE
volatile static uint32_t systime = 0;
// Get current milliseconds, must be implemented when no RTOS is used
uint32_t board_millis(void)
{
  return systime;
}

void SysTick_Handler(void)
{
  systime++;
}
#endif

// Get characters from UART. Return number of read bytes
int board_uart_read(uint8_t *buf, int len)
{
  if (UART2->CSR & UART_FLAG_RXAVL)
  {
    buf[0] = UART2->RDR;
    return 1;
  }
  else
  {
    return 0;
  }
}

// Send characters to UART. Return number of sent bytes
int board_uart_write(void const *buf, int len)
{
  for (int i = 0; i < len; i++)
  {
    putchar(((char*)(buf))[i]);
  }
  return len;
}

// stdio getchar() is blocking, this is non-blocking version
int board_getchar(void)
{
  if (UART2->CSR & UART_FLAG_RXAVL)
  {
    return UART2->RDR;
  }
  else
  {
    return 0;
  }
}

#endif
