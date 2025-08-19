#ifndef _BOOT_H_
#define _BOOT_H_

#include "platform.h"

#define BootJumpFlagAddress (0x8000000 + 19 * 1024ul)
#define ApplicationAddress (0x8000000 + 20 * 1024ul)

#define APP_SIZE (256 - 20) // 240KB flash for app

#define BLOCK_NUM 4 // ���֧��4��hex��¼

#define GET_VERSION 0x20
#define ERASE_APP 0x21
#define SYSTEM_RESET 0x22
#define WRITE_APP 0x23
#define VERIFY_APP 0x24
#define CLEAR_FLAG 0x25
#define WRITE_FLAG 0x26
#define SEG_STARTADDR 0x27 // ��֧��hex�ļ�������أ������ָ��
#define MCU_INFO 0x28      // ��λ����ȡMCU�ָ�APP�ռ��С��APP��ŵ���ʼλ��

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define vu16 volatile uint16_t

typedef struct
{
    u32 BlockStartAddr;
    u32 BlockLength;
    u32 BlockCheckSum;
} FileData_Block;

extern const u8 Valid_buf[4];

void boot_protocol(u8 *buff, u16 len);
void FLASH_Read(u8 *buff, u32 addr, u32 readNumber);

void USB_DeviceClockInit(void);

#endif
