#include "boot.h"
#include "string.h"
#include "hal_flash.h"
#include "hid_device.h"

#define Send_Size 64

const u8 USB_Return_Err[64] = {0xD5, 0x04, 0x00, 0x00, 0x59};
const u8 Valid_buf[4] = {0x55, 0xAA, 0xAA, 0x55};
// const u8 unValid_buf[4]={0xff,0xff,0xff,0xff};

#define BURN_VER "0.0.1"

FileData_Block appdata_block[BLOCK_NUM];

u8 code_buff[512];
u8 currentSegment = 0;
u16 code_CurrentLength;
u32 code_TotalLength;

u8 sendbuff[64];

#define code_buff_size sizeof(code_buff)

void Boot_SendGroup(u8 *buffer, u16 bufsize)
{
    tud_hid_report(0, buffer, bufsize);
}

static u16 CheckSum(uint8_t *pdat, uint8_t count) // 和校验;
{
    register int sum = 0;

    while (count > 0)
    {
        sum = sum + *pdat;
        pdat++;
        count--;
    }

    return sum;

    //	while (sum >> 16) //当和的高16位不为0，把高16位作为校验和的一部分求和，
    //		sum = (sum & 0xffff) + (sum >> 16);

    //	return (u16)~sum;
}

static uint8_t Check_data(u8 *txrxCmd)
{
    // uint8_t Flag_check = 0;
    uint16_t tmp_Checksum = 0;
    uint8_t temp1, temp2;

    tmp_Checksum = CheckSum(txrxCmd, txrxCmd[1] - 2);

    temp1 = tmp_Checksum & 0x00FF;
    temp2 = (tmp_Checksum >> 8) & 0x00FF;

    if ((temp2 != txrxCmd[txrxCmd[1] - 2]) || (temp1 != txrxCmd[txrxCmd[1] - 1]))
    {
        return 1;
    }

    return 0;
}

static void FLASH_Write(const u8 *buff, u32 addr, u32 writeNumber)
{
    u32 temp;
    u32 WriteAddress;
    u16 HalfWord;

    WriteAddress = addr;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    for (temp = 0; temp < writeNumber; temp += 2)
    {
        HalfWord = buff[temp];
        HalfWord = HalfWord | (buff[temp + 1] << 8);
        FLASH_ProgramHalfWord(WriteAddress, HalfWord);
        WriteAddress = WriteAddress + 2;
    }

    FLASH_Lock();
}

void FLASH_Read(u8 *buff, u32 addr, u32 readNumber)
{
    u32 temp;
    u32 Address;
    u16 halfword;

    Address = addr;

    for (temp = 0; temp < readNumber; temp += 2)
    {
        halfword = *((vu16 *)Address) & 0xFFFF;
        buff[temp] = halfword & 0xFF;
        buff[temp + 1] = (halfword >> 8) & 0xFF;
        Address += 2;
    }
}

void appVerify(u8 *buff)
{
    u8 j;
    u16 i, idx;

    u16 page_num;
    u16 page_mud;

    u32 addr;
    u32 DatWord = 0;
    u32 check_sum = 0;

    for (j = 0; j < 4; j++)
    {
        if (appdata_block[j].BlockLength == 0)
            break;
        check_sum = 0;
        page_num = appdata_block[j].BlockLength / code_buff_size;
        page_mud = appdata_block[j].BlockLength % code_buff_size;

        // addr = APPFILE_SAVE_ADDR+(appdata_block[j].BlockStartAddr - appdata_block[0].BlockStartAddr);
        addr = appdata_block[j].BlockStartAddr;

        for (i = 0; i < page_num; i++)
        {
            FLASH_Read(code_buff, addr, code_buff_size);
            addr += code_buff_size;
            for (idx = 0; idx < code_buff_size; idx++)
            {
                if (idx & 0x01)
                {
                    DatWord = ((code_buff[idx] << 8) + DatWord);
                    check_sum += DatWord;
                }
                else
                    DatWord = code_buff[idx];
            }
        }

        if (page_mud)
        {
            FLASH_Read(code_buff, addr, page_mud);
            for (idx = 0; idx < page_mud; idx++)
            {
                if (idx & 0x01)
                {
                    DatWord = ((code_buff[idx] << 8) + DatWord);
                    check_sum += DatWord;
                }
                else
                    DatWord = code_buff[idx];
            }
        }
        // printf("checksum = 0x%x",check_sum);
        if (check_sum != appdata_block[j].BlockCheckSum)
        {
            memcpy(buff, USB_Return_Err, 0x40);
            Boot_SendGroup(buff, Send_Size);
            return;
        }
    }

    //    if(memcmp(Valid_buf,code_buff,4) != 0)
    //    {
    //        memcpy(buff, USB_Return_Err, 0x40);
    //        Boot_SendGroup(buff, Send_Size);
    //        return;
    //    }
    memset(buff, 0, 0x40);
    buff[0] = 0xC0 | VERIFY_APP;
    buff[1] = 5;
    buff[2] = 1;
    buff[3] = ((buff[0] + buff[1] + buff[2]) >> 8) & 0xFF;
    buff[4] = (buff[0] + buff[1] + buff[2]) & 0xFF;
    Boot_SendGroup(buff, Send_Size);
}

void segmentInfo(u8 *buff)
{
    u8 i;
    u32 segmentLen;

    code_CurrentLength = 0;
    code_TotalLength = 0;
    currentSegment = 0;

    for (i = 0; i < BLOCK_NUM; i++)
    {
        appdata_block[i].BlockLength = 0;
        appdata_block[i].BlockStartAddr = 0;
        appdata_block[i].BlockCheckSum = 0;
    }

    for (i = 0; i < BLOCK_NUM; i++)
    {
        segmentLen = (buff[i * 12 + 2] << 24) | (buff[i * 12 + 3] << 16) | (buff[i * 12 + 4] << 8) | buff[i * 12 + 5];
        if (segmentLen == 0)
            break;
        appdata_block[i].BlockLength = segmentLen;
        appdata_block[i].BlockStartAddr = (buff[i * 12 + 6] << 24) | (buff[i * 12 + 7] << 16) | (buff[i * 12 + 8] << 8) | buff[i * 12 + 9];
        appdata_block[i].BlockCheckSum = (buff[i * 12 + 10] << 24) | (buff[i * 12 + 11] << 16) | (buff[i * 12 + 12] << 8) | buff[i * 12 + 13];
    }

    buff[0] = SEG_STARTADDR | 0xC0;
    buff[1] = 4;
    buff[2] = ((buff[0] + buff[1]) >> 8) & 0xFF;
    buff[3] = (buff[0] + buff[1]) & 0xFF;
    Boot_SendGroup(buff, Send_Size);
}

static void Data_Recive(u8 *buff, u32 StartAddr, u32 SegmentLength)
{
    if (code_CurrentLength + 60 <= code_buff_size)
    {
        memcpy(&code_buff[code_CurrentLength], &buff[2], buff[1] - 4);
        code_CurrentLength += buff[1] - 4;
    }
    else
    {
        FLASH_Write(code_buff, StartAddr + code_TotalLength, code_CurrentLength);
        code_TotalLength += code_CurrentLength;

        memcpy(&code_buff, &buff[2], buff[1] - 4);
        code_CurrentLength = buff[1] - 4;
    }

    if ((code_TotalLength + code_CurrentLength) == SegmentLength)
    {
        FLASH_Write(code_buff, StartAddr + code_TotalLength, code_CurrentLength);

        code_CurrentLength = 0;
        code_TotalLength = 0;
        if (currentSegment < 3)
            currentSegment++;
    }
}

void appCodeDownload(u8 *buff)
{
    static u8 count = 0;

    //    printf("currentSegment:%d    ncount:%d\r\n",currentSegment+1,count++);
    Data_Recive(buff, appdata_block[currentSegment].BlockStartAddr, appdata_block[currentSegment].BlockLength);

    memset(buff, 0, 0x40);
    buff[0] = WRITE_APP | 0xC0;
    buff[1] = 4;
    buff[2] = ((buff[0] + buff[1]) >> 8) & 0xFF;
    buff[3] = (buff[0] + buff[1]) & 0xFF;

    Boot_SendGroup(buff, Send_Size);
}

void resetMCU(u8 *buff)
{
    u16 sum;

    //    if (Check_data(buff))
    //	{
    //		memset(buff, 0, 0x40);
    //		Boot_SendGroup(buff, Send_Size);
    //	}
    //	else
    {

        buff[0] = buff[0] | 0xc0;
        sum = CheckSum(buff, buff[1] - 2);
        buff[2] = (sum >> 8) & 0xFF;
        buff[3] = sum & 0xFF;
        Boot_SendGroup(buff, Send_Size);
        PLATFORM_DelayMS(5);
        NVIC_SystemReset(); // 系统复位
    }
}

void eraseAppSpace(u8 *buff)
{
    uint32_t Address;
    uint16_t index;
    u16 sum;
    u16 page_erase = (buff[2] << 8) | buff[3];

    //	printf("erase page=%d\r\n",page_erase);
    //	if (Check_data(buff))
    //	{
    //		memset(buff, 0, 0x40);
    //		Boot_SendGroup(buff, Send_Size);
    //	}
    //	else
    {
        Address = ApplicationAddress; // APP flash   0x08002000 -- 0x0801FFFF  0x1E000
        FLASH_Unlock();
        FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        for (index = 0; index < page_erase; index++)
        {
            FLASH_ErasePage(Address);
            Address = Address + 0x400;
        }
        FLASH_Lock();
        buff[0] = buff[0] | 0xC0;
        sum = CheckSum(buff, buff[1] - 2);
        buff[4] = (sum >> 8) & 0xFF;
        buff[5] = sum & 0xff;
        Boot_SendGroup(buff, Send_Size);
    }
}

void getVersion(u8 *buff)
{
    u16 sum = 0;

    memset(buff, 0, 0x40);
    buff[0] = 0xC9;
    buff[1] = 9;
    strncpy((char *)(buff + 2), BURN_VER, 5);
    sum = CheckSum(buff, buff[1] - 2);
    buff[7] = (sum >> 8) & 0xFF;
    buff[8] = sum & 0xFF;

    Boot_SendGroup(buff, Send_Size);
}

void clearAppFlag(u8 *buff)
{
    code_CurrentLength = 0;
    code_TotalLength = 0;
    currentSegment = 0;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    FLASH_ErasePage(BootJumpFlagAddress);

    FLASH_Lock();

    memset(buff, 0, 0x40);
    buff[0] = CLEAR_FLAG | 0xC0;
    buff[1] = 4;
    buff[2] = ((buff[0] + buff[1]) >> 8) & 0xFF;
    buff[3] = (buff[0] + buff[1]) & 0xFF;
    Boot_SendGroup(buff, Send_Size);
}

void writeAppFlag(u8 *buff)
{
    u8 buf[4];

    FLASH_Write(Valid_buf, BootJumpFlagAddress, sizeof(Valid_buf));
    FLASH_Read(buf, BootJumpFlagAddress, 4);
    if (memcmp(buf, Valid_buf, 4) != 0)
    {
        memcpy(buff, USB_Return_Err, 0x40);
        Boot_SendGroup(buff, Send_Size);
        return;
    }
    memset(buff, 0, 0x40);
    buff[0] = WRITE_FLAG | 0xC0;
    buff[1] = 4;
    buff[2] = ((buff[0] + buff[1]) >> 8) & 0xFF;
    buff[3] = (buff[0] + buff[1]) & 0xFF;
    Boot_SendGroup(buff, Send_Size);
}

void mcuInfo(u8 *buff)
{
    u16 sum;

    buff[0] = MCU_INFO | 0xC0;
    buff[1] = 12;
    buff[2] = (ApplicationAddress >> 24) & 0xFF;
    buff[3] = (ApplicationAddress >> 16) & 0xFF;
    buff[4] = (ApplicationAddress >> 8) & 0xFF;
    buff[5] = ApplicationAddress & 0xFF;

    buff[6] = ((APP_SIZE * 1024) >> 24) & 0xFF;
    buff[7] = ((APP_SIZE * 1024) >> 16) & 0xFF;
    buff[8] = ((APP_SIZE * 1024) >> 8) & 0xFF;
    buff[9] = (APP_SIZE * 1024) & 0xFF;

    sum = CheckSum(buff, buff[1] - 2);
    buff[10] = (sum >> 8) & 0xFF;
    buff[11] = sum & 0xFF;

    Boot_SendGroup(buff, Send_Size);
}

void boot_protocol(u8 *buff, u16 len)
{
    u8 cmd;

    memcpy(sendbuff, buff, len);
    buff = sendbuff;
    if (Check_data(buff)) // 校验不通过;
    {
        memcpy(buff, USB_Return_Err, 0x40);
        Boot_SendGroup(buff, Send_Size);
        return;
    }

    cmd = buff[0];
    switch (cmd)
    {
    case GET_VERSION:
        getVersion(buff);
        break;
    case ERASE_APP:
        eraseAppSpace(buff);
        break;
    case SYSTEM_RESET:
        resetMCU(buff);
        break;
    case WRITE_APP:
        appCodeDownload(buff);
        break;
    case VERIFY_APP:
        appVerify(buff);
        break;
    case CLEAR_FLAG:
        clearAppFlag(buff);
        break;
    case WRITE_FLAG:
        writeAppFlag(buff);
        break;
    case SEG_STARTADDR:
        segmentInfo(buff);
        break;
    case MCU_INFO:
        mcuInfo(buff);
        break;
    default:
        break;
    }
}
