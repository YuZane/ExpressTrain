/***********************************************************************************************************************
    @file    i2c_master_polling.c
    @author  FAE Team
    @date    08-May-2023
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
#define _I2C_MASTER_POLLING_C_

/* Files include */
#include <stdio.h>
#include "platform.h"
#include "i2c_master_polling.h"

#define PAJ7620U2_I2C_ADDRESS		0x73<<1 /*paj7620的i2c地址，左移一位后为左对齐七位地址*/
#if 0
#define PAJ_BANK_SELECT			0xEF		/*寄存器页选择寄存器，Bank0=0x00,Bank1=0x01*/
#define PAJ_INT_FLAG1		    0x43		/*检测结果数据低八位数据所在地址*/
#define PAJ_INT_FLAG2			0x44		/*检测结果数据高八位数据所在地址*/
//Gesture detection interrupt flag
#define PAJ_UP				    0x01 
#define PAJ_DOWN			    0x02
#define PAJ_LEFT			    0x04 
#define PAJ_RIGHT			    0x08
#define PAJ_FORWARD			    0x10 
#define PAJ_BACKWARD		    0x20
#define PAJ_CLOCKWISE			0x40
#define PAJ_COUNT_CLOCKWISE		0x80
#define PAJ_WAVE				0x100

uint8_t Init_Register_Array[][2] = {
	{0xEF, 0x00},
  {0x32, 0x29},
  {0x33, 0x01},
  {0x34, 0x00},
  {0x35, 0x01},
  {0x36, 0x00},
  {0x37, 0x07},
  {0x38, 0x17},
  {0x39, 0x06},
  {0x3A, 0x12},
  {0x3F, 0x00},
  {0x40, 0x02},
  {0x41, 0xFF},
  {0x42, 0x01},
  {0x46, 0x2D},
  {0x47, 0x0F},
  {0x48, 0x3C},
  {0x49, 0x00},
  {0x4A, 0x1E},
  {0x4B, 0x00},
  {0x4C, 0x20},
  {0x4D, 0x00},
  {0x4E, 0x1A},
  {0x4F, 0x14},
  {0x50, 0x00},
  {0x51, 0x10},
  {0x52, 0x00},
  {0x5C, 0x02},
  {0x5D, 0x00},
  {0x5E, 0x10},
  {0x5F, 0x3F},
  {0x60, 0x27},
  {0x61, 0x28},
  {0x62, 0x00},
  {0x63, 0x03},
  {0x64, 0xF7},
  {0x65, 0x03},
  {0x66, 0xD9},
  {0x67, 0x03},
  {0x68, 0x01},
  {0x69, 0xC8},
  {0x6A, 0x40},
  {0x6D, 0x04},
  {0x6E, 0x00},
  {0x6F, 0x00},
  {0x70, 0x80},
  {0x71, 0x00},
  {0x72, 0x00},
  {0x73, 0x00},
  {0x74, 0xF0},
  {0x75, 0x00},
  {0x80, 0x42},
  {0x81, 0x44},
  {0x82, 0x04},
  {0x83, 0x20},
  {0x84, 0x20},
  {0x85, 0x00},
  {0x86, 0x10},
  {0x87, 0x00},
  {0x88, 0x05},
  {0x89, 0x18},
  {0x8A, 0x10},
  {0x8B, 0x01},
  {0x8C, 0x37},
  {0x8D, 0x00},
  {0x8E, 0xF0},
  {0x8F, 0x81},
  {0x90, 0x06},
  {0x91, 0x06},
  {0x92, 0x1E},
  {0x93, 0x0D},
  {0x94, 0x0A},
  {0x95, 0x0A},
  {0x96, 0x0C},
  {0x97, 0x05},
  {0x98, 0x0A},
  {0x99, 0x41},
  {0x9A, 0x14},
  {0x9B, 0x0A},
  {0x9C, 0x3F},
  {0x9D, 0x33},
  {0x9E, 0xAE},
  {0x9F, 0xF9},
  {0xA0, 0x48},
  {0xA1, 0x13},
  {0xA2, 0x10},
  {0xA3, 0x08},
  {0xA4, 0x30},
  {0xA5, 0x19},
  {0xA6, 0x10},
  {0xA7, 0x08},
  {0xA8, 0x24},
  {0xA9, 0x04},
  {0xAA, 0x1E},
  {0xAB, 0x1E},
  {0xCC, 0x19},
  {0xCD, 0x0B},
  {0xCE, 0x13},
  {0xCF, 0x64},
  {0xD0, 0x21},
  {0xD1, 0x0F},
  {0xD2, 0x88},
  {0xE0, 0x01},
  {0xE1, 0x04},
  {0xE2, 0x41},
  {0xE3, 0xD6},
  {0xE4, 0x00},
  {0xE5, 0x0C},
  {0xE6, 0x0A},
  {0xE7, 0x00},
  {0xE8, 0x00},
  {0xE9, 0x00},
  {0xEE, 0x07},
  {0xEF, 0x01},
  {0x00, 0x1E},
  {0x01, 0x1E},
  {0x02, 0x0F},
  {0x03, 0x10},
  {0x04, 0x02},
  {0x05, 0x00},
  {0x06, 0xB0},
  {0x07, 0x04},
  {0x08, 0x0D},
  {0x09, 0x0E},
  {0x0A, 0x9C},
  {0x0B, 0x04},
  {0x0C, 0x05},
  {0x0D, 0x0F},
  {0x0E, 0x02},
  {0x0F, 0x12},
  {0x10, 0x02},
  {0x11, 0x02},
  {0x12, 0x00},
  {0x13, 0x01},
  {0x14, 0x05},
  {0x15, 0x07},
  {0x16, 0x05},
  {0x17, 0x07},
  {0x18, 0x01},
  {0x19, 0x04},
  {0x1A, 0x05},
  {0x1B, 0x0C},
  {0x1C, 0x2A},
  {0x1D, 0x01},
  {0x1E, 0x00},
  {0x21, 0x00},
  {0x22, 0x00},
  {0x23, 0x00},
  {0x25, 0x01},
  {0x26, 0x00},
  {0x27, 0x39},
  {0x28, 0x7F},
  {0x29, 0x08},
  {0x30, 0x03},
  {0x31, 0x00},
  {0x32, 0x1A},
  {0x33, 0x1A},
  {0x34, 0x07},
  {0x35, 0x07},
  {0x36, 0x01},
  {0x37, 0xFF},
  {0x38, 0x36},
  {0x39, 0x07},
  {0x3A, 0x00},
  {0x3E, 0xFF},
  {0x3F, 0x00},
  {0x40, 0x77},
  {0x41, 0x40},
  {0x42, 0x00},
  {0x43, 0x30},
  {0x44, 0xA0},
  {0x45, 0x5C},
  {0x46, 0x00},
  {0x47, 0x00},
  {0x48, 0x58},
  {0x4A, 0x1E},
  {0x4B, 0x1E},
  {0x4C, 0x00},
  {0x4D, 0x00},
  {0x4E, 0xA0},
  {0x4F, 0x80},
  {0x50, 0x00},
  {0x51, 0x00},
  {0x52, 0x00},
  {0x53, 0x00},
  {0x54, 0x00},
  {0x57, 0x80},
  {0x59, 0x10},
  {0x5A, 0x08},
  {0x5B, 0x94},
  {0x5C, 0xE8},
  {0x5D, 0x08},
  {0x5E, 0x3D},
  {0x5F, 0x99},
  {0x60, 0x45},
  {0x61, 0x40},
  {0x63, 0x2D},
  {0x64, 0x02},
  {0x65, 0x96},
  {0x66, 0x00},
  {0x67, 0x97},
  {0x68, 0x01},
  {0x69, 0xCD},
  {0x6A, 0x01},
  {0x6B, 0xB0},
  {0x6C, 0x04},
  {0x6D, 0x2C},
  {0x6E, 0x01},
  {0x6F, 0x32},
  {0x71, 0x00},
  {0x72, 0x01},
  {0x73, 0x35},
  {0x74, 0x00},
  {0x75, 0x33},
  {0x76, 0x31},
  {0x77, 0x01},
  {0x7C, 0x84},
  {0x7D, 0x03},
  {0x7E, 0x01},
};

//初始化，返回0则初始化失败
uint8_t PAJ7620U2_init(void)
{
  uint8_t i,State,n;
  State = 0;
  while(0 > I2C_Write(0xef, &State, 1)){PLATFORM_DelayMS(5);printf("1");}

  for (i=0;i< 219 ;i++)
  {
    while(0 > I2C_Write(Init_Register_Array[i][0], &Init_Register_Array[i][1], 1)){PLATFORM_DelayMS(5);printf("2");}
    PLATFORM_DelayMS(5);
  }

  while(0 > I2C_Write(0xef, &State, 1)){PLATFORM_DelayMS(5);printf("3");}
  PLATFORM_DelayMS(5);

  while (0 > I2C_Read(0x32, &n, 1)){PLATFORM_DelayMS(5);printf("4");}
  if(n != 0x29)
  {
  return 0;
  }

  return 1;
}
 
//检测手势并输出
void gesture(void)
{	
  uint8_t Data[2]={0,0};
  uint16_t Gesture_Data;
  I2C_Read(PAJ_INT_FLAG1, &Data[0], 1);
  PLATFORM_DelayMS(5);

  I2C_Read(PAJ_INT_FLAG2, &Data[1], 1);
  PLATFORM_DelayMS(5);

  Gesture_Data= Data[1] <<8 | Data[0];
  printf("\n %x \r\n",Gesture_Data);
  if(Gesture_Data !=0)
  {	
  switch (Gesture_Data)
      {
        case PAJ_UP:			  			  printf("Up\r\n");				break;
        case PAJ_DOWN:							printf("Down\r\n");				break;
        case PAJ_LEFT:
          printf("Left\r\n");
          PLATFORM_LED_Enable(LED1, 0);
          break;
        case PAJ_RIGHT:
        	printf("Right\r\n");
          PLATFORM_LED_Enable(LED1, 1);
          break;
        case PAJ_FORWARD:						printf("Forward\r\n");			break;
        case PAJ_BACKWARD:					printf("Backward\r\n"); 		break;
        case PAJ_CLOCKWISE:					printf("Clockwise\r\n"); 		break;
        case PAJ_COUNT_CLOCKWISE:		printf("AntiClockwise\r\n"); 	break;
        case PAJ_WAVE:							printf("Wave\r\n"); 			break;
        default: break;
      }
      Gesture_Data=0;
    }
  }
#endif
/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void I2C2_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef  I2C_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    I2C_DeInit(I2C2);

    I2C_StructInit(&I2C_InitStruct);
    I2C_InitStruct.I2C_Mode       = I2C_MODE_MASTER;
    I2C_InitStruct.I2C_OwnAddress = I2C_OWN_ADDRESS;
    I2C_InitStruct.I2C_ClockSpeed = 100000;
    I2C_Init(I2C2, &I2C_InitStruct);

    I2C_TargetAddressConfig(I2C2, PAJ7620U2_I2C_ADDRESS);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_4);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    I2C_Cmd(I2C2, ENABLE);
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void I2C_TxData_Polling(uint8_t *Buffer, uint8_t Length)
{
    uint8_t i = 0;

    for (i = 0; i < Length; i++)
    {
        I2C_SendData(I2C2, Buffer[i]);

        while (RESET == I2C_GetFlagStatus(I2C2, I2C_STATUS_FLAG_TFE))
        {
        }
    }
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void I2C_RxData_Polling(uint8_t *Buffer, uint16_t Length)
{
    uint8_t i = 0;

    for (i = 0; i < Length; i++)
    {
        I2C_ReadCmd(I2C2);

        while (RESET == I2C_GetFlagStatus(I2C2, I2C_STATUS_FLAG_RFNE))
        {
        }

        Buffer[i] = I2C_ReceiveData(I2C2);
    }
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
int I2C_Write(uint8_t Address, uint8_t *Buffer, uint8_t Length)
{
    I2C_TxData_Polling((uint8_t *)&Address, 0x01);

    I2C_TxData_Polling((uint8_t *)Buffer, Length);

    while (RESET == I2C_GetFlagStatus(I2C2, I2C_STATUS_FLAG_TFE))
    {
    }

    I2C_GenerateSTOP(I2C2, ENABLE);

    while (RESET == I2C_GetFlagStatus(I2C2, I2C_STATUS_FLAG_TFE))
    {
    }
    return Length;
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
int I2C_Read(uint8_t Address, uint8_t *Buffer, uint8_t Length)
{
    I2C_TxData_Polling((uint8_t *)&Address, 0x01);

    I2C_RxData_Polling((uint8_t *)Buffer, Length);

    I2C_GenerateSTOP(I2C2, ENABLE);

    while (RESET == I2C_GetFlagStatus(I2C2, I2C_STATUS_FLAG_TFE))
    {
    }
    return Length;
}

extern u8 Gesture_test(u8 *ges);
/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void I2C_Master_Polling_Sample(void)
{
    uint8_t i = 0;
    uint8_t ReadBuffer[20], WriteBuffer[20];
    u8 ges;
    printf("\r\nTest %s", __FUNCTION__);

    I2C2_Configure();
    // PAJ7620U2_init();
    // Gesture_test(&ges);
    while (1)
    {
        // gesture();
        Gesture_test(&ges);
        // PLATFORM_LED_Toggle(LED2);
        // PLATFORM_DelayMS(300);
    }
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

