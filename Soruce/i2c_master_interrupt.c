/***********************************************************************************************************************
    @file    i2c_master_eeprom_interrupt.c
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
#define _I2C_MASTER_EEPROM_INTERRUPT_C_

/* Files include */
#include <stdio.h>
#include "platform.h"
#include "i2c_master_interrupt.h"

/**
  * @addtogroup MM32F5330_LibSamples
  * @{
  */

/**
  * @addtogroup I2C
  * @{
  */

/**
  * @addtogroup I2C_Master_EEPROM_Interrupt
  * @{
  */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/
#define EEPROM_I2C_ADDRESS      0xA0
#define EEPROM_PAGE_SIZE        0x08

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void I2C_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef  I2C_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    I2C_DeInit(I2C2);

    I2C_StructInit(&I2C_InitStruct);
    I2C_InitStruct.I2C_Mode       = I2C_MODE_MASTER;
    I2C_InitStruct.I2C_OwnAddress = I2C_OWN_ADDRESS;
    I2C_InitStruct.I2C_ClockSpeed = 100000;
    I2C_Init(I2C2, &I2C_InitStruct);

    I2C_TargetAddressConfig(I2C2, EEPROM_I2C_ADDRESS);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_4);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = I2C2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    I2C_Cmd(I2C2, ENABLE);
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void I2C_TxData_Interrupt(uint8_t *Buffer, uint8_t Length)
{
    uint8_t i = 0;

    for (i = 0; i < Length; i++)
    {
        I2C_TxStruct.Buffer[i] = Buffer[i];
    }

    I2C_TxStruct.Length = Length;
    I2C_TxStruct.CurrentCount = 0;
    I2C_TxStruct.CompleteFlag = 0;

    I2C_ClearFlag(I2C2, I2C_FLAG_TX_EMPTY);
    I2C_ITConfig(I2C2, I2C_IT_TX_EMPTY, ENABLE);

    while (0 == I2C_TxStruct.CompleteFlag)
    {
    }
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void I2C_RxData_Interrupt(uint8_t *Buffer, uint16_t Length)
{
    uint8_t i = 0;

    I2C_RxStruct.Length = Length;
    I2C_RxStruct.CurrentCount = 0;
    I2C_RxStruct.CompleteFlag = 0;

    I2C_ClearFlag(I2C2, I2C_FLAG_RX_FULL);
    I2C_ITConfig(I2C2, I2C_IT_RX_FULL, ENABLE);

    I2C_ReadCmd(I2C2);

    while (0 == I2C_RxStruct.CompleteFlag)
    {
    }

    for (i = 0; i < Length; i++)
    {
        Buffer[i] = I2C_RxStruct.Buffer[i];
    }
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void EEPROM_WritePage(uint8_t Address, uint8_t *Buffer, uint8_t Length)
{
    I2C_TxData_Interrupt((uint8_t *)&Address, 0x01);

    I2C_TxData_Interrupt((uint8_t *)Buffer, Length);

    while (RESET == I2C_GetFlagStatus(I2C2, I2C_STATUS_FLAG_TFE))
    {
    }

    I2C_GenerateSTOP(I2C2, ENABLE);

    while (RESET == I2C_GetFlagStatus(I2C2, I2C_FLAG_STOP_DET))
    {
    }
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void EEPROM_ReadData(uint8_t Address, uint8_t *Buffer, uint8_t Length)
{
    I2C_TxData_Interrupt((uint8_t *)&Address, 0x01);

    I2C_RxData_Interrupt((uint8_t *)Buffer, Length);

    I2C_GenerateSTOP(I2C2, ENABLE);

    while (RESET == I2C_GetFlagStatus(I2C2, I2C_FLAG_STOP_DET))
    {
    }
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void EEPROM_WriteData(uint8_t Address, uint8_t *Buffer, uint8_t Length)
{
    uint8_t Start = 0;
    uint8_t StartCount = 0, PageNumber = 0, FinalCount = 0;

    if ((Address % EEPROM_PAGE_SIZE) == 0)
    {
        StartCount = 0;
        PageNumber = Length / EEPROM_PAGE_SIZE;
        FinalCount = Length % EEPROM_PAGE_SIZE;
    }
    else
    {
        Start = Address % EEPROM_PAGE_SIZE;

        if (((Start + Length) / EEPROM_PAGE_SIZE) == 0)
        {
            StartCount = Length;
            PageNumber = 0;
            FinalCount = 0;
        }
        else
        {
            StartCount = EEPROM_PAGE_SIZE - Start;
            PageNumber = (Length - StartCount) / EEPROM_PAGE_SIZE;
            FinalCount = (Length - StartCount) % EEPROM_PAGE_SIZE;
        }
    }

    if (StartCount)
    {
        EEPROM_WritePage(Address, Buffer, StartCount);

        Address += StartCount;
        Buffer  += StartCount;

        PLATFORM_DelayMS(50);
    }

    while (PageNumber--)
    {
        EEPROM_WritePage(Address, Buffer, EEPROM_PAGE_SIZE);

        Address += EEPROM_PAGE_SIZE;
        Buffer  += EEPROM_PAGE_SIZE;

        PLATFORM_DelayMS(50);
    }

    if (FinalCount)
    {
        EEPROM_WritePage(Address, Buffer, FinalCount);
    }
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void I2C_Master_EEPROM_Interrupt_Sample(void)
{
    uint8_t i = 0;
    uint8_t ReadBuffer[20], WriteBuffer[20];

    printf("\r\nTest %s", __FUNCTION__);

    I2C_Configure();

    for (i = 0; i < 20; i++)
    {
        ReadBuffer[i]  = 0;
        WriteBuffer[i] = i + 0x20;
    }

    printf("\r\n\r\nEEPROM Write : ");

    EEPROM_WriteData(0, WriteBuffer, 20);

    printf("OK");

    printf("\r\n\r\nEEPROM Read  : \r\n");

    EEPROM_ReadData(0, ReadBuffer, 20);

    for (i = 0; i < 20; i++)
    {
        printf("0x%02x ", ReadBuffer[i]);

        if (0 == ((i + 1) % 10))
        {
            printf("\r\n");
        }
    }

    while (1)
    {
        PLATFORM_LED_Toggle(LED1);
        PLATFORM_DelayMS(100);
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

