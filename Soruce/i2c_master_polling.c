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
    while (1)
    {
        Gesture_test(&ges);
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

