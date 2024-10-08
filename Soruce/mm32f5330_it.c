/***********************************************************************************************************************
    @file    mm32f5330_it.c
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
#define _MM32F5330_IT_C_

/* Files include */
#include <stdio.h>
#include "platform.h"
#include "i2c_master_polling.h"
#include "uart_interrupt.h"
#include "gpio_key_input.h"
#include "paj7620u2.h"
#include "mm32f5330_it.h"


extern u32 DmaIdle;
/**
  * @addtogroup MM32F5330_LibSamples
  * @{
  */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/

/***********************************************************************************************************************
  * @brief  This function handles Non maskable interrupt
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void NMI_Handler(void)
{
    while (1)
    {
    }
}

/***********************************************************************************************************************
  * @brief  This function handles Hard fault interrupt
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void HardFault_Handler(void)
{
    while (1)
    {
    }
}

/***********************************************************************************************************************
  * @brief  This function handles Memory management fault
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void MemManage_Handler(void)
{
    while (1)
    {
    }
}

/***********************************************************************************************************************
  * @brief  This function handles Pre-fetch fault, memory access fault
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void BusFault_Handler(void)
{
    while (1)
    {
    }
}

/***********************************************************************************************************************
  * @brief  This function handles Undefined instruction or illegal state
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

/***********************************************************************************************************************
  * @brief  This function handles System service call via SWI instruction
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void SVCall_Handler(void)
{
}

/***********************************************************************************************************************
  * @brief  This function handles Debug monitor
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void DebugMon_Handler(void)
{
}

/***********************************************************************************************************************
  * @brief  This function handles Pendable request for system service
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void PendSV_Handler(void)
{
}

/***********************************************************************************************************************
  * @brief  This function handles System tick timer
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void SysTick_Handler(void)
{
    if (0 != PLATFORM_DelayTick)
    {
        PLATFORM_DelayTick--;
    }
}
#if 0
/***********************************************************************************************************************
  * @brief  This function handles I2C2 Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void I2C2_IRQHandler(void)
{
    uint8_t Data = 0;

    if (SET == I2C_GetITStatus(I2C2, I2C_IT_RX_FULL))
    {
        I2C_ClearITPendingBit(I2C2, I2C_IT_RX_FULL);

        Data = I2C_ReceiveData(I2C2);

        if (0 == I2C_RxStruct.CompleteFlag)
        {
            I2C_RxStruct.Buffer[I2C_RxStruct.CurrentCount++] = Data;

            if (I2C_RxStruct.CurrentCount == I2C_RxStruct.Length)
            {
                I2C_RxStruct.CompleteFlag = 1;

                I2C_ITConfig(I2C2, I2C_IT_RX_FULL, DISABLE);
            }
            else
            {
                I2C_ReadCmd(I2C2);
            }
        }
    }

    if (SET == I2C_GetITStatus(I2C2, I2C_IT_TX_EMPTY))
    {
        I2C_ClearITPendingBit(I2C2, I2C_IT_TX_EMPTY);

        if (0 == I2C_TxStruct.CompleteFlag)
        {
            I2C_SendData(I2C2, I2C_TxStruct.Buffer[I2C_TxStruct.CurrentCount++]);

            if (I2C_TxStruct.CurrentCount == I2C_TxStruct.Length)
            {
                I2C_TxStruct.CompleteFlag = 1;

                I2C_ITConfig(I2C2, I2C_IT_TX_EMPTY, DISABLE);
            }
        }
    }
}
#endif

/***********************************************************************************************************************
  * @brief  This function handles UART2 Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
#if 1
char VoiceCmdstr[] = "cmd_id:2";
char VoiceCmd;

void UART2_IRQHandler(void)
{
    uint8_t RxData = 0;
    static int index = 0;
    if (SET == UART_GetITStatus(UART2, UART_IT_RX))
    {
        RxData = UART_ReceiveData(UART2);
        // cmd_id:2
        // printf("%c", RxData);
        if (index >= 7) {
            VoiceCmd = RxData;
            index = 0;
        }
        if (RxData != VoiceCmdstr[index]) {
            index = 0;
        } else {
            index++;
        }
        UART_ClearITPendingBit(UART2, 0xff);
    }
}
#else
void UART2_IRQHandler(void)
{
    uint8_t RxData = 0;

    if (SET == UART_GetITStatus(UART2, UART_IT_RX))
    {
        RxData = UART_ReceiveData(UART2);

        UART_ClearITPendingBit(UART2, UART_IT_RX);

        if (0 == UART_RxStruct.CompleteFlag)
        {
            UART_RxStruct.Buffer[UART_RxStruct.CurrentCount++] = RxData;

            if (UART_RxStruct.CurrentCount == UART_RxStruct.Length)
            {
                UART_RxStruct.CompleteFlag = 1;

                UART_ITConfig(UART2, UART_IT_RX, DISABLE);
            }
        }
    }

    if (SET == UART_GetITStatus(UART2, UART_IT_TX))
    {
        UART_ClearITPendingBit(UART2, UART_IT_TX);

        if (0 == UART_TxStruct.CompleteFlag)
        {
            UART_SendData(UART2, UART_TxStruct.Buffer[UART_TxStruct.CurrentCount++]);

            if (UART_TxStruct.CurrentCount == UART_TxStruct.Length)
            {
                UART_TxStruct.CompleteFlag = 1;

                UART_ITConfig(UART2, UART_IT_TX, DISABLE);
            }
        }
    }
}
#endif

/***********************************************************************************************************************
  * @brief  This function handles EXTI1 Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void EXTI1_IRQHandler(void)
{
    /* K3 */
    if (SET == EXTI_GetITStatus(EXTI_Line1))
    {
        PLATFORM_LED_Toggle(LED3);
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

/***********************************************************************************************************************
  * @brief  This function handles EXTI2 Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void EXTI2_IRQHandler(void)
{
    /* K4 */
    if (SET == EXTI_GetITStatus(EXTI_Line2))
    {
        PLATFORM_LED_Toggle(LED4);
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

/***********************************************************************************************************************
  * @brief  This function handles EXTI4 Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
extern KeyState_t KeyState;
void EXTI4_IRQHandler(void)
{
    int i, count = 0;
    /* K1 */
    if (SET == EXTI_GetITStatus(EXTI_Line4))
    {
        PLATFORM_LED_Toggle(LED1);
        #if 1
          for ( i = 0; i < 10000; i++) {
            if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == Bit_SET)
              count++;
          }
          if (count > 9000) {
            KeyState.update = 1;
            printf("yz debug %s-%d key update\n", __FUNCTION__, __LINE__);
          }
          printf("yz debug %s-%d key count %d\n", __FUNCTION__, __LINE__, count);
        #else
          #if 0
          // if (KeyState.intr) {
            KeyState.intr = 0;
            // KeyState.update = 1;
            printf("yz debug %s-%d key update\n", __FUNCTION__, __LINE__);
          // }
          #else
          printf("yz debug %s-%d key update\n", __FUNCTION__, __LINE__);
          #endif
        #endif
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

/***********************************************************************************************************************
  * @brief  This function handles EXTI9_5 Handler
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void EXTI9_5_IRQHandler(void)
{
    /* K2 */
    if (SET == EXTI_GetITStatus(EXTI_Line5))
    {
        PLATFORM_LED_Toggle(LED2);
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}
extern gesture_info_t gesture;
void EXTI0_IRQHandler(void)
{
    uint8_t ges[2] = {0};
    /* K3 */
    if (SET == EXTI_GetITStatus(EXTI_Line0))
    {
        PLATFORM_LED_Toggle(LED4);
        EXTI_ClearITPendingBit(EXTI_Line0);
        I2C_Read(0x43, ges, 2);
        gesture.data =(u16)ges[1] << 8 | ges[0];
        gesture.update = 1;
    }
}

/***********************************************************************************************************************
  * @brief  This function handles TIM2
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void TIM2_IRQHandler(void)
{
    if (RESET != TIM_GetITStatus(TIM2, TIM_IT_Update))
    {
        // PLATFORM_LED_Toggle(LED2);

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

/***********************************************************************************************************************
  * @brief  This function handles TIM5
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void TIM5_IRQHandler(void)
{
  static int a = 0;
    a = !a;
    if (RESET != TIM_GetITStatus(TIM5, TIM_IT_Update))
    {
        // PLATFORM_LED_Toggle(LED3);
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, a);

        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
}

void DMA1_CH2_IRQHandler(void)
{
    if (SET == DMA_GetITStatus(DMA1_IT_TC2))
    {
        DMA_ClearFlag(DMA1_FLAG_GL2);
        DMA_ClearFlag(DMA1_FLAG_TC2);
        DMA_ClearFlag(DMA1_FLAG_HT2);
        DMA_ClearFlag(DMA1_FLAG_TE2);
        DMA_Cmd(DMA1_Channel2, DISABLE);
        DMA_ClearITPendingBit(DMA2_IT_TC2);
        TIM_DMACmd(TIM1, TIM_DMA_CC1, DISABLE);
        TIM_Cmd(TIM1, DISABLE);
        DmaIdle = 1;
        // printf("yz dma\r\n");

        // USART_TX_DMA_InterruptFlag = 1;
        
        
    }
}

/********************************************** (C) Copyright MindMotion **********************************************/

