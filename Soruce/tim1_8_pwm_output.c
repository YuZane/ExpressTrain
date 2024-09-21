/***********************************************************************************************************************
    @file    tim1_8_pwm_output.c
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
#define _TIM1_8_PWM_OUTPUT_C_

/* Files include */
#include <stdio.h>
#include "platform.h"
#include "tim1_8_pwm_output.h"

/**
  * @addtogroup MM32F5330_LibSamples
  * @{
  */

/**
  * @addtogroup TIM1_8
  * @{
  */

/**
  * @addtogroup TIM1_8_PWM_Output
  * @{
  */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/

#define LED_HIGH (140)
#define LED_LOW (60)

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void TIM1_DMA_Interrupt(uint32_t *Buffer, uint32_t Length)
{
    DMA_InitTypeDef  DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel2);

    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(TIM1->DMAR);
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)Buffer;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize         = Length;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Word;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStruct.DMA_Auto_Reload        = DMA_Auto_Reload_Disable;
    DMA_Init(DMA1_Channel2, &DMA_InitStruct);

    DMA_ClearFlag(DMA1_FLAG_TC2);
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = DMA1_CH2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // USART_TX_DMA_InterruptFlag = 0;
    DMA_Cmd(DMA1_Channel2, ENABLE);
  
    TIM_DMACmd(TIM1, TIM_DMA_CC1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void TIM1_8_Configure(void)
{
    GPIO_InitTypeDef        GPIO_InitStruct;
    TIM_OCInitTypeDef       TIM_OCInitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

    uint32_t TimerPeriod1 = 0, Channel1Pulse1 = 0, Channel2Pulse1 = 0, Channel3Pulse1 = 0;
    uint32_t TimerPeriod8 = 0, Channel1Pulse8 = 0, Channel2Pulse8 = 0, Channel3Pulse8 = 0;

    /* Compute the value to be set in ARR regiter to generate signal frequency at 100 Khz */
    TimerPeriod1 = TIM_GetTIMxClock(TIM1) / 900000;
    TimerPeriod8 = TIM_GetTIMxClock(TIM8) / 900000;

    /* Compute CCR1 value to generate a duty cycle at 70% for channel 1 */
    Channel1Pulse1 = (uint32_t)0 * TimerPeriod1 / 1000;
    Channel1Pulse8 = (uint32_t)0 * TimerPeriod8 / 1000;

    /* Compute CCR2 value to generate a duty cycle at 50% for channel 2 */
    Channel2Pulse1 = (uint32_t)300 * TimerPeriod1 / 1000;
    Channel2Pulse8 = (uint32_t)500 * TimerPeriod8 / 1000;

    /* Compute CCR3 value to generate a duty cycle at 25% for channel 3 */
    Channel3Pulse1 = (uint32_t)700 * TimerPeriod1 / 1000;
    Channel3Pulse8 = (uint32_t)250 * TimerPeriod8 / 1000;

   printf("\r\nT1:%d, %d, %d, %d", TimerPeriod1, Channel1Pulse1, Channel2Pulse1, Channel3Pulse1);
//    printf("\r\nT8:%d, %d, %d, %d", TimerPeriod8, Channel1Pulse8, Channel2Pulse8, Channel3Pulse8);

    /* TIM1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    TIM_TimeBaseStruct.TIM_Prescaler         = 0;
    TIM_TimeBaseStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period            = TimerPeriod1;
    TIM_TimeBaseStruct.TIM_ClockDivision     = TIM_CKD_Div1;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse        = 0;
    TIM_OCInitStruct.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OCIdleState  = TIM_OCIdleState_Set;

    TIM_OCInitStruct.TIM_Pulse = Channel1Pulse1;;
    TIM_OC1Init(TIM1, &TIM_OCInitStruct);

    TIM_OCInitStruct.TIM_Pulse = Channel2Pulse1;
    TIM_OC2Init(TIM1, &TIM_OCInitStruct);

    TIM_OCInitStruct.TIM_Pulse = Channel3Pulse1;
    TIM_OC3Init(TIM1, &TIM_OCInitStruct);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_1);    /* TIM1_CH1 */

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_High;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_DMAConfig(TIM1, TIM_DMABase_CCR1, 1);
    TIM_DMACmd(TIM1, TIM_DMA_CC1, ENABLE);
    TIM_SelectCCDMA(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

#define LED_NUM 60
typedef struct {
    u32 G[8];
    u32 R[8];
    u32 B[8];
}one_color_t;

one_color_t ColorBuf[3][LED_NUM + 4];
//show GRB
void setAllColor_dma(one_color_t *color, uint32_t c) {
    uint8_t r, g, b;
    r = (uint8_t) (c >> 16);
    g = (uint8_t) (c >> 8);
    b = (uint8_t) c;
    uint8_t i, j;
    for (i = 1; i < LED_NUM + 1; i++) {
        for (j = 0; j < 8; j++) {
            color[i].R[j] = (r & (1 << j)) ? LED_HIGH : LED_LOW;
            color[i].G[j] = (g & (1 << j)) ? LED_HIGH : LED_LOW;
            color[i].B[j] = (b & (1 << j)) ? LED_HIGH : LED_LOW;
        }
    }
}


/***********************************************************************************************************************
  * @brief
  * @note   none
  * @param  none
  * @retval none
  *********************************************************************************************************************/
void TIM1_8_PWM_Output_Sample(void)
{
  int i = 0;
  static int j = 0;
    // for (i = 0; i < 89; i++) {
    //   // ggbuf[i] = i;
    //   if (i % 2) {
    //     ggbuf[i] = 0x37;
    //   } else {
    //     ggbuf[i] = 0x82;
    //   }
    // }
    printf("yz debug %s-%d\n", __FUNCTION__, __LINE__);
    setAllColor_dma(ColorBuf[1], 0x00ff00);
    printf("yz debug %s-%d\n", __FUNCTION__, __LINE__);
    // printf("\r\nTest %s, ggbuf1 %d, ggbuf2 %d", __FUNCTION__, ggbuf[1], ggbuf[2]); 

    TIM1_8_Configure();
    // printf("yz debug %s-%d\n", __FUNCTION__, __LINE__);
    // ggbuf[59] = 0;
    // ggbuf[58] = 0;
    // ggbuf[0] = 0;
    // ggbuf[1] = 0;
    // printf("yz debug %s-%d\n", __FUNCTION__, __LINE__);

    printf("\r\nRemove C18 & C19 & X2(32.768KHz).");

    printf("\r\nUse Logic Analyzer to monitor PA8  / PA9  / PA10 output.");
    printf("\r\nUse Logic Analyzer to monitor PC13 / PC14 / PC15 output.");

    while (1)
    {
				setAllColor_dma(ColorBuf[1], 0x000000);
				TIM1_DMA_Interrupt((u32 *)ColorBuf[1], (LED_NUM + 2) * 24);
				PLATFORM_DelayMS(1000);
			
			  setAllColor_dma(ColorBuf[1], 0xff0000);
				TIM1_DMA_Interrupt((u32 *)ColorBuf[1], (LED_NUM + 2) * 24);
        PLATFORM_DelayMS(1000);
			
				setAllColor_dma(ColorBuf[1], 0x00ff00);
				TIM1_DMA_Interrupt((u32 *)ColorBuf[1], (LED_NUM + 2) * 24);
				PLATFORM_DelayMS(1000);
			
				setAllColor_dma(ColorBuf[1], 0x0000ff);
				TIM1_DMA_Interrupt((u32 *)ColorBuf[1], (LED_NUM + 2) * 24);
				PLATFORM_DelayMS(1000);
			
				PLATFORM_LED_Toggle(LED1);
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

