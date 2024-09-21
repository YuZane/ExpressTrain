#define _TIM1_8_PWM_OUTPUT_C_

/* Files include */
#include <stdio.h>
#include "platform.h"
#include "rgbandhsv.h"
#include "tim1_8_pwm_output.h"


#define LED_NUM 60
#define LED_HIGH (140)
#define LED_LOW (60)

typedef struct {
    u32 G[8];
    u32 R[8];
    u32 B[8];
}dma_color_t;

typedef struct {
	u8 R;
	u8 G;
	u8 B;
}color_rgb_t;

dma_color_t ColorBuf[3][LED_NUM + 4];


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

void setOneColor_dma(dma_color_t *color, uint32_t rgb) {
    uint8_t r, g, b;
    r = (uint8_t) (rgb >> 16);
    g = (uint8_t) (rgb >> 8);
    b = (uint8_t) rgb;
    uint8_t j;
		for (j = 0; j < 8; j++) {
				color->R[j] = (r & (1 << j)) ? LED_HIGH : LED_LOW;
				color->G[j] = (g & (1 << j)) ? LED_HIGH : LED_LOW;
				color->B[j] = (b & (1 << j)) ? LED_HIGH : LED_LOW;
		}
}

void setAllColor_dma(dma_color_t *color, uint32_t rgb) {
    uint8_t i;
    for (i = 1; i < LED_NUM + 1; i++) {
        setOneColor_dma(&color[i], rgb);
    }
}

void LED_CONFIG_ALL(u32 rgb)
{
		setAllColor_dma(ColorBuf[2], rgb);
		TIM1_DMA_Interrupt((u32 *)ColorBuf[2], (LED_NUM + 2) * 24);
}

void Marquee(u32 rgb)
{
		int i = 1;
		for (i = 1; i < LED_NUM + 1; i++) {
				setOneColor_dma(&ColorBuf[2][i], rgb);
				TIM1_DMA_Interrupt((u32 *)ColorBuf[2], (LED_NUM + 2) * 24);
				PLATFORM_DelayMS(40);
				setOneColor_dma(&ColorBuf[2][i], 0x000000);
		}
}

#if 0
/* 呼吸灯曲线表 */
const uint16_t index_wave[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 
                               4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 
                               13, 13, 14, 14, 15, 15, 16, 16, 17, 18, 18, 19, 20, 20, 21, 22, 23, 24, 25, 25, 26, 27, 28, 30, 31, 32, 33, 
                               34, 36, 37, 38, 40, 41, 43, 45, 46, 48, 50, 52, 54, 56, 58, 60, 62, 65, 67, 70, 72, 75, 78, 81, 84, 87, 90, 
                               94, 97, 101, 105, 109, 113, 117, 122, 126, 131, 136, 141, 146, 152, 158, 164, 170, 176, 183, 190, 197, 205, 
                               213, 221, 229, 238, 247, 256, 256, 247, 238, 229, 221, 213, 205, 197, 190, 183, 176, 170, 164, 158, 152, 146, 
                               141, 136, 131, 126, 122, 117, 113, 109, 105, 101, 97, 94, 90, 87, 84, 81, 78, 75, 72, 70, 67, 65, 62, 60, 58, 
                               56, 54, 52, 50, 48, 46, 45, 43, 41, 40, 38, 37, 36, 34, 33, 32, 31, 30, 28, 27, 26, 25, 25, 24, 23, 22, 21, 20, 
                               20, 19, 18, 18, 17, 16, 16, 15, 15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 7, 6, 
                               6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 
                               2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
                               
#else
const uint16_t index_wave[] = {
	85,  // 600 ms
		30,  // 300 ms
	        5,   // 100 ms
	    70,  // 500 ms
	    15,  // 200 ms
    50,  // 400 ms
    //95,  // 700 ms
    100, // 800 ms
	100,
    50,
    15,
    70,
    5,
    30,
    85,
    85,    // 1600 ms
		85,    // 1600 ms
};
#endif
/* light 0-255 */
void LED_LIGHT(u32 rgb, u8 light) {
    u32 rgbout;
    float h, s, v;
    uint8_t r, g, b;
    r = (uint8_t) (rgb >> 16);
    g = (uint8_t) (rgb >> 8);
    b = (uint8_t) rgb;

    rgb2hsv(r, g, b, &h, &s, &v);
		printf("yz debug %s-%d h %f s %f v %f\n", __FUNCTION__, __LINE__, h,s,v);
		v = (float)light / 100;
    hsv2rgb(h ,s ,v, &r, &g, &b);
		printf("yz debug %s-%d r %d g %d b %d v %f\n", __FUNCTION__, __LINE__, r,g,b,v);

    rgbout = r << 16 | g << 8 | g;

		printf("yz debug %s-%d rgbout %d\n", __FUNCTION__, __LINE__, rgbout);

    LED_CONFIG_ALL(rgbout);
		PLATFORM_DelayMS(100);
}

void Breath(u32 rgb)
{
    int i;
    printf("yz debug %s-%d sizeof(index_wave) / sizeof(uint16_t) %d\n", __FUNCTION__, __LINE__, sizeof(index_wave) / sizeof(uint16_t));
    for (i = 0; i < sizeof(index_wave) / sizeof(uint16_t); i++)
    {
        LED_LIGHT(rgb, index_wave[i]);
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

    printf("yz debug %s-%d\n", __FUNCTION__, __LINE__);
		setAllColor_dma(ColorBuf[0], 0x000000);
    setAllColor_dma(ColorBuf[1], 0x00ff00);
    printf("yz debug %s-%d\n", __FUNCTION__, __LINE__);
    float h,s,v;
    rgb2hsv(0xff, 0, 0, &h, &s, &v);
    printf("yz debug %s-%d 0xff h %f s %f v %f\n", __FUNCTION__, __LINE__, h, s, v);
    rgb2hsv(0x0f, 0, 0, &h, &s, &v);
    printf("yz debug %s-%d 0x0f h %f s %f v %f\n", __FUNCTION__, __LINE__, h, s, v);
    rgb2hsv(0xf0, 0, 0, &h, &s, &v);
    printf("yz debug %s-%d 0xf0 h %f s %f v %f\n", __FUNCTION__, __LINE__, h, s, v);


    TIM1_8_Configure();
		LED_CONFIG_ALL(0x000000);
    while (1)
    {
				//setOneColor_dma(&ColorBuf[2][1], 0xff0000);
				// Marquee(0xff0000);
        Breath(0xff0000);
				#if 0
				LED_CONFIG_ALL(0x000000);
				PLATFORM_DelayMS(1000);
				LED_CONFIG_ALL(0xf00000);
				PLATFORM_DelayMS(1000);
				LED_CONFIG_ALL(0x00f000);
				PLATFORM_DelayMS(1000);
				LED_CONFIG_ALL(0x0000f0);
				PLATFORM_DelayMS(1000);
				#endif
				// PLATFORM_LED_Toggle(LED1);
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

