/***********************************************************************************************************************
    @file    main.c
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
#define _MAIN_C_

/* Files include */
#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "i2c_master_polling.h"
#include "uart_interrupt.h"
#include "gpio_key_input.h"
#include "exti_interrupt.h"
#include "ws2812b_tim1_1_pwm_dma.h"
#include "paj7620u2.h"
#include "main.h"

/**
  * @addtogroup MM32F5330_LibSamples
  * @{
  */

/* Private typedef ****************************************************************************************************/

/* Private define *****************************************************************************************************/

/* Private macro ******************************************************************************************************/

/* Private variables **************************************************************************************************/

/* Private functions **************************************************************************************************/

enum {
    MODE_IDLE = 0,
    MODE_KEYUPDATE,
    MODE_LEFT,
    MODE_RIGHT,
    MODE_FORWARD,
    MODE_BACKWARD,
    MODE_ON,
    MODE_CLOSE,
    MODE_BREATH,
    MODE_MARQUEE,
};
KeyState_t KeyState;
gesture_info_t gesture;
u8 LedCurOn;
u8 mode;
u8 LastMode;
extern u32 DmaIdle;
extern char VoiceCmd;
EXTERN volatile UART_RxTx_TypeDef UART_RxStruct;
static dma_color_t ColorBuf[LED_NUM + 4];
extern uint16_t IndexHeart[215];
extern uint16_t IndexForward[113];
extern uint16_t IndexBackward[116];

#define VOICE_IDLE 0x0
#define VOICE_OPEN_LED 0x32
#define VOICE_CLOSE_LED 0x33
#define VOICE_BREATH 0x34
#define VOICE_MARQUEE 0x35

/***********************************************************************************************************************
  * @brief  This function is main entrance
  * @note   main
  * @param  none
  * @retval none
  *********************************************************************************************************************/
int main(void)
{
    static uint8_t LedIndex = 0;
    uint8_t KeyCount = 0;
    u8 gsdate[2];
    int r;
    u32 rgb;

    PLATFORM_Init();
    GPIO_Configure();
    TIM1_8_Configure();
    I2C2_Configure();
    UART_Configure(921600);

    paj7620u2_init();
    paj7620u2_init_gesture();
    EXTI_Configure();
    LED_CONFIG_ALL(0x000000);
    LedCurOn = 0;
    LastMode = mode = MODE_IDLE;

    while (1) {
        if (gesture.update) {
            gesture.update = 0;
            switch (gesture.data)
            {
                case PAJ_UP:	
                    // printf("Up\r\n");
                    break;
                case PAJ_DOWN:
                    // printf("Down\r\n");
                    break;
                case PAJ_LEFT:
                    mode = MODE_LEFT;
                    printf("Left\r\n");
                    break;
                case PAJ_RIGHT:
                    mode = MODE_RIGHT;
                    printf("Right\r\n");
                    break;
                case PAJ_FORWARD:
                    mode = MODE_FORWARD;
                    printf("Forward\r\n");
                    break;
                case PAJ_BACKWARD:
                    mode = MODE_BACKWARD;
                    printf("Backward\r\n");
                    break;
                case PAJ_CLOCKWISE:
                    // printf("Clockwise\r\n");
                    break;
                case PAJ_COUNT_CLOCKWISE:
                    // printf("AntiClockwise\r\n");
                    break;
                case PAJ_WAVE:
                    // printf("Wave\r\n");
                    break;
                default:
                    break;
            }
        }

        //voice
        switch (VoiceCmd)
        {
            case VOICE_OPEN_LED:
                mode = MODE_ON;
                break;
            case VOICE_CLOSE_LED:
                mode = MODE_CLOSE;
                break;
            case VOICE_MARQUEE:
                mode = MODE_MARQUEE;
                break;
            case VOICE_BREATH:
                mode = MODE_BREATH;
                break;
            default:
                break;
        }
        VoiceCmd = VOICE_IDLE;

        //key
        KeyState.intr = 1;
        if (KeyState.update) {
            KeyState.update = 0;
            mode = MODE_KEYUPDATE;
        }

        if (LastMode != mode || mode == MODE_IDLE) {
            LedIndex = 0;
        }

        if (DmaIdle) {
            switch (mode)
            {
                case MODE_KEYUPDATE:
                    if (/*KeyState.status*/ !LedCurOn) {
                        printf("yz debug %s-%d en\n", __FUNCTION__, __LINE__);
                        PLATFORM_LED_Enable(LED1, ENABLE);
                        LED_CONFIG_ALL(0xf00000);
                        LedCurOn = 1;
                    } else {
                        printf("yz debug %s-%d dis\n", __FUNCTION__, __LINE__);
                        PLATFORM_LED_Enable(LED1, DISABLE);
                        LED_CONFIG_ALL(0x000000);
                        LedCurOn = 0;
                    }
                    mode = MODE_IDLE;
                    break;
                case MODE_LEFT:
                    // Marquee_R2L(0xf00000);
                    if (LedIndex >= LED_NUM) {
                        LedIndex = 0;
                        mode = MODE_IDLE;
                        break;
                    }
                    if (LedIndex == 0) {
                        printf("MODE_LEFT\r\n");
                        setAllColor_dma(ColorBuf, 0x000000);
                    }
                    setOneColor_dma(&ColorBuf[LED_NUM - 1 - LedIndex], 0x00f000);
                    TIM1_DMA_Interrupt((u32 *)ColorBuf, (LED_NUM + 2) * 24);
                    PLATFORM_DelayMS(20);
                    setOneColor_dma(&ColorBuf[LED_NUM - 1 - LedIndex], 0x000000);
                    LedIndex++;
                    LedCurOn = 1;
                    break;
                case MODE_RIGHT:
                    // Marquee_L2R(0xf00000);
                    if (LedIndex >= LED_NUM) {
                        LedIndex = 0;
                        mode = MODE_IDLE;
                        break;
                    }
                    if (LedIndex == 0) {
                        printf("MODE_RIGHT\r\n");
                        setAllColor_dma(ColorBuf, 0x000000);
                    }
                    setOneColor_dma(&ColorBuf[LedIndex], 0x0000f0);
                    TIM1_DMA_Interrupt((u32 *)ColorBuf, (LED_NUM + 2) * 24);
                    PLATFORM_DelayMS(20);
                    setOneColor_dma(&ColorBuf[LedIndex], 0x000000);
                    LedIndex++;
                    LedCurOn = 1;
                    break;
                case MODE_FORWARD:
                    #if 0
                    LedIndex = 0;
                    mode = MODE_IDLE;
                    Forward(0xf00000);
                    #else
                    if (LedIndex >= sizeof(IndexForward) / sizeof(uint16_t)) {
                        LedIndex = 0;
                        mode = MODE_IDLE;
                        break;
                    }
                    if (LedIndex == 0) {
                        printf("MODE_FORWARD\r\n");
                        setAllColor_dma(ColorBuf, 0x000000);
                    }
                    LED_LIGHT(0xf00000, IndexForward[LedIndex++], 5);
                    #endif
                    LedCurOn = 1;
                    break;
                case MODE_BACKWARD:
                    #if 0
                    LedIndex = 0;
                    mode = MODE_IDLE;
                    Backward(0xf00000);
                    #else
                    if (LedIndex >= sizeof(IndexBackward) / sizeof(uint16_t)) {
                        LedIndex = 0;
                        mode = MODE_IDLE;
                        break;
                    }
                    if (LedIndex == 0) {
                        printf("MODE_BACKWARD\r\n");
                    }
                    LED_LIGHT(0xf00000, IndexBackward[LedIndex++], 5);
                    #endif
                    LedCurOn = 0;
                    break;
                case MODE_ON:
                    printf("MODE_ON\r\n");
                    LED_CONFIG_ALL(0xf00000);
                    LedCurOn = 1;
                    mode = MODE_IDLE;
                    break;
                case MODE_CLOSE:
                    printf("MODE_CLOSE\r\n");
                    LED_CONFIG_ALL(0x000000);
                    LedCurOn = 0;
                    mode = MODE_IDLE;
                    break;
                case MODE_BREATH:
                    if (LedIndex >= sizeof(IndexHeart) / sizeof(uint16_t)) {
                        LedIndex = 0;
                    }
                    if (LedIndex == 0) {
                        printf("MODE_BREATH\r\n");
                    }
                    LED_LIGHT(0xf00000, IndexHeart[LedIndex++], 10);
                    LedCurOn = 1;
                    break;
                case MODE_MARQUEE:
                    // Marquee(0xf00000);
                    if (LedIndex >= LED_NUM) {
                        LedIndex = 0;
                    }
                    if (LedIndex == 0) {
                        printf("MODE_MARQUEE\r\n");
                    }
                    rgb = (rand() % 255) << 16 | (rand() % 255) << 8 | (rand() % 255) ;
                    setOneColor_dma(&ColorBuf[LedIndex], rgb);
                    TIM1_DMA_Interrupt((u32 *)ColorBuf, (LED_NUM + 2) * 24);
                    PLATFORM_DelayMS(5);
                    setOneColor_dma(&ColorBuf[LedIndex], 0x000000);
                    LedIndex++;
                    LedCurOn = 1;
                    break;
                default:
                    break;
            }
        }
        LastMode = mode;
        r += 21;
    }
}

