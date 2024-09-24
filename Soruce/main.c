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
#include "i2c_master_interrupt.h"
#include "i2c_master_polling.h"
#include "uart_interrupt.h"
#include "gpio_key_input.h"
#include "exti_interrupt.h"
#include "tim2_5_timebase.h"
#include "tim1_8_pwm_output.h"
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
u8 led_cur_on;
u8 mode;
u8 LastMode;
extern u32 dma_idle;
extern char voice_cmd;
EXTERN volatile UART_RxTx_TypeDef UART_RxStruct;
static dma_color_t ColorBuf[LED_NUM + 4];
extern uint16_t index_heart[215];
extern uint16_t index_forward[116];
extern uint16_t index_backward[116];

#define VOICE_OPEN_LED 0x32
#define VOICE_CLOSE_LED 0x33
#define VOICE_MARQUEE 0x34
#define VOICE_BREATH 0x35

/***********************************************************************************************************************
  * @brief  This function is main entrance
  * @note   main
  * @param  none
  * @retval none
  *********************************************************************************************************************/
int main(void)
{
    static uint8_t led_index = 0;
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
    led_cur_on = 0;
    LastMode = mode = MODE_IDLE;

    // while (1)
    // {
    //   UART_RxData_Interrupt(130);
    //   while (UART_RxStruct.CompleteFlag != 1);
    //   UART_RxStruct.Buffer[131] = '\0';
    //   printf("%s\n", UART_RxStruct.Buffer);
    // }
    
    while (1)
    {
        //gesture
        // gesture.data = GS_Read_Status();
        // if(!gesture.data) {
        //     gesture.update = 1;
        // }
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
        switch (voice_cmd)
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

        //key
        KeyState.intr = 1;
        if (KeyState.update) {
            KeyState.update = 0;
            mode = MODE_KEYUPDATE;
        }

        if (LastMode != mode || mode == MODE_IDLE) {
          led_index = 0;
        }

        if (dma_idle) {
          switch (mode)
          {
              case MODE_KEYUPDATE:
                  if (/*KeyState.status*/ !led_cur_on) {
                    printf("yz debug %s-%d en\n", __FUNCTION__, __LINE__);
                    PLATFORM_LED_Enable(LED1, ENABLE);
                    LED_CONFIG_ALL(0xf00000);
                    led_cur_on = 1;
                  } else {
                    printf("yz debug %s-%d dis\n", __FUNCTION__, __LINE__);
                    PLATFORM_LED_Enable(LED1, DISABLE);
                    LED_CONFIG_ALL(0x000000);
                    led_cur_on = 0;
                  }
                  mode = MODE_IDLE;
                  break;
              case MODE_LEFT:
                // Marquee_R2L(0xf00000);
                if (led_index >= LED_NUM + 1) {
                  led_index = 0;
                }
                if (led_index == 0) {
                  setAllColor_dma(ColorBuf, 0x000000);
                }
                rgb = (rand() % 255) << 16 | (rand() % 255) << 8 | (rand() % 255) ;
                setOneColor_dma(&ColorBuf[LED_NUM + 1 - led_index], rgb);
                TIM1_DMA_Interrupt((u32 *)ColorBuf, (LED_NUM + 2) * 24);
                PLATFORM_DelayMS(10);
                setOneColor_dma(&ColorBuf[LED_NUM + 1 - led_index], 0x000000);
                led_index++;
                break;
              case MODE_RIGHT:
                // Marquee_L2R(0xf00000);
                if (led_index >= LED_NUM + 1) {
                  led_index = 0;
                }
                if (led_index == 0) {
                  setAllColor_dma(ColorBuf, 0x000000);
                }
                rgb = (rand() % 255) << 16 | (rand() % 255) << 8 | (rand() % 255) ;
                setOneColor_dma(&ColorBuf[led_index], rgb);
                TIM1_DMA_Interrupt((u32 *)ColorBuf, (LED_NUM + 2) * 24);
                PLATFORM_DelayMS(10);
                setOneColor_dma(&ColorBuf[led_index], 0x000000);
                led_index++;
                break;
              case MODE_FORWARD:
                // Forward(0xf00000);
                if (led_index >= sizeof(index_forward) / sizeof(uint16_t)) {
                  led_index = 0;
                  mode = MODE_IDLE;
									break;
                }
                if (led_index == 0) {
                  setAllColor_dma(ColorBuf, 0x000000);
                }
                LED_LIGHT(0xf00000, index_forward[led_index++]);
                break;
              case MODE_BACKWARD:
                // Backward(0xf00000);
                if (led_index >= sizeof(index_backward) / sizeof(uint16_t)) {
                  led_index = 0;
                  mode = MODE_IDLE;
									break;
                }
                LED_LIGHT(rgb, index_backward[led_index++]);
                break;
              case MODE_ON:
                LED_CONFIG_ALL(0xf00000);
                led_cur_on = 1;
                mode = MODE_IDLE;
                break;
              case MODE_CLOSE:
                LED_CONFIG_ALL(0x000000);
                led_cur_on = 0;
                mode = MODE_IDLE;
                break;
              case MODE_BREATH:
                if (led_index >= sizeof(index_heart) / sizeof(uint16_t)) {
                  led_index = 0;
                }
                LED_LIGHT(rgb, index_heart[led_index]);
                break;
              case MODE_MARQUEE:
                // Marquee(0xf00000);
                if (led_index >= LED_NUM + 1) {
                  led_index = 0;
                }
                if (led_index == 0) {
                  setAllColor_dma(ColorBuf, 0x000000);
                }
                setOneColor_dma(&ColorBuf[led_index], rgb);
                TIM1_DMA_Interrupt((u32 *)ColorBuf, (LED_NUM + 2) * 24);
                PLATFORM_DelayMS(40);
                setOneColor_dma(&ColorBuf[led_index], 0x000000);
                break;
              default:
                break;
          }
        }
        LastMode = mode;
        r += 21;
    }
}

