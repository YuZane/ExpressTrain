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
  MODE_Marquee,
};
KeyState_t KeyState;
gesture_info_t gesture;
u8 led_cur_on;
u8 mode;
extern u32 dma_idle;
/***********************************************************************************************************************
  * @brief  This function is main entrance
  * @note   main
  * @param  none
  * @retval none
  *********************************************************************************************************************/
int main(void)
{
    uint8_t KeyCount = 0;
    u8 gsdate[2];
    PLATFORM_Init();
		GPIO_Configure();
    TIM1_8_Configure();
    I2C2_Configure();

    paj7620u2_init();
		paj7620u2_init_gesture();
    EXTI_Configure();
    LED_CONFIG_ALL(0x000000);
    led_cur_on = 0;
    mode = MODE_IDLE;
		// while (1) {
		// 	Gesture_test();
		// }
    while (1)
    {
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

        #if 0
        KEY_FSM_Handler(&KeyState, &KeyCount, GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4), Bit_SET, "K1");
        #endif
        KeyState.intr = 1;
        if (KeyState.update) {
            KeyState.update = 0;
            mode = MODE_KEYUPDATE;
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
                Marquee_R2L(0xf00000);
                break;
              case MODE_RIGHT:
                Marquee_L2R(0xf00000);
                break;
              case MODE_FORWARD:
                Forward(0xf00000);
                break;
              case MODE_BACKWARD:
                Backward(0xf00000);
                break;
              case MODE_ON:
                LED_CONFIG_ALL(0xf00000);
                led_cur_on = 1;
                break;
              case MODE_CLOSE:
                LED_CONFIG_ALL(0x000000);
                led_cur_on = 0;
                break;
              case MODE_BREATH:
                Breath(0xf00000);
                break;
              case MODE_Marquee:
                Marquee(0xf00000);
                break;
              default:
                break;
          }
        }
        mode = MODE_IDLE;
        // PLATFORM_DelayMS(300);
				// printf("1\r\n");
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

