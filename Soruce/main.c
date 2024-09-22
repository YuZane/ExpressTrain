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

/***********************************************************************************************************************
  * @brief  This function is main entrance
  * @note   main
  * @param  none
  * @retval none
  *********************************************************************************************************************/
int main(void)
{
    KeyState_t KeyState = {0, 0};
    uint8_t KeyCount = 0;
  
    PLATFORM_Init();
		GPIO_Configure();
    TIM1_8_Configure();

    while (1)
    {
      KEY_FSM_Handler(&KeyState, &KeyCount, GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4), Bit_SET, "K1");

      if (KeyState.update) {
        KeyState.update = 0;
        if (KeyState.status) {
          printf("yz debug %s-%d en\n", __FUNCTION__, __LINE__);
          PLATFORM_LED_Enable(LED1, ENABLE);
          LED_CONFIG_ALL(0x0f0000);
        } else {
          printf("yz debug %s-%d dis\n", __FUNCTION__, __LINE__);
          PLATFORM_LED_Enable(LED1, DISABLE);
          LED_CONFIG_ALL(0x000000);
        }
      }
    }

#if 0
    //i2c2
    GPIO_KEY_Input_Sample();
    TIM1_8_PWM_Output_Sample();
    TIM2_5_TimeBase_Sample();
    EXTI_Interrupt_Sample();
    I2C_Master_Polling_Sample();
		//uart2
    UART_Interrupt_Sample();
		

    while (1)
    {
    }
    #endif
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

