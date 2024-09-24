/***********************************************************************************************************************
    @file    ws2812b_tim1_1_pwm_dma.h
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
#ifndef _TIM1_8_PWM_OUTPUT_H_
#define _TIM1_8_PWM_OUTPUT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Files include */
#include "hal_conf.h"

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

/* Exported types *****************************************************************************************************/

/* Exported constants *************************************************************************************************/

/* Exported macro *****************************************************************************************************/

/* Exported variables *************************************************************************************************/

/* Exported functions *************************************************************************************************/
void TIM1_8_Configure(void);
void TIM1_8_PWM_Output_Sample(void);
void LED_CONFIG_ALL(u32 rgb);
void Marquee(u32 rgb);
void Marquee_L2R(u32 rgb);
void Marquee_R2L(u32 rgb);
void Breath(u32 rgb);
void Forward(u32 rgb);
void Backward(u32 rgb);
void setOneColor_dma(dma_color_t *color, uint32_t rgb);
void setAllColor_dma(dma_color_t *color, uint32_t rgb);
void TIM1_DMA_Interrupt(uint32_t *Buffer, uint32_t Length);
#ifdef __cplusplus
}
#endif

#endif /* _TIM1_8_PWM_OUTPUT_H_ */

/********************************************** (C) Copyright MindMotion **********************************************/

