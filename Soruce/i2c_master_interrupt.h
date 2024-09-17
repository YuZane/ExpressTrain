/***********************************************************************************************************************
    @file    i2c_master_eeprom_interrupt.h
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
#ifndef _I2C_MASTER_EEPROM_INTERRUPT_H_
#define _I2C_MASTER_EEPROM_INTERRUPT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Files include */
#include "hal_conf.h"

/* Exported types *****************************************************************************************************/
typedef struct
{
    uint8_t Buffer[255];
    uint8_t Length;
    uint8_t CurrentCount;
    uint8_t CompleteFlag;
} I2C_RxTx_TypeDef;

/* Exported constants *************************************************************************************************/

/* Exported macro *****************************************************************************************************/

/* Exported variables *************************************************************************************************/
#undef EXTERN

#ifdef _I2C_MASTER_EEPROM_INTERRUPT_C_
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN volatile I2C_RxTx_TypeDef I2C_RxStruct;
EXTERN volatile I2C_RxTx_TypeDef I2C_TxStruct;

/* Exported functions *************************************************************************************************/
void I2C_Master_EEPROM_Interrupt_Sample(void);

#ifdef __cplusplus
}
#endif

#endif /* _I2C_MASTER_EEPROM_INTERRUPT_H_ */

/********************************************** (C) Copyright MindMotion **********************************************/

