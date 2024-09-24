#include <stdio.h>
#include "platform.h"
#include "i2c_master_polling.h"
#include "paj7620u2.h"
#include "paj7620u2_cfg.h"

#define u8 unsigned char
#define u16 unsigned short

u8 GS_Write_Byte(u8 REG_Address,u8 REG_data)
{
    return I2C_Write(REG_Address, &REG_data, 1);
}

u8 GS_Read_Byte(u8 REG_Address)
{
    unsigned char tmp = 0;
    I2C_Read(REG_Address, &tmp, 1);
    return tmp;
}

u8 GS_Read_nByte(u8 REG_Address,u16 len,u8 *buf)
{
    unsigned char tmp = 0,ret = 0;

    ret  = I2C_Read(REG_Address, buf, len);
    if(ret) return 0;
    else return 1;
}

// //PAJ7620唤醒
// void GS_WakeUp(void)
// {
//     i2c_1_write(0x73, 0, 0, 0, 0);
// }

//选择PAJ7620U2 BANK区域
void paj7620u2_selectBank(bank_e bank)
{
    switch(bank)
    {
    case BANK0: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK0);break;//BANK0寄存器区域
    case BANK1: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK1);break;//BANK1寄存器区域
    }
}

//PAJ7620U2唤醒
// u8 paj7620u2_wakeup(void)
// { 
//     u8 data=0x0a;
//     GS_WakeUp();//唤醒PAJ7620U2
//     delay_ms(5);//唤醒时间>400us
//     GS_WakeUp();//唤醒PAJ7620U2
//     delay_ms(5);//唤醒时间>400us
//     paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
//     data = GS_Read_Byte(0x00);//读取状态
//     if(data!=0x20) return 0; //唤醒失败
    
//     return 1;
// }

//PAJ7620U2初始化
//返回值：0:失败 1:成功
u8 paj7620u2_init(void)
{
    u8 i;
    static u8 status = 0;

    //     GS_i2c_init();//IIC初始化
    //    status = paj7620u2_wakeup();//唤醒PAJ7620U2
    // if(!status) return 0;
    if(status) return 0;
    paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
    for(i=0;i<INIT_SIZE;i++)
    {
        GS_Write_Byte(init_Array[i][0],init_Array[i][1]);//初始化PAJ7620U2
    }
    paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
    status = 1;
    return 1;
}

void paj7620u2_init_gesture(void)
{
    int i;
    paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
    for(i=0;i<GESTURE_SIZE;i++)
    {
        GS_Write_Byte(gesture_arry[i][0],gesture_arry[i][1]);//手势识别模式初始化
    }
    paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
}


u16 GS_Read_Status(void)
{
    int status;
    u8 data[2]={0x00};
    status = GS_Read_nByte(PAJ_GET_INT_FLAG1, 2, data);//读取手势状态
    if(!status) {
        return (u16)data[1]<<8 | data[0];
    }
    return 0;
}
//手势识别测试
u8 Gesture_test()
{
    u8 i;
    u8 status;
    u8 data[2]={0x00};
    u16 gesture_data;
    static u8 state = 1;
    
    if(state == 0)
    {
        paj7620u2_init();
        paj7620u2_init_gesture();
        state = 1;
    }
    else
    {
        status = GS_Read_nByte(PAJ_GET_INT_FLAG1,2,&data[0]);//读取手势状态            
        if(!status) {
            gesture_data =(u16)data[1]<<8 | data[0];
            if(gesture_data) {
                // *ges = gesture_data;
                switch (gesture_data)
                {
                    case PAJ_UP:    printf("Up\r\n");                break;
                    case PAJ_DOWN:    printf("Down\r\n");                break;
                    case PAJ_LEFT:
                        printf("Left\r\n");
                        PLATFORM_LED_Enable(LED1, 0);
                    break;
                    case PAJ_RIGHT:
                        printf("Right\r\n");
                        PLATFORM_LED_Enable(LED1, 1);
                    break;
                    case PAJ_FORWARD:    printf("Forward\r\n");            break;
                    case PAJ_BACKWARD:    printf("Backward\r\n");         break;
                    case PAJ_CLOCKWISE:    printf("Clockwise\r\n");         break;
                    case PAJ_COUNT_CLOCKWISE:    printf("AntiClockwise\r\n");     break;
                    case PAJ_WAVE:    printf("Wave\r\n");             break;
                    default: break;
                }
                gesture_data=0;
            }
        }
    }
    return 0;
}

//接近检测测试
u8 Ps_test(u8 *brightness, u16 *size)
{
    u8 i;
    u8 data[2]={0x00};
    u8 obj_brightness=0;
    u16 obj_size=0;
    static u8 state = 0;
    
    if(state == 0)
    {
        paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
        for(i=0;i<PROXIM_SIZE;i++)
        {
            GS_Write_Byte(proximity_arry[i][0],proximity_arry[i][1]);//接近检测模式初始化
        }
        paj7620u2_selectBank(BANK0);//返回BANK0寄存器区域
    }
    else
    {    
        obj_brightness = GS_Read_Byte(PAJ_GET_OBJECT_BRIGHTNESS);//读取物体亮度
        data[0] = GS_Read_Byte(PAJ_GET_OBJECT_SIZE_1);//读取物体大小
        data[1] = GS_Read_Byte(PAJ_GET_OBJECT_SIZE_2);
        obj_size = ((u16)data[1] & 0x0f)<<8 | data[0];
    }
    
    return 0;
}
