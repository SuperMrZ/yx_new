#include "bsp_uart.h"

gambleState gamble_state;

extern SBUS_Buffer SBUS;
extern uint8_t SBUS_RXBuffer[25];//声明遥控器接收缓存数组

int32_t pushrot_M2006_positionTarget;
int16_t M3508Friction_speedTarget[3];
int32_t Load_M3508_positionTarget;
int16_t Load_M3508_speedTarget;

void remoteDecode();

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        //USART_CallBack[0]();
    } else if (huart->Instance == USART3) {
        remoteDecode();
    } else if (huart->Instance == USART6) {
        //USART_CallBack[5]();
    }
}

/// @brief 
/// @return 
void remoteDecode()
{

    SBUS.SE_last = SBUS.SE;
    SBUS.SF_last = SBUS.SF;
    SBUS.SG_last = SBUS.SG;
    SBUS.SH_last = SBUS.SH;
    SBUS.Start=SBUS_RXBuffer[0];
    SBUS.Ch1  = ((int16_t)SBUS_RXBuffer[1] >> 0  | ((int16_t)SBUS_RXBuffer[2] << 8)) & 0x07FF;
    SBUS.Ch2  = ((int16_t)SBUS_RXBuffer[2] >> 3  | ((int16_t)SBUS_RXBuffer[3] << 5)) & 0x07FF;
    SBUS.Ch3  = ((int16_t)SBUS_RXBuffer[3] >> 6  | ((int16_t)SBUS_RXBuffer[4] << 2) | (int16_t)SBUS_RXBuffer[5] << 10) & 0x07FF;
    SBUS.Ch4  = ((int16_t)SBUS_RXBuffer[5] >> 1  | ((int16_t)SBUS_RXBuffer[6] << 7)) & 0x07FF;
    SBUS.Ch5  = ((int16_t)SBUS_RXBuffer[6] >> 4  | ((int16_t)SBUS_RXBuffer[7] << 4)) & 0x07FF;
    SBUS.Ch6  = ((int16_t)SBUS_RXBuffer[7] >> 7  | ((int16_t)SBUS_RXBuffer[8] << 1) | (int16_t)SBUS_RXBuffer[9] << 9) & 0x07FF;
    SBUS.Ch7  = ((int16_t)SBUS_RXBuffer[9] >> 2 | ((int16_t)SBUS_RXBuffer[10] << 6)) & 0x07FF;
    SBUS.Ch8  = ((int16_t)SBUS_RXBuffer[10] >> 5 | ((int16_t)SBUS_RXBuffer[11] << 3)) & 0x07FF;
    SBUS.Ch9  = ((int16_t)SBUS_RXBuffer[12] >> 0 | ((int16_t)SBUS_RXBuffer[13] << 8)) & 0x07FF;
    SBUS.Ch10 = ((int16_t)SBUS_RXBuffer[13] >> 3 | ((int16_t)SBUS_RXBuffer[14] << 5)) & 0x07FF;
    SBUS.Ch11 = ((int16_t)SBUS_RXBuffer[14] >> 6 | ((int16_t)SBUS_RXBuffer[15] << 2) | (int16_t)SBUS_RXBuffer[16] << 10) & 0x07FF;
    SBUS.Ch12 = ((int16_t)SBUS_RXBuffer[16] >> 1 | ((int16_t)SBUS_RXBuffer[17] << 7)) & 0x07FF;
    SBUS.Ch13 = ((int16_t)SBUS_RXBuffer[17] >> 4 | ((int16_t)SBUS_RXBuffer[18] << 4)) & 0x07FF;
    SBUS.Ch14 = ((int16_t)SBUS_RXBuffer[18] >> 7 | ((int16_t)SBUS_RXBuffer[19] << 1) | (int16_t)SBUS_RXBuffer[20] << 9) & 0x07FF;
    SBUS.Ch15 = ((int16_t)SBUS_RXBuffer[20] >> 2 | ((int16_t)SBUS_RXBuffer[21] << 6)) & 0x07FF;
    SBUS.Ch16 = ((int16_t)SBUS_RXBuffer[21] >> 5 | ((int16_t)SBUS_RXBuffer[22] << 3)) & 0x07FF;
    SBUS.Flag = SBUS_RXBuffer[23];
    SBUS.End  = SBUS_RXBuffer[24];
    SBUS.SF = SBUS.Ch6;
    SBUS.SH = SBUS.Ch7;
    SBUS.SE = SBUS.Ch8;
    SBUS.SG = SBUS.Ch9;

    if(SBUS.SF ==353)//急停
    {
        disable_damiao_motor(0x01);
        M3508Friction_currnt[0]=0;
        M3508Friction_currnt[1]=0;
        M3508Friction_currnt[2]=0;
        M3508Friction_currnt[3]=0;
        M3508Load_currnt       =0;
        M2006Pushrop_currnt    =0;


    }
    if(SBUS.SF == 1695)
    {
        enable_damiao_motor(0x01);
        if(SBUS.SH == 1695 && SBUS.SH_last == 353 && gamble_state.bullet ==1 && gamble_state.pushrot_position ==2)
        {
            Load_M3508_positionTarget += 8192*10;
        }

        if(SBUS.SG == 1024)
        {
            Load_M3508_speedTarget =0;
        }
        if(SBUS.SG == 353)
        {
            Load_M3508_speedTarget =-500;
        }
        if(SBUS.SG == 1695)
        {
            Load_M3508_speedTarget =500;
        }


        if (SBUS.SE == 353)
        {
            M3508Friction_speedTarget[0]=0;
            M3508Friction_speedTarget[1]=0;
            M3508Friction_speedTarget[2]=0;
        }

        if (SBUS.SE == 1024)
        {
            M3508Friction_speedTarget[0]=8000;
            M3508Friction_speedTarget[1]=-8000;
            M3508Friction_speedTarget[2]=8000;
        }

    }



    // if(SBUS.SG ==1695 && SBUS.SG_last ==1024 && gamble_state.bullet ==1 && gamble_state.pushrot_position ==2)
    // {
    //     //2006旋转圈数
    //     pushrot_M2006_positionTarget += 8192*10;//目标转10转

    // }

    //   if(SBUS.SG ==1024 && SBUS.SG_last ==353 && gamble_state.bullet ==1 && gamble_state.pushrot_position ==2)
    // {
    //     //2006旋转圈数
    //     pushrot_M2006_positionTarget += 8192*10;//目标转10转

    // } 
    //      if(SBUS.SG == 1024 && SBUS.SG_last == 1695 & gamble_state.bullet ==1 && gamble_state.pushrot_position ==2)
    // {
    //     pushrot_M2006_positionTarget -= 8192*10;
    // }
    //      if(SBUS.SG == 353 && SBUS.SG_last == 1024 & gamble_state.bullet ==1 && gamble_state.pushrot_position ==2)
    // {

    //     pushrot_M2006_positionTarget -= 8192*10;
    // }

    // if(SBUS.SH == 1695 && SBUS.SH_last == 353 && gamble_state.bullet ==0 && gamble_state.pushrot_position ==2)




    
    


    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,SBUS_RXBuffer,25);						
	__HAL_DMA_DISABLE_IT(huart3.hdmarx ,DMA_IT_HT );


}






