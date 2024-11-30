#include "bsp_uart.h"



extern SBUS_Buffer SBUS;
extern uint8_t SBUS_RXBuffer[25];//声明遥控器接收缓存数组



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

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,SBUS_RXBuffer,25);						
	__HAL_DMA_DISABLE_IT(huart3.hdmarx ,DMA_IT_HT );

}




