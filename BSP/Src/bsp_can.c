#include "bsp_can.h"

/* Private variables ---------------------------------------------------------*/
CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;
CAN_TxFrameTypeDef hcan1TxFrame;
CAN_TxFrameTypeDef hcan2TxFrame;


extern motorReceiveInfo M3508Friction[4];
extern motorReceiveInfo M2006Pushrop;
extern motorReceiveInfo M3508Load;
extern damiao_recieve damiao_recieve_pitch;
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  BspCan1Init此函数用于can1的初始化
  * @param  
  * @return 返回值
  * @retval 无
  * @note    目前过滤器只配置了一个，并且为全通
  * @note   
  */

void BspCan1Init() {
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;//FIFO0
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        Error_Handler();
    }
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  BspCan2Init此函数用于can2的初始化
  * @param  
  * @return 返回值
  * @retval 无
  * @note    目前过滤器只配置了一个，并且为全通
  * @note   
  */

void BspCan2Init() {
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;//FIFO0
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK) {
        Error_Handler();
    }
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}



/**
 *	@brief	CAN 接收中断回调函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
        //在下面进行解码begin
        switch (hcan1RxFrame.header.StdId)
        {
        case 0x114:
        {
           	damiao_recieve_pitch.p=(hcan1RxFrame.data[1] << 8) |hcan1RxFrame.data[2];
		    damiao_recieve_pitch.v=(hcan1RxFrame.data[3] << 4) |(hcan1RxFrame.data[4] >> 4);
		    damiao_recieve_pitch.t=((hcan1RxFrame.data[4]&0xF) << 8) |hcan1RxFrame.data[5];


            damiao_recieve_pitch.position = uint_to_float(damiao_recieve_pitch.p, -12.5, 12.5, 16); // (-12.5, 12.5)
            damiao_recieve_pitch.velocity = uint_to_float(damiao_recieve_pitch.v, -45, 45, 12); // (-45.0, 45.0)
            damiao_recieve_pitch.torque = uint_to_float(damiao_recieve_pitch.t, -18, 18, 12);   // (-18.0, 18.0) 

        }
        break;

        case 0x201:
        case 0x202:
        case 0x203:
        {
          	int16_t i=0;
			i=hcan1RxFrame.header.StdId-0x201;
			M3508Friction[i].ecd=(hcan1RxFrame.data[0]<<8)|hcan1RxFrame.data[1];//转子机械角度
		    M3508Friction[i].speed_rpm=(hcan1RxFrame.data[2]<<8)|hcan1RxFrame.data[3];//转子转速
	    	M3508Friction[i].given_current=(hcan1RxFrame.data[4]<<8)|hcan1RxFrame.data[5];//实际转矩电流
		    M3508Friction[i].temperate=hcan1RxFrame.data[6];//电机温度	
        }
            break;
        case 0x204:
        {
            M3508Load.ecd=(hcan1RxFrame.data[0]<<8)|hcan1RxFrame.data[1];//转子机械角度
		    M3508Load.speed_rpm=(hcan1RxFrame.data[2]<<8)|hcan1RxFrame.data[3];//转子转速
	    	M3508Load.given_current=(hcan1RxFrame.data[4]<<8)|hcan1RxFrame.data[5];//实际转矩电流
		    M3508Load.temperate=hcan1RxFrame.data[6];//电机温度 
        }
            break;
        
        case 0x205:
        {
            M2006Pushrop.ecd=(hcan1RxFrame.data[0]<<8)|hcan1RxFrame.data[1];//转子机械角度
		    M2006Pushrop.speed_rpm=(hcan1RxFrame.data[2]<<8)|hcan1RxFrame.data[3];//转子转速
	    	M2006Pushrop.given_current=(hcan1RxFrame.data[4]<<8)|hcan1RxFrame.data[5];//实际转矩电流
		    M2006Pushrop.temperate=hcan1RxFrame.data[6];//电机温度	

        }
            /* code */
            break;
        
        default:
            break;
        }


        //解码end
	}
	else if(hcan->Instance == CAN2)
	{

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
		//在下面进行解码begin


        //解码end
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)//这个地方有疑问
	{

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
        //在下面进行解码begin


        //解码end
	}
	else if(hcan->Instance == CAN2)
	{

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
		//在下面进行解码begin


        //解码end
	}
}




uint8_t CAN_SendData(int8_t can, uint32_t stdId, int16_t *dat)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef *txFrame;
	
	if(can==1)
	{
		while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0 );
		txFrame = &hcan1TxFrame;
	}
	else if(can==2)
	{
		while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan2 ) == 0 );
		txFrame = &hcan2TxFrame;
	}
	else
		return HAL_ERROR;
	
	txFrame->header.StdId = stdId;
	txFrame->header.IDE = CAN_ID_STD;
	txFrame->header.RTR = CAN_RTR_DATA;
	txFrame->header.DLC = 8;
	
	// 先发高8位数据，再发低8位数据
	txFrame->data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txFrame->data[1] = (uint8_t)((int16_t)dat[0]);
	txFrame->data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txFrame->data[3] = (uint8_t)((int16_t)dat[1]);
	txFrame->data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	txFrame->data[5] = (uint8_t)((int16_t)dat[2]);
	txFrame->data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	txFrame->data[7] = (uint8_t)((int16_t)dat[3]);		

	if(can == 1)
    {
        if(HAL_CAN_AddTxMessage(&hcan1, &txFrame->header, &txFrame->data[0], &txMailBox) != HAL_OK)
        {
		return HAL_ERROR;
	    }
    }
	if(can == 2)
    {
        if(HAL_CAN_AddTxMessage(&hcan2, &txFrame->header, &txFrame->data[0], &txMailBox) != HAL_OK)
        {
		return HAL_ERROR;
	    }
    }
	
	return HAL_OK;
}




