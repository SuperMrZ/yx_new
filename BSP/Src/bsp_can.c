#include "bsp_can.h"

/* Private variables ---------------------------------------------------------*/
CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;
CAN_TxFrameTypeDef hcan1TxFrame;
CAN_TxFrameTypeDef hcan2TxFrame;

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



