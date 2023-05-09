#include "STMgood.h"
#include "bsp_usart.h"
#include "usart.h"
#include "stdio.h"
#include "bsp_dubs.h"
#include "judge.h"

uint8_t usart1_rx_buffer[18]; // SBUS
uint8_t usart6_rx_buffer[30] = {0}; // 裁判系统
uint8_t usart3_rx_buffer[18] = {0}; // 激光测距
uint8_t usart7_rx_buffer; // DEBUG

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart7_rx;

void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_addr,uint8_t buff_size)
{
	HAL_UART_Receive_IT(huart,buffer_addr,buff_size);
	__HAL_UART_ENABLE_IT(huart,UART_IT_ERR);	
}


void USART_Send_Char(UART_HandleTypeDef *huart,uint8_t u8_char)
{
		while((huart->Instance->SR&0X40)==0); 
		huart->Instance->DR = u8_char;
}

void USART_DMA_Enable(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma,uint8_t * buffer_addr,uint8_t data_size)
{
	 __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);								/*!<使能串口的中断为空闲中断    */
	 HAL_UART_Receive_DMA(huart,buffer_addr,data_size);								/*!<DMA Receive data            */
	 __HAL_UART_ENABLE_IT(huart,UART_IT_ERR);								/*!<Enable Usart Error IT      	*/
}

void USART1_IDLE_IRQ(void)
{	
 	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
	{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);	//清除标志位和SR，DR寄存器
			get_dbus_data(usart1_rx_buffer);
		  HAL_UART_DMAStop(&huart1);
			HAL_UART_Receive_DMA(&huart1,usart1_rx_buffer,18);//函数中包括重新配置DMA
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		//Dealdata(usart6_r6_buffer[0]);
		JudgeData(usart6_rx_buffer[0]);
		__HAL_UART_CLEAR_PEFLAG(&huart6);//清除中断标志位
		HAL_UART_Receive_IT(&huart6,usart6_rx_buffer,1);//使能串口2
	}
//	if(huart->Instance == UART7)
//	{
//		Dealdata(usart7_rx_buffer);
//		//JudgeData(usart7_rx_buffer);
//		__HAL_UART_CLEAR_PEFLAG(&huart7);//清除中断标志位
//		HAL_UART_Receive_IT(&huart7,&usart7_rx_buffer,1);//使能串口2
//	}
    if(huart->Instance == USART3)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart3);//清除中断标志位
		HAL_UART_Receive_IT(&huart3,usart3_rx_buffer,11);//使能串口2
	}

}

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{   
	if(huart->Instance == USART1)
	 RemoteDataProcess(dbus_rx_buffer);
		
	
   if(huart->Instance == USART6)
   {  
	  Dealdata(usart6_rx_buffer[0]);//如果格式符合上位机的格式，则对相应变量进行赋值（p，i，d）
	  __HAL_UART_CLEAR_PEFLAG(&huart6 );//清除中断标志位
	  HAL_UART_Receive_IT(&huart6 , usart6_rx_buffer, 1);
		
   }
}*/


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
   if (huart->ErrorCode == HAL_UART_ERROR_ORE)	   
   {
	  __HAL_UART_CLEAR_OREFLAG(huart);//清除错误标志位，清空SR、DR寄存器
   }
 }

int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	while((USART6->SR&0X40)==0); 
	USART6->DR = (uint8_t) ch;      
	return ch;
}

void Usart_SendString(uint8_t *str)
{
    unsigned int k=0;
    do 
	{
      HAL_UART_Transmit(&huart1,(uint8_t *)(str + k) ,1,1000);
      k++;
    } 
	while (*(str + k)!='\0');
}
