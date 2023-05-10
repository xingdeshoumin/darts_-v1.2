#ifndef BSP_USART_H
#define BSP_USART_H

#include "stm32f4xx.h"
#include "stdio.h"
#include "usart.h"
#include "STMGood.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint8_t usart3_updated_flag;


void USART_Enable(UART_HandleTypeDef *huart,uint8_t * buffer_addr,uint8_t buff_size);
void USART_Send_Char(UART_HandleTypeDef *huart,uint8_t u8_char);
void USART_DMA_Enable(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma,uint8_t * buffer_addr,uint8_t data_size);
void USART1_IDLE_IRQ(void);


extern uint8_t usart1_rx_buffer[18];
extern uint8_t usart6_rx_buffer[30];
extern uint8_t usart3_rx_buffer[18];
extern uint8_t usart7_rx_buffer;
extern int fputc(int ch, FILE *f);
void deal_uart_IT(void);
#endif
