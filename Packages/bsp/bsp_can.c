#include "bsp_can.h"
#include "can.h"
#include "chassis_control.h"
#include "chassis_behavior.h"

extern CAN_HandleTypeDef hcan1;

int receive_limt=7100;//接收限制转速的信息

void CAN_Enable(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_Start(hcan);//对can进行激活
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);//使能can接收中断
	CANFilter_Enable(hcan);//使能滤波器
}


void CANFilter_Enable(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef filter1;
	
	if(hcan->Instance == CAN1)
	{
		filter1.FilterActivation=ENABLE;
		filter1.FilterBank=0U;
		filter1.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		filter1.FilterIdHigh=0x0000;
		filter1.FilterIdLow=0x0000;
		filter1.FilterMaskIdHigh=0x0000;
		filter1.FilterMaskIdLow=0x0000;
		filter1.FilterMode=CAN_FILTERMODE_IDMASK;
		filter1.FilterScale=CAN_FILTERSCALE_32BIT;
		filter1.SlaveStartFilterBank=14;
		
		HAL_CAN_ConfigFilter(&hcan1,&filter1);
	}

	
}
/*
void CAN_cmd_chassis(int16_t motor1,int16_t motor2)
{
 CAN_TxHeaderTypeDef  chassis_tx_message;
 uint8_t              chassis_can_send_data[4];
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
	  chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_cmd_yaw(int16_t motor1,int16_t motor2)
{
 CAN_TxHeaderTypeDef  chassis_tx_message;
 uint8_t              chassis_can_send_data[4];
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x1FF;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
	  chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
*/
void CANTx_SendCurrent(CAN_HandleTypeDef *hcan,uint32_t id,int16_t current1,int16_t current2,int16_t current3,int16_t current4)
{
    CAN_Message tx_message;
	CAN_TxHeaderTypeDef CanTxHeader;
	
	CanTxHeader.StdId = id;
	CanTxHeader.IDE = CAN_ID_STD;
	CanTxHeader.RTR = CAN_RTR_DATA;
	CanTxHeader.DLC = 0x08;

	tx_message.Data[0]=(unsigned char)(current1>>8);
	tx_message.Data[1]=(unsigned char)current1;
	tx_message.Data[2]=(unsigned char)(current2>>8);
	tx_message.Data[3]=(unsigned char)current2;
	tx_message.Data[4]=(unsigned char)(current3>>8);
	tx_message.Data[5]=(unsigned char)current3;
	tx_message.Data[6]=(unsigned char)(current4>>8);
	tx_message.Data[7]=(unsigned char)current4;

	HAL_CAN_AddTxMessage(hcan,&CanTxHeader,tx_message.Data,(uint32_t*)CAN_TX_MAILBOX0);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Message can1_rx_message;
	CAN_RxHeaderTypeDef Can1RxHeader;
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Can1RxHeader,can1_rx_message.Data);
	    switch (Can1RxHeader.StdId)
		{
			case 0x201:
			{
				motor.esc_back_position = can1_rx_message.Data[0]<<8 | can1_rx_message.Data[1];
			  motor.esc_back_speed = can1_rx_message.Data[2]<<8 | can1_rx_message.Data[3];//电调返回速度
				motor.esc_back_given_current = can1_rx_message.Data[4]<<8 | can1_rx_message.Data[5];//电调返回扭矩电流
				motor.esc_back_temperature = can1_rx_message.Data[6];//电调返回温度
				process_motor_encoder_to_serial(&motor);
				if(motor.receive_no==0)
				{
					motor.num_position = motor.esc_back_position;
				}
				motor.receive_no++;
				break;
			}
			case 0x202:
			{
				fire_l.esc_back_position = can1_rx_message.Data[0]<<8 | can1_rx_message.Data[1];
				fire_l.esc_back_speed_last = fire_l.esc_back_speed;
			  	fire_l.esc_back_speed = can1_rx_message.Data[2]<<8 | can1_rx_message.Data[3];//电调返回速度
				fire_l.esc_back_given_current = can1_rx_message.Data[4]<<8 | can1_rx_message.Data[5];//电调返回扭矩电流
				fire_l.esc_back_temperature = can1_rx_message.Data[6];//电调返回温度
				process_motor_encoder_to_serial(&fire_l);
				if(fire_l.esc_back_speed>receive_limt||fire_l.esc_back_speed<-receive_limt)
				{
					fire_l.sum_speed+=fire_l.esc_back_speed;
					fire_l.receive_no++;
					fire_l.average_speed=fire_l.sum_speed/fire_l.receive_no;
				}
				break;
			}
			
			case 0x203:
			{
				pitch.esc_back_position = can1_rx_message.Data[0]<<8 | can1_rx_message.Data[1];
			  pitch.esc_back_speed = can1_rx_message.Data[2]<<8 | can1_rx_message.Data[3];//电调返回速度
				pitch.esc_back_given_current = can1_rx_message.Data[4]<<8 | can1_rx_message.Data[5];//电调返回扭矩电流
				pitch.esc_back_temperature = can1_rx_message.Data[6];//电调返回温度
				process_motor_encoder_to_serial(&pitch);
				if(pitch.receive_no==0)
				{
					pitch.num_position = pitch.esc_back_position;
				}
				pitch.receive_no++;
				break;
			
			}
			case 0x204:
			{
				fire_r.esc_back_position = can1_rx_message.Data[0]<<8 | can1_rx_message.Data[1];
				fire_r.esc_back_speed_last = fire_r.esc_back_speed;
			  	fire_r.esc_back_speed = can1_rx_message.Data[2]<<8 | can1_rx_message.Data[3];//电调返回速度
				fire_r.esc_back_given_current = can1_rx_message.Data[4]<<8 | can1_rx_message.Data[5];//电调返回扭矩电流
				fire_r.esc_back_temperature = can1_rx_message.Data[6];//电调返回温度
				process_motor_encoder_to_serial(&fire_r);
				if(fire_r.esc_back_speed>receive_limt||fire_r.esc_back_speed<-receive_limt)
				{
					fire_r.sum_speed+=fire_r.esc_back_speed;
					fire_r.receive_no++;
					fire_r.average_speed=fire_r.sum_speed/fire_r.receive_no;
				}
				
				break;
			}
			case 0x206:
			{
			  yaw.esc_back_position = can1_rx_message.Data[0]<<8 | can1_rx_message.Data[1];
			  yaw.esc_back_speed = can1_rx_message.Data[2]<<8 | can1_rx_message.Data[3];//电调返回速度
				yaw.esc_back_given_current = can1_rx_message.Data[4]<<8 | can1_rx_message.Data[5];//电调返回扭矩电流
				yaw.esc_back_temperature = can1_rx_message.Data[6];//电调返回温度
				process_motor_encoder_to_serial(&yaw);
				if(yaw.receive_no==0)
				{
					yaw.num_position = yaw.esc_back_position;
				}
				yaw.receive_no++;
				break;
			
			}
		}
	}
}
