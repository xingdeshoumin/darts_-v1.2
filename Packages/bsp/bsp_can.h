#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx.h"

extern CAN_HandleTypeDef hcan1;

void CAN_Enable(CAN_HandleTypeDef *hcan);
void CANFilter_Enable(CAN_HandleTypeDef *hcan);
//void CAN_cmd_chassis(int16_t motor1,int16_t motor2);
//void CAN_cmd_yaw(int16_t motor1,int16_t motor2);
void CANTx_SendCurrent(CAN_HandleTypeDef *hcan,uint32_t id,int16_t current1,int16_t current2,int16_t current3,int16_t current4);

#endif
