#include "Judge.h"
#include "stdio.h"
uint8_t rbuff[255];
uint8_t HeaderData[20],ReceiveData[255];
uint8_t UpData[50];
int32_t fps;
int16_t datatype;
int JudgeReceivedNewData = 0;

int JudgeSendFresh = 0;
int imu_receive = 0;



float ToJudgeData[3];
uint8_t ToJudgeMask;
uint8_t JudgeReceivedNewDataSignal[16] = {0};

wl2data w2data;
wl4data w4data;

ext_game_state_t 														Judge_GameState;
ext_game_robot_HP_t Judge_game_robot_HP;
ext_game_robot_state_t											Judge_GameRobotState;
ext_dart_client_cmd_t                       Judge_DartClientCmd;

void RobotSendMsgToClient(float data1,float data2,float data3,uint8_t mask){
	static uint8_t seq;
	if(seq == 255) seq = 0;
	seq++;
	UpData[0] 	= 0xA5;
	w2data.d 	= 19;
	UpData[1] 	= w2data.c[0];
	UpData[2] 	= w2data.c[1];
	UpData[3] 	= seq;
	Append_CRC8_Check_Sum(UpData,5);
	/* cmd_id */
	w2data.d 	= 0x0301;
	UpData[5] 	= w2data.c[0];
	UpData[6] 	= w2data.c[1];
	/* content id */
	w2data.d 	= 0xD180;
	UpData[7] 	= w2data.c[0];
	UpData[8] 	= w2data.c[1];
	/* sender id */
	w2data.d 	= Judge_GameRobotState.robot_id;
	UpData[9] 	= w2data.c[0];
	UpData[10] 	= w2data.c[1];
	/* receiver id */
	if(Judge_GameRobotState.robot_id < 9)
		w2data.d 	= Judge_GameRobotState.robot_id | 0x0100;
	else if(Judge_GameRobotState.robot_id > 9 && Judge_GameRobotState.robot_id < 16)
		w2data.d 	= (Judge_GameRobotState.robot_id + 6) | 0x0100;
		
	//printf("send_ig = %x\r\n", w2data.d);
	UpData[11] 	= w2data.c[0];
	UpData[12] 	= w2data.c[1];
	
	/* data send */
	w4data.f 	= data1;
	UpData[13] 	= w4data.c[0];
	UpData[14] 	= w4data.c[1];
	UpData[15] 	= w4data.c[2];
	UpData[16] 	= w4data.c[3];
	w4data.f 	= data2;
	UpData[17] 	= w4data.c[0];
	UpData[18] 	= w4data.c[1];
	UpData[19] 	= w4data.c[2];
	UpData[20] 	= w4data.c[3];
	w4data.f 	= data3;
	UpData[21] 	= w4data.c[0];
	UpData[22] 	= w4data.c[1];
	UpData[23] 	= w4data.c[2];
	UpData[24] 	= w4data.c[3];
	UpData[25] 	= mask;
	/* CRC-check */
	Append_CRC16_Check_Sum(UpData,28);
	//HAL_UART_Transmit_IT(&huart1,UpData,28);
}

void JudgeData(uint8_t data){
	static int HeaderIndex;
	static int dataIndex;
	static int InfoStartReceive;
	static int16_t datalength;
	static uint8_t packindex;
	if(data == 0xA5){
		HeaderIndex = 1;
		HeaderData[0] = data;
		InfoStartReceive = 0;
	}else{
		if(HeaderIndex < 5){
			HeaderData[HeaderIndex++] = data;
			if(HeaderIndex == 5 && Verify_CRC8_Check_Sum(HeaderData,5)){
		//	if(HeaderIndex == 5 ){

				w2data.c[0] = HeaderData[1];
				w2data.c[1] = HeaderData[2];
				datalength = w2data.d;
				packindex = HeaderData[3];
				InfoStartReceive = 1;
				dataIndex = 5;
				memcpy(ReceiveData,HeaderData,5);
				return;
			}
		}
		if(InfoStartReceive){
			if(dataIndex < datalength+9){//9: frame(5)+cmd_id(2)+crc16(2)
				ReceiveData[dataIndex++] = data;
			}
			if(dataIndex == datalength+9){//do the deal once
				InfoStartReceive = 0;
				if(Verify_CRC16_Check_Sum(ReceiveData,datalength+9)){
					w2data.c[0] = ReceiveData[5];
					w2data.c[1] = ReceiveData[6];
					datatype = w2data.d;//cmd_id
					fps++;
					JudgeReceivedNewData = 1;
					switch(datatype){
						
						case 0x0001:{
							JudgeReceivedNewDataSignal[0] = 1;
							memcpy(&Judge_GameState,&ReceiveData[7],sizeof(ext_game_state_t));
							break;
						}
                        case 0x0003:{
                            JudgeReceivedNewDataSignal[1] = 1;
                            memcpy(&Judge_game_robot_HP,&ReceiveData[7],sizeof(ext_game_robot_HP_t));
                            break;
						}
						case 0x0201:{		
							JudgeReceivedNewDataSignal[2] = 1;
							memcpy(&Judge_GameRobotState,&ReceiveData[7],sizeof(ext_game_robot_state_t));
							break;
						}
						case 0x020A:{
                            JudgeReceivedNewDataSignal[3] = 1;
                            memcpy(&Judge_DartClientCmd,&ReceiveData[7],sizeof(ext_dart_client_cmd_t));
						}
						
					}
}
}
}
}
}

