#ifndef _JUDGE_H_
#define _JUDGE_H_

#include "stm32f4xx_hal.h"
#include "CRC.h"
#include <string.h>
#include <bsp_usart.h>

#define VERSION19

typedef union{
	uint8_t c[2];//2�ֽ�
	int16_t d;
	uint16_t ud;
}wl2data;

typedef union{
	uint8_t c[4];
	float f;
	uint32_t d;
}wl4data;

extern int JudgeSendFresh;


//0x0001	����״̬����
typedef __packed struct{ 
	uint8_t game_type : 4; /*1 ���״�ʦ��   
												  *2  ������   
												  *3  ICRA	*/
	uint8_t game_progress : 4;/*0  δ��ʼ���� 
														 *1  ׼���׶�  
														 *2  �Լ�׶�
														 *3  5s����ʱ 
														 *4  ��ս��
														 *5  ����������  */
	uint16_t stage_remain_time; /* ��ǰ�׶�ʣ��ʱ�� s*/
}ext_game_state_t;

// 0x0201
typedef __packed struct {
	uint8_t robot_id;/* ������ID 
									  * 1�� ��Ӣ��
									  * 2�� �칤��
									  * 3/4/5���첽��
									  * 6�� �����
									  * 7�� ���ڱ�
										  * 11����Ӣ��
										  * 12��������
										  * 13/14/15��������		
										  * 16  ������
										  * 17  ���ڱ�*/
	uint8_t robot_level; /* �����˵ȼ� 1/2/3*/
	uint16_t remain_HP; /* ������ʣ��Ѫ�� */
	uint16_t max_HP; /* ����������Ѫ�� */
	uint16_t shooter_id1_17mm_cooling_rate; /* ������17mmǹ��ÿ����ȴֵ */
	uint16_t shooter_id1_17mm_cooling_limit; /* ������17mmǹ���������� */
	uint16_t shooter_id1_17mm_speed_limit;	/* ������17mmǹ���ٶ����� */
	uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;
  uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1; /* ��̨��Դ������ */
	uint8_t mains_power_chassis_output : 1; /* ���̵�Դ������ */
	uint8_t mains_power_shooter_output : 1; /* ����Ħ���ֵ�Դ������ */
} ext_game_robot_state_t;

// 0x020A
typedef __packed struct
{
uint8_t dart_launch_opening_status;  //  ��ǰ���ڷ����״̬
uint8_t dart_attack_target;
uint16_t target_change_time;
uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

extern ext_game_state_t 														Judge_GameState;
extern ext_game_robot_state_t											Judge_GameRobotState;
extern ext_dart_client_cmd_t                       Judge_DartClientCmd;


extern float ToJudgeData[3];

void RobotSendMsgToClient(float data1,float data2,float data3,uint8_t mask);
void JudgeData(uint8_t data);





#endif
