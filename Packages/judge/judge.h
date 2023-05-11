#ifndef _JUDGE_H_
#define _JUDGE_H_

#include "stm32f4xx_hal.h"
#include "CRC.h"
#include <string.h>
#include <bsp_usart.h>

#define VERSION19

typedef union{
	uint8_t c[2];//2字节
	int16_t d;
	uint16_t ud;
}wl2data;

typedef union{
	uint8_t c[4];
	float f;
	uint32_t d;
}wl4data;

extern int JudgeSendFresh;


//0x0001	比赛状态数据
typedef __packed struct{ 
	uint8_t game_type : 4; /*1 机甲大师赛   
												  *2  单项赛   
												  *3  ICRA	*/
	uint8_t game_progress : 4;/*0  未开始比赛 
														 *1  准备阶段  
														 *2  自检阶段
														 *3  5s倒计时 
														 *4  对战中
														 *5  比赛结算中  */
	uint16_t stage_remain_time; /* 当前阶段剩余时间 s*/
}ext_game_state_t;

// 0x0201
typedef __packed struct {
	uint8_t robot_id;/* 机器人ID 
									  * 1： 红英雄
									  * 2： 红工程
									  * 3/4/5：红步兵
									  * 6： 红空中
									  * 7： 红哨兵
										  * 11：蓝英雄
										  * 12：蓝工程
										  * 13/14/15：蓝步兵		
										  * 16  蓝空中
										  * 17  蓝哨兵*/
	uint8_t robot_level; /* 机器人等级 1/2/3*/
	uint16_t remain_HP; /* 机器人剩余血量 */
	uint16_t max_HP; /* 机器人上限血量 */
	uint16_t shooter_id1_17mm_cooling_rate; /* 机器人17mm枪口每秒冷却值 */
	uint16_t shooter_id1_17mm_cooling_limit; /* 机器人17mm枪口热量上限 */
	uint16_t shooter_id1_17mm_speed_limit;	/* 机器人17mm枪口速度上限 */
	uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;
  uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1; /* 云台电源输出情况 */
	uint8_t mains_power_chassis_output : 1; /* 底盘电源输出情况 */
	uint8_t mains_power_shooter_output : 1; /* 发射摩擦轮电源输出情况 */
} ext_game_robot_state_t;

// 0x020A
typedef __packed struct
{
uint8_t dart_launch_opening_status;  //  当前飞镖发射口状态
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
