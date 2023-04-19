#ifndef CHASSIS_CONTROL_H_
#define CHASSIS_CONTROL_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "STMGood.h"
#include "pid.h"

#define SPEED_SMOOTH_COEF 0.01f

typedef float fp32;
typedef double fp64;

typedef struct
{
	int16_t esc_back_position;       //电机电调返回的位置(码盘值)
	int16_t esc_back_speed;          //电机电调返回的转速
	int16_t esc_back_given_current;  //电机电调返回的电流
	int16_t esc_back_temperature;    //电机电调返回的温度
	int16_t esc_back_angle;          //码盘值角度
	/* 处理成连续的码盘值 */
	uint8_t process_position_flag;   //处理码盘值标志
	int16_t esc_back_position_last;  //电调返回的上一次位置(码盘值) 
	int16_t esc_back_speed_last;
	int16_t esc_back_angle_last;
	int64_t circle_num;              //电机旋转圈数
	int64_t serial_position;         //最终处理后的码盘值
	int64_t position_angle;          //将码盘处理成角度
	int16_t mid_position;            //位置中值 
	int16_t max_position;            //位置最大值
	int16_t min_position;            //位置最小值
	int16_t init_position;           //速度为零时位置
	int16_t init_serial_positon;      //速度为零时真实位置
	int16_t num_position;              //第一圈所转的编码值
	int16_t angle_eer;
	int16_t serial_angle;
	int16_t eer_eer;
	int receive_no;	//中断次数（速度大于400）
	int64_t sum_speed;	//返回速度总和
	float average_speed;	//滤波后速度
	/* 目标值 */
	float   target_speed;            //目标速度
	float  target_position;         //目标位置
	float   target_angle;            //目标角度
	/* 电机输出电流 */
	int16_t out_current;
	int16_t back_current;
} motor_data_t;



typedef struct
{
	uint8_t Data[8];
}CAN_Message;

extern s_pid_absolute_t  MOTOR_S_PID;
extern s_pid_absolute_t  MOTOR_P_PID;

extern s_pid_absolute_t  YAW_S_PID;
extern s_pid_absolute_t  YAW_P_PID;

extern s_pid_absolute_t  PITCH_S_PID;
extern s_pid_absolute_t  PITCH_P_PID;

extern s_pid_absolute_t  FIRE_L_S_PID;
extern s_pid_absolute_t  FIRE_R_S_PID;

extern s_pid_absolute_t  DEVIATION_PID;

extern motor_data_t motor ;
extern motor_data_t yaw ;
extern motor_data_t pitch ;
extern motor_data_t fire_r ;
extern motor_data_t fire_l ;


extern int16_t deviation_out;


void chassis_control_loop_init(void);
void chassis_control_loop_reset(void);
void chassis_motor_control_loop_pid_control(void);
void zero_current(void);
float YL_error_correction(float YL);
float FL_error_correction(void);

#endif


