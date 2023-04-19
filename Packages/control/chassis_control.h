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
	int16_t esc_back_position;       //���������ص�λ��(����ֵ)
	int16_t esc_back_speed;          //���������ص�ת��
	int16_t esc_back_given_current;  //���������صĵ���
	int16_t esc_back_temperature;    //���������ص��¶�
	int16_t esc_back_angle;          //����ֵ�Ƕ�
	/* ���������������ֵ */
	uint8_t process_position_flag;   //��������ֵ��־
	int16_t esc_back_position_last;  //������ص���һ��λ��(����ֵ) 
	int16_t esc_back_speed_last;
	int16_t esc_back_angle_last;
	int64_t circle_num;              //�����תȦ��
	int64_t serial_position;         //���մ���������ֵ
	int64_t position_angle;          //�����̴���ɽǶ�
	int16_t mid_position;            //λ����ֵ 
	int16_t max_position;            //λ�����ֵ
	int16_t min_position;            //λ����Сֵ
	int16_t init_position;           //�ٶ�Ϊ��ʱλ��
	int16_t init_serial_positon;      //�ٶ�Ϊ��ʱ��ʵλ��
	int16_t num_position;              //��һȦ��ת�ı���ֵ
	int16_t angle_eer;
	int16_t serial_angle;
	int16_t eer_eer;
	int receive_no;	//�жϴ������ٶȴ���400��
	int64_t sum_speed;	//�����ٶ��ܺ�
	float average_speed;	//�˲����ٶ�
	/* Ŀ��ֵ */
	float   target_speed;            //Ŀ���ٶ�
	float  target_position;         //Ŀ��λ��
	float   target_angle;            //Ŀ��Ƕ�
	/* ���������� */
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


