#ifndef CHASSIS_BEHAVOR_H
#define CHASSIS_BEHAVOR_H

#include "stm32f4xx.h"
#include "chassis_control.h"

typedef enum
{
    UNTRIGGERED = 0,
    TRIGGERED,
} Switch_Status_e;

typedef struct
{
	uint8_t color;			// 0 blue 1 red
	uint8_t num;			// num
	float delta_YL;
    float distance;
	float delta_FL;
}dart_struct;

typedef struct
{
  	fp32 V;
	fp32 num;//Ŀ��ֵ
	fp32 buffer;
	
}rc_motor_message;

typedef struct
{
	
float task_finish_flag ;

}task_t;

static int phase;

extern rc_motor_message LL;
extern rc_motor_message PL;
extern rc_motor_message YL;
extern rc_motor_message FL;
extern int flag_zero;
extern int flag_zero_all;
extern int flag_first_position;
extern float yaw_first_position;
extern float pitch_first_position;
extern int16_t dart_num;
extern int16_t last_dart_num;
extern float measure_distance;
extern float last_measure_distance;
extern float caled_yaw;
extern Switch_Status_e switch_state;

void rc_to_motor(void);
void game_model(void);
void grab_to_motor(void);
void process_motor_encoder_to_serial(motor_data_t *motor);
float intermediate_data_compensation(int16_t dart_num, float measure_distance, int16_t ditl_state);
void rc_to_task(void);
void average_motor_speed(motor_data_t *motor);
void add_dart_struct_array(dart_struct *p, dart_struct dart, uint32_t num);
float angle_to_pitch(float angle);

int16_t correct_speed(int16_t data1,int16_t data2);

#endif

