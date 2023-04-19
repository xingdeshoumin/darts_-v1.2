#include "chassis_control.h"
#include "string.h"
#include "chassis_behavior.h"
#include "bsp_dubs.h"
#include "STMGood.h"
#include "math.h"

float F_P=10,F_I=6.0,F_D=0;//Ä¦²ÁÂÖPID P£º6~10
float F_F=0.08; // Ä¦²ÁÂÖÇ°À¡
float YAW_F = 0.1; // yawÇ°À¡


s_pid_absolute_t  MOTOR_S_PID= {0};
s_pid_absolute_t  MOTOR_P_PID = {0};

s_pid_absolute_t  YAW_S_PID= {0};
s_pid_absolute_t  YAW_P_PID = {0};

s_pid_absolute_t  PITCH_S_PID= {0};
s_pid_absolute_t  PITCH_P_PID = {0};

s_pid_absolute_t  FIRE_L_S_PID= {0};

s_pid_absolute_t  FIRE_R_S_PID= {0};

s_pid_absolute_t  DEVIATION_PID= {0};

motor_data_t motor = {0};
motor_data_t yaw = {0};
motor_data_t pitch = {0};
motor_data_t fire_r = {0};
motor_data_t fire_l = {0};

int16_t deviation_out = 0;

void chassis_control_loop_init(void)
{
  	memset(&motor , 0 ,sizeof(motor_data_t));
	memset(&pitch , 0 ,sizeof(motor_data_t));
	memset(&yaw , 0 ,sizeof(motor_data_t));
	memset(&fire_l , 0 ,sizeof(motor_data_t));
	memset(&fire_r , 0 ,sizeof(motor_data_t));
	
	pid_abs_param_init(&MOTOR_S_PID,0,0,0,20000.0f,16000.0f);
	pid_abs_param_init(&MOTOR_P_PID,0,0,0,0.0f,4000.0f);
	
	pid_abs_param_init(&PITCH_S_PID,0,0,0,20000.0f,16000.0f);
	pid_abs_param_init(&PITCH_P_PID,0,0,0,0.0f,4000.0f);
	
	pid_abs_param_init(&YAW_S_PID,0,0,0,20000.0f,28000.0f);
	pid_abs_param_init(&YAW_P_PID,0,0,0,0.0f,400.0f);
	
	pid_abs_param_init(&FIRE_L_S_PID,0,0,0,20000.0f,16000.0f);

	pid_abs_param_init(&FIRE_R_S_PID,0,0,0,20000.0f,16000.0f);

	pid_abs_param_init(&DEVIATION_PID,0,0,0,20000.0f,28000.0f);
}

void chassis_control_loop_reset(void)
{ 
	
	pid_abs_param_init(&MOTOR_S_PID,13,0,0,3000.0f,8000.0f);
	pid_abs_param_init(&MOTOR_P_PID,0.8,0.01,0,0.0f,4000.0f);
	
	pid_abs_param_init(&PITCH_S_PID,9,0.3,0,20000.0f,16000.0f);
	pid_abs_param_init(&PITCH_P_PID,0.5,0.1,0,0.0f,4000.0f);
	
	pid_abs_param_init(&YAW_S_PID,15,0,2,20000.0f,28000.0f);
	pid_abs_param_init(&YAW_P_PID,7,0.1,2.1,0.0f,4000.0f);
	
	pid_abs_param_init(&FIRE_L_S_PID,F_P,F_I,F_D,20000.0f,16000.0f);

	pid_abs_param_init(&FIRE_R_S_PID,F_P,F_I,F_D,20000.0f,16000.0f);
	
	pid_abs_param_init(&DEVIATION_PID,60,0,0,20000.0f,28000.0f);
}


void chassis_motor_control_loop_pid_control(void)
{
	YL.num += YL_error_correction(YL.num);
	YL.num = fmaxf(YL.num, -2083.0f);
	YL.num = fminf(YL.num, 3000.0f);
	// FL.V += FL_error_correction();

	motor.target_position = LL.num;
	pitch.target_position = PL.num;
	yaw.target_position = YL.num;
	fire_l.target_speed = FL.V;
	fire_r.target_speed = -FL.V;
	
	MOTOR_P_PID.NowError = motor.target_position - motor.serial_position;
	PITCH_P_PID.NowError =pitch.target_position - pitch.serial_position;
	YAW_P_PID.NowError = yaw.target_position - yaw.serial_position;

    fire_l.average_speed = (1.0f - SPEED_SMOOTH_COEF) * fire_l.average_speed + SPEED_SMOOTH_COEF * fire_l.esc_back_speed;
    fire_r.average_speed = (1.0f - SPEED_SMOOTH_COEF) * fire_r.average_speed + SPEED_SMOOTH_COEF * fire_r.esc_back_speed;
    
	FIRE_L_S_PID.NowError = fire_l.target_speed - fire_l.average_speed;
	FIRE_R_S_PID.NowError = fire_r.target_speed - fire_r.average_speed;
	
	PID_AbsoluteMode(&MOTOR_P_PID);
	PID_AbsoluteMode(&PITCH_P_PID);
	PID_AbsoluteMode(&YAW_P_PID);
	
	PID_AbsoluteMode(&FIRE_L_S_PID);
	PID_AbsoluteMode(&FIRE_R_S_PID);
	
	MOTOR_S_PID.NowError = MOTOR_P_PID.PIDout - motor.esc_back_speed;
	PITCH_S_PID.NowError = PITCH_P_PID.PIDout - pitch.esc_back_speed;
	YAW_S_PID.NowError = YAW_P_PID.PIDout - yaw.esc_back_speed;
	
	PID_AbsoluteMode(&MOTOR_S_PID);
	PID_AbsoluteMode(&PITCH_S_PID);
	PID_AbsoluteMode(&YAW_S_PID);
	
	motor.out_current = MOTOR_S_PID.PIDout;
	pitch.out_current = PITCH_S_PID.PIDout;
	yaw.out_current = YAW_S_PID.PIDout;
	fire_l.out_current = FIRE_L_S_PID.PIDout + F_F * fire_l.target_speed;
	fire_r.out_current = FIRE_R_S_PID.PIDout - F_F * fire_r.target_speed;
}

float YL_error_correction(float YL)
{
	static float last_YL;
	static int last_YL_direction_flag;
	static int last_YL_Init_flag;
	static int YL_direction_flag;
	float back_num = 0;

	if (last_YL_Init_flag == 0)
	{
		last_YL = YL;
		last_YL_Init_flag = 1;
	}
	
	if ((YL - last_YL) > 0)
	{
		YL_direction_flag = 1;
	}
	else if ((YL - last_YL) < 0)
	{
		YL_direction_flag = 0;
	}
	if (YL_direction_flag > last_YL_direction_flag)
	{
		back_num = 60.0;
	}
	else if (YL_direction_flag < last_YL_direction_flag)
	{
		back_num = -60.0;
	}
	
	last_YL = YL;
	last_YL_direction_flag = YL_direction_flag;

	return back_num;
}

float FL_error_correction(void)
{	float return_num = (40.0f - (fire_l.esc_back_temperature + fire_r.esc_back_temperature) * 0.5f) / 15.0f * 50.0f;
	return_num = fminf(return_num, 50.0f);
	return_num = fmaxf(return_num, 0.0f);
	return return_num;
}
