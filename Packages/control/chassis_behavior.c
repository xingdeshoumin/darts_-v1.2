#include "chassis_behavior.h"
#include "bsp_dubs.h"
#include "chassis_control.h"
#include "bsp_io.h"
#include "stdlib.h"
#include "cmath"
#include "judge.h"
#include "bsp_usart.h"
#include "target_finder.h"

#define motor_lence 750
#define fire_speed 8600.0f

#define R_1 {1, 1, -75.0, 7350.0f, 0.0}
#define R_4 {1, 4, -250.0, 7450.0f, 0.0}
#define R_5 {1, 5, 100.0, 7390.0f, 0.0}		// ok -
#define R_7 {1, 7, -550.0, 7400.0f, 0.0}	// 464
#define B_1 {0, 1, -200.0, 7410.0f, 0.0}	// ok	// -180
#define B_2 {0, 2, 0.0, 7380.0f, 0.0}		// ok - 
#define B_4 {0, 4, -300.0, 7550.0f, 0.0}	// -264.0

dart_struct dart_list[3] = {B_1, B_2, R_5};

rc_motor_message LL;
rc_motor_message PL;
rc_motor_message YL;
rc_motor_message FL;

TargetInstance *outpost; // 前哨站瞄准数据
TargetInstance *base; // 基地瞄准数据

float caled_yaw;
float measure_distance; // 目标与弹道原点的距离
float last_measure_distance;

int flag_zero;
int flag_zero_all; // 零电流标志位
int flag_first_position;
int FL_V_Init_flag;
int dart_num_Init_flag;
int low_FLV_flag;
int target_change_flag;
float yaw_first_position;
float pitch_first_position;
int16_t dart_num = 0;
int16_t last_dart_num;
int16_t FLV_count;
int16_t target_find_count;
int16_t ditl_state; // 自动模式拨轮状态标志位
int16_t last_ditl_state; // 自动模式拨轮状态标志位

void game_model(void)
{
	if(Judge_GameState.game_progress == 4)//比赛进行中
	{
		if (dart_num_Init_flag == 0)
		{
			dart_num = 0;
			dart_num_Init_flag = 1;
		}

		if(Judge_DartClientCmd.dart_launch_opening_status == 0)//闸门已开启
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);

			if (motor.circle_num<motor_lence)
			{
				LL.num += 450.0f;
			}

			FL.V = dart_list[dart_num].delta_FL; // ...修改为不同镖在不同距离下的查表

			flag_zero = 0;
		}
		else if(Judge_DartClientCmd.dart_launch_opening_status == 2)//闸门正在开启
		{
			flag_zero = 0;
			FL.V = fire_speed;
		}
		else if(Judge_DartClientCmd.dart_launch_opening_status == 1)//闸门关闭
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
			flag_zero=1;
			FL.V=0;
			LL.num=motor.esc_back_position*1.0f;
		}

        // ...需要在开机时把滑块调到特定位置， 后续改进
		if (motor.circle_num < 319)
		{
			dart_num = 0;
		}
		else if (motor.circle_num < 580)
		{
			dart_num = 1;
		}
		else
		{
			dart_num = 2;
		}
		
	}
	else if(Judge_GameState.game_progress != 4)
	{
		if(RC_Ctl.rc.s1 == 1&&RC_Ctl.rc.s2 == 1)
		{
			dart_num = 0;
		}
		else if(RC_Ctl.rc.s1 == 1&&RC_Ctl.rc.s2 == 2)
		{
			dart_num = 2;
		}
		else if (RC_Ctl.rc.s1 == 1&&RC_Ctl.rc.s2 == 3)
		{
			dart_num = 1;
		}
		flag_zero = 1;
		// LL.num+=0;

		dart_num_Init_flag = 0;
	}

	led_show(dart_list[dart_num].num);
	led_show_color(dart_list[dart_num].color);

	if (dart_num != last_dart_num) // 根据调试表格调用相对坐标数据
	{
		YL.num -= dart_list[last_dart_num].delta_YL;
		YL.num += dart_list[dart_num].delta_YL;
		// PL.num -= angle_to_pitch(dart_list[last_dart_num].delta_angle);
		// PL.num += angle_to_pitch(dart_list[dart_num].delta_angle);
	}

    if (target_find_count > 10 && usart3_updated_flag == 1){ // 分频等待测距仪变化
        if ((RC_Ctl.rc.ditl-1024) > 500){ // 拨轮向下
            ditl_state = 1;
        }
        else if ((RC_Ctl.rc.ditl-1024) < -500){ // 拨轮向上
            ditl_state = 2;
        }
        else{
            ditl_state = 0;
        }

        if (last_ditl_state != ditl_state){ // 防止target_finder卡死
            if (ditl_state == 1){
                if (outpost->target_data.find_start_flag == 1){
                    YL.num = -outpost->active_angle * REDUCTION_RATIO_WHEEL / 360.0f * 8192.0f + GIMBAL_OFFSET;
                }
                else{
                    YL.num=yaw.serial_position*1.0f;
                }
            }
            if (ditl_state == 2){
                if (base->target_data.find_start_flag == 1){
                    YL.num = -base->active_angle * REDUCTION_RATIO_WHEEL / 360.0f * 8192.0f + GIMBAL_OFFSET;
                }
                else{
                    YL.num=yaw.serial_position*1.0f;
                }
            }

            target_change_flag = 1;
        }
        else{
            target_change_flag = 0;
        }
        

        if (ditl_state == 1 && target_change_flag == 0){ // ...yaw缓慢移动待做
            TargetFindOut(outpost, caled_yaw, measure_distance);
            YL.num = -outpost->active_angle * REDUCTION_RATIO_WHEEL / 360.0f * 8192.0f + GIMBAL_OFFSET;
        }
        if (ditl_state == 2 && target_change_flag == 0){
            TargetFindOut(base, caled_yaw, measure_distance);
            YL.num = -base->active_angle * REDUCTION_RATIO_WHEEL / 360.0f * 8192.0f + GIMBAL_OFFSET;
        }
        target_find_count = 0;
        usart3_updated_flag = 0;
    }
    else{
        if (target_find_count < 200){
            target_find_count++;
        }
    }
    
	last_dart_num = dart_num;
    last_ditl_state = ditl_state;
}

void rc_to_motor(void)
{
	PL.num += (RC_Ctl.rc.ch3-1024)*1.0f;
	YL.num += (RC_Ctl.rc.ch2-1024)*0.005f;

	if(RC_Ctl.rc.s1 == 2&&RC_Ctl.rc.s2 == 2)
	{
		flag_zero = 0;
		if (FL_V_Init_flag == 0)
		{
			FL.V = fire_speed;
			FL_V_Init_flag = 1;
		}
		
		// FL.V -= (RC_Ctl.rc.ditl-1024)*0.005f; // RC_Ctl.rc.ditl // RC_Ctl.rc.ch0
        if ((RC_Ctl.rc.ditl-1024) < -100 && (RC_Ctl.rc.ditl-1024) > -500 && FLV_count > 100){
            FL.V += 10.0f;
            FLV_count = 0;
        }
        if ((RC_Ctl.rc.ditl-1024) < -500 && FLV_count > 100){
            FL.V += 100.0f;
            FLV_count = 0;
        }
        if ((RC_Ctl.rc.ditl-1024) > 100 && (RC_Ctl.rc.ditl-1024) < 500 && FLV_count > 100){
            FL.V -= 10.0f;
            FLV_count = 0;
        }
        if ((RC_Ctl.rc.ditl-1024) > 500 && FLV_count > 100){
            FL.V -= 100.0f;
            FLV_count = 0;
        }

        if (FLV_count < 200)
            FLV_count++;
	}
	if(RC_Ctl.rc.s1 == 2&&RC_Ctl.rc.s2 == 1)
	{
		flag_zero = 0;
		if ((RC_Ctl.rc.ditl-1024) < -300)
		{
			low_FLV_flag = 0;
		}
		else if ((RC_Ctl.rc.ditl-1024) > 300)
		{
			low_FLV_flag = 1;
		}

		FL.V = (low_FLV_flag) ? (-450.0f) : (450.0f);
		

		FL_V_Init_flag = 0;
	}
	if(RC_Ctl.rc.s1 == 2&&RC_Ctl.rc.s2 == 3)
	{
		flag_zero = 1;

		FL_V_Init_flag = 0;
	}

	if (motor.circle_num<motor_lence)
	{
		LL.num += (RC_Ctl.rc.ch1-1024)*1.0f;
	}
	else
	{
		LL.num += fminf((RC_Ctl.rc.ch1-1024)*1.0f, 0.0f);
	}
}

int motor_flag;
void grab_to_motor(void)
{
	if(motor_flag == 0)
	{

		LL.num = 0.0f;

	}
	else if(motor_flag == 1)
	{

		LL.num = 770000.0f;

	}

}

void motor_finish(motor_data_t *motor,task_t *task)
{
	if( fabs(motor->target_position - motor->serial_position) < 800)
	{
		task->task_finish_flag = 1;

	}
	else
	{
		task->task_finish_flag = 0;
	}
}

task_t motor1;

void outpost_task(void)
{
	YL.num = 3883.0f;
	//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET );
	//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET );
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET );   //右
	HAL_GPIO_WritePin(GPIOI,GPIO_PIN_9,GPIO_PIN_RESET );    //左
	motor_flag = 1;
	grab_to_motor();
	motor.target_position = LL.num;
	motor_finish(&motor,&motor1);
	if(motor1.task_finish_flag == 1)
	{
		phase = 1;
	
	}
}

void rc_to_task(void)
{

    u8Arry2float(usart3_rx_buffer, &measure_distance);
    caled_yaw = (-(yaw.filted_position - GIMBAL_OFFSET) / 8192.0f * 360.0f) / REDUCTION_RATIO_WHEEL;

	if(RC_Ctl.rc.s1 == 2) // 左拨杆在下
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
		flag_zero_all=0;
		flag_first_position=1;
		rc_to_motor();
		//	phase = 0 ;
		//	phase1 = 0;
	}
	else if(RC_Ctl.rc.s1 == 1) // 左拨杆在上
	{
		// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
		flag_zero_all=0;
		if(flag_first_position==1)
		{
            Target_Init_Config_s outpost_config = {
                .target_data = {
                    // .angle_range_max = 10.5f,
                    // .angle_range_min = 2.5f,
                    // .distance_range_max = 16.5f,
                    // .distance_range_min = 15.5f,
                    .angle_range_max = -3.62f,
                    .angle_range_min = -6.36f,
                    .distance_range_max = 6.5f,
                    .distance_range_min = 5.5f,

                    .find_out_layer = 3,
                    .find_out_step = 0.12f, // 与距离相关
                },
                .target_type = OUTPOST,
            };
            Target_Init_Config_s base_config = {
                .target_data = {
                    // .angle_range_max = 0.0f,
                    // .angle_range_min = -14.3f,
                    // .distance_range_max = 26.0f,
                    // .distance_range_min = 25.0f,
                    .angle_range_max = 2.48f,
                    .angle_range_min = -0.38f,
                    .distance_range_max = 9.0f,
                    .distance_range_min = 8.0f,

                    .find_out_layer = 3,
                    .find_out_step = 0.12f,
                },
                .target_type = BASE,
            };

            outpost = TargetInit(&outpost_config);
            base = TargetInit(&base_config);

			YL.num=yaw.serial_position*1.0f;
			LL.num=motor.esc_back_position*1.0f;
			PL.num=pitch.esc_back_position*1.0f;
			yaw_first_position = YL.num;
			pitch_first_position = PL.num;
			flag_first_position=0;
		}
		
		game_model();
		//
		//yaw_task();
		//task3();
	}
	else if(RC_Ctl.rc.s1 == 3) // 左拨杆在中
	{
		flag_zero_all=1;
		flag_first_position=1;
		//
		//yaw_task();
		//task3();
	}
    last_measure_distance = measure_distance;
}

void process_motor_encoder_to_serial(motor_data_t *motor)
{	
	
	//处理成连续的码盘值
	if(motor->esc_back_position - motor->esc_back_position_last > 6000) 
		--motor->circle_num;
	else if(motor->esc_back_position - motor->esc_back_position_last < -6000) 
		++motor->circle_num;
	motor->serial_position = motor->esc_back_position + motor->circle_num * 8192.0f;
	
		//码盘值转换成角度
	    motor->position_angle = motor->serial_position / 8192.0f * 360.0f;
	
	motor->esc_back_position_last =  motor->esc_back_position;	
}

float angle_to_pitch(float angle)
{
	return angle / (43.2 - 34.5) * (722906 + 4860916);
}
