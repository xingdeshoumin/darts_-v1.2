#include "chassis_behavior.h"
#include "bsp_dubs.h"
#include "chassis_control.h"
#include "bsp_io.h"
#include "stdlib.h"
#include "cmath"
#include "judge.h"
#include "bsp_usart.h"
#include "target_finder.h"
#include "darts_list.h"

#define DEBUG_MODE 0
// #define motor_lence 730 // װ3��+1������ͷ������
// #define motor_lence 810 // װ�ķ������񶯹�����������
// #define motor_lence 940
#define fire_speed 6000.0f

rc_motor_message LL;
rc_motor_message PL;
rc_motor_message YL;
rc_motor_message FL;

TargetInstance *outpost; // ǰ��վ��׼����
TargetInstance *base; // ������׼����

Switch_Status_e switch_state;

float caled_yaw;
float measure_distance; // Ŀ���뵯��ԭ��ľ���
float last_measure_distance;

int flag_zero;
int flag_zero_all; // �������־λ
int flag_first_position;
int FL_V_Init_flag;
int dart_num_Init_flag;
int low_FLV_flag;
int target_change_flag;
int motor_count_Init_flag;
int ll_stop_flag;
float yaw_first_position;
float pitch_first_position;
float flv_offset;
float caled_flv;
int16_t dart_num = 0;
int16_t last_dart_num;
int16_t FLV_count;
int16_t target_find_count;
int64_t motor_lence = 940;
int64_t motor_count;
int16_t ditl_state; // �Զ�ģʽ����״̬��־λ 1 ǰ��վ 2 ����
int16_t gymbal_state;
int16_t s2_state;
int16_t last_s2_state;
int16_t last_ditl_state; // �Զ�ģʽ����״̬��־λ

void game_model(void) // �󲦸����ϼ��ϳ�ģʽ
{
    /**************************** ״̬���� ****************************/
    if ((RC_Ctl.rc.ditl-1024) > 500){ // �������� // ���²���״̬��־λ
        ditl_state = 1;
    }
    else if ((RC_Ctl.rc.ditl-1024) < -500){ // ��������
        ditl_state = 2;
    }
    else{
        ditl_state = 0;
    }

    if(RC_Ctl.rc.s1 == 1&&RC_Ctl.rc.s2 == 1) // �����Ҳ���״̬��־λ
    {
        s2_state = 2;
    }
    else if(RC_Ctl.rc.s1 == 1&&RC_Ctl.rc.s2 == 2)
    {
        s2_state = 1;
    }
    else if (RC_Ctl.rc.s1 == 1&&RC_Ctl.rc.s2 == 3)
    {
        s2_state = 0;
    }

    if (caled_yaw >= outpost->target_data.angle_range_min && caled_yaw <= outpost->target_data.angle_range_max){ // ����yaw����״̬��־λ
        gymbal_state = 1;
    }
    else if (caled_yaw >= base->target_data.angle_range_min && caled_yaw <= base->target_data.angle_range_max){
        gymbal_state = 2;
    }
    else{
        gymbal_state = 0;
    }

    /**************************** ������Ϣ ****************************/
    if (last_s2_state == 1 && s2_state == 0 && (RC_Ctl.rc.ch2-1024) < 500 && (RC_Ctl.rc.ch2-1024) > -500 && (RC_Ctl.rc.ch3-1024) < 500 && (RC_Ctl.rc.ch3-1024) > -500 && (RC_Ctl.rc.ch0-1024) < 500 && (RC_Ctl.rc.ch0-1024) > -500){ // �Ҳ������������л�
        if (ditl_state == 0){
            flv_offset -= 10.0f; // ����OLED�ϲ鿴��ǰFL.V��ת��תĦ����ȡ���ڵ�ǰ����״̬
        }
        else if (ditl_state == 2){
            flv_offset -= 500.0f;
        }
        else if (ditl_state == 1){
            flv_offset -= 100.0f;
        }
    }
    else if (last_s2_state == 2 && s2_state == 0 && (RC_Ctl.rc.ch2-1024) < 500 && (RC_Ctl.rc.ch2-1024) > -500 && (RC_Ctl.rc.ch3-1024) < 500 && (RC_Ctl.rc.ch3-1024) > -500 && (RC_Ctl.rc.ch0-1024) < 500 && (RC_Ctl.rc.ch0-1024) > -500){ // �Ҳ������������л�
        if (ditl_state == 0){
            flv_offset += 10.0f; // ����OLED�ϲ鿴��ǰFL.V��ת��תĦ����ȡ���ڵ�ǰ����״̬
        }
        else if (ditl_state == 2){
            flv_offset += 500.0f;
        }
        else if (ditl_state == 1){
            flv_offset += 100.0f;
        }
    }
    else if ((last_s2_state == 2 || last_s2_state == 1) && s2_state == 0 && (RC_Ctl.rc.ch2-1024) > 500){ // ���������������л�+��ҡ������
        YL.num += 0.05f * REDUCTION_RATIO_WHEEL / 360.0f * 8192.0f;
    }
    else if ((last_s2_state == 2 || last_s2_state == 1) && s2_state == 0 && (RC_Ctl.rc.ch2-1024) < -500){ // ���������������л�+��ҡ������
        YL.num -= 0.05f * REDUCTION_RATIO_WHEEL / 360.0f * 8192.0f;
    }
    else if ((last_s2_state == 2 || last_s2_state == 1) && s2_state == 0 && (RC_Ctl.rc.ch3-1024) > 500){ // ���������������л�+��ҡ������
        ll_stop_flag = ll_stop_flag ? 0 : 1;
    }
    else if ((last_s2_state == 2 || last_s2_state == 1) && s2_state == 0 && (RC_Ctl.rc.ch3-1024) < -500){ // ���������������л�+��ҡ������
        flv_offset = 0.0f;
    }
    else if ((last_s2_state == 2 || last_s2_state == 1) && s2_state == 0 && (RC_Ctl.rc.ch0-1024) > 500){ // ���������������л�+��ҡ������
        caled_flv = intermediate_data_compensation(dart_num, measure_distance, ditl_state);
    }

    FL.V = (fp32)caled_flv;
    FL.V += flv_offset;

    /**************************** target_finder ****************************/ // ��ʱû���� // ��ǰ��վ�����и��ʷ���0�� �޷�ʹ��target_finder

    // if (target_find_count > 10 && ((usart3_updated_flag == 1) || (ditl_state == 1 && outpost->target_data.find_start_flag == 0) || (ditl_state == 2 && base->target_data.find_start_flag == 0))){ // ��Ƶ�ȴ�����Ǳ仯
    //     if (last_ditl_state != ditl_state){ // ��ֹtarget_finder����������ǰ�л������ʵ�yaw
    //         if (ditl_state == 1){
    //             if (outpost->target_data.find_start_flag == 1){
    //                 YL.num = -outpost->active_angle * REDUCTION_RATIO_WHEEL / 360.0f * 8192.0f + GIMBAL_OFFSET;
    //             }
    //             else{
    //                 YL.num=yaw.serial_position*1.0f;
    //             }
    //         }
    //         if (ditl_state == 2){
    //             if (base->target_data.find_start_flag == 1){
    //                 YL.num = -base->active_angle * REDUCTION_RATIO_WHEEL / 360.0f * 8192.0f + GIMBAL_OFFSET;
    //             }
    //             else{
    //                 YL.num=yaw.serial_position*1.0f;
    //             }
    //         }

    //         target_change_flag = 1;
    //     }
    //     else{
    //         target_change_flag = 0;
    //     }

    //     if (ditl_state == 1 && target_change_flag == 0){ // ...yaw�����ƶ�����
    //         TargetFindOut(outpost, caled_yaw, measure_distance);
    //         YL.num = -outpost->active_angle * REDUCTION_RATIO_WHEEL / 360.0f * 8192.0f + GIMBAL_OFFSET;
    //     }
    //     if (ditl_state == 2 && target_change_flag == 0){
    //         TargetFindOut(base, caled_yaw, measure_distance);
    //         YL.num = -base->active_angle * REDUCTION_RATIO_WHEEL / 360.0f * 8192.0f + GIMBAL_OFFSET;
    //     }
    //     target_find_count = 0;
    //     usart3_updated_flag = 0;
    // }
    // else{
    //     if (target_find_count < 200){
    //         target_find_count++;
    //     }
    // }

    /**************************** ����״̬ ****************************/
	if(Judge_GameState.game_progress == 4)//����������
	{
		if (dart_num_Init_flag == 0)
		{
			dart_num = 0;
			dart_num_Init_flag = 1;
		}

		if(Judge_DartClientCmd.dart_launch_opening_status == 0)//բ���ѿ���
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
#if DEBUG_MODE != 1
			if (motor.circle_num<motor_lence && ll_stop_flag == 0) // ...
			{
				LL.num += 660.0f;
			}
#endif
			// FL.V = dart_list[dart_num].delta_FL; // ...�޸�Ϊ��ͬ���ڲ�ͬ�����µĲ��
			flag_zero = 0;
		}
		else if(Judge_DartClientCmd.dart_launch_opening_status == 2)//բ�����ڿ���
		{
            ll_stop_flag = 0;
			flag_zero = 0;
		}
		else if(Judge_DartClientCmd.dart_launch_opening_status == 1)//բ�Źر�
		{
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

            if (fire_l.esc_back_temperature < 39){ // �¶ȿ���
                flag_zero = 0;
            }
            else {
                flag_zero = 1;
            }

			
            // ...Ħ�����¶��ȶ�����Ƿ��ȶ����� // ��ǰ��վ�������¶�Խ��һ����Խ� ���ؾ�����û������Ӱ��
			// LL.num=motor.esc_back_position*1.0f; // ...��
		}

        /**************************** ��������궨 ****************************/
        // ...��Ҫ�ڿ���ʱ�ѻ�������ض�λ�ã� �����Ľ� // �����з�������궨
		// if (motor.circle_num < 319)
		// {
		// 	dart_num = 0;
		// }
		// else if (motor.circle_num < 580)
		// {
		// 	dart_num = 1;
		// }
		// else
		// {
		// 	dart_num = 2;
		// }
		
	}
	else if(Judge_GameState.game_progress != 4) // �����ڱ�����
	{
        // if (last_s2_state == 1 && s2_state == 0){ // �Ҳ������������л� // �����з�������궨
        //     if (dart_num > -1){
        //         dart_num--;
        //     }
        // }
        // else if (last_s2_state == 2 && s2_state == 0){ // �Ҳ������������л�
        //     if (dart_num < 3){
        //         dart_num++;
        //     }
        // }

		flag_zero = 1;
		dart_num_Init_flag = 0;
	}

	led_show(dart_list[dart_num].num);
	led_show_color(dart_list[dart_num].color);

	if (dart_num != last_dart_num) // ���ݵ��Ա���������������� // �����з�������궨
	{
		// YL.num -= dart_list[last_dart_num].delta_YL;
		// YL.num += dart_list[dart_num].delta_YL;
	}

    FL.V = fmaxf(FL.V, 3000.0f);
    FL.V = fminf(FL.V, 7800.0f); // 8000
    
	last_dart_num = dart_num;
    last_ditl_state = ditl_state;
    last_s2_state = s2_state;
}

void rc_to_motor(void) // �󲦸�����, ����ģʽ
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

        FL.V = fmaxf(FL.V, 3000.0f);
		FL.V = fminf(FL.V, 7800.0f);

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
        if ((RC_Ctl.rc.ditl-1024) > 300) // ִ��У׼
		{
            
			motor_lence = motor.circle_num;

		}
        else if ((RC_Ctl.rc.ditl-1024) < -300) // ����У׼
		{
			motor_lence = 940;
		}

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
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET );   //��
	HAL_GPIO_WritePin(GPIOI,GPIO_PIN_9,GPIO_PIN_RESET );    //��
	motor_flag = 1;
	grab_to_motor();
	motor.target_position = LL.num;
	motor_finish(&motor,&motor1);
	if(motor1.task_finish_flag == 1)
	{
		phase = 1;
	
	}
}

void rc_to_task(void) // ·�ں���
{

    u8Arry2float(usart3_rx_buffer, &measure_distance);
    caled_yaw = (-(yaw.filted_position - GIMBAL_OFFSET) / 8192.0f * 360.0f) / REDUCTION_RATIO_WHEEL;

	if(RC_Ctl.rc.s1 == 3) // �󲦸�����
	{
        if(flag_first_position!=2)
		{
            YL.num=yaw.serial_position*1.0f;
        }
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
		flag_zero_all=0;
		flag_first_position=2;
		rc_to_motor();
		//	phase = 0 ;
		//	phase1 = 0;
	}
	else if(RC_Ctl.rc.s1 == 1) // �󲦸�����
	{
		// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
		flag_zero_all=0;
		if(flag_first_position!=0)
		{
            Target_Init_Config_s outpost_config = {
                .target_data = {
                    // .angle_range_max = 8.0f,
                    // .angle_range_min = 5.0f,
                    // .distance_range_max = 17.0f,
                    // .distance_range_min = 15.5f,
                    .angle_range_max = 9.71f,
                    .angle_range_min = 0.0f,
                    .distance_range_max = 17.077f, // ���뷶Χ��ʵ�����ݹҹ��� �����෵��ת�ٻῨ��
                    .distance_range_min = 15.828f,

                    .find_out_layer = 3,
                    .find_out_step = 0.12f, // ��������
                },
                .target_type = OUTPOST,
            };
            Target_Init_Config_s base_config = {
                .target_data = {
                    // .angle_range_max = -6.5f,
                    // .angle_range_min = -8.5f,
                    // .distance_range_max = 26.0f,
                    // .distance_range_min = 23.0f,
                    .angle_range_max = -0.0f,
                    .angle_range_min = -10.61f,
                    .distance_range_max = 26.023f,
                    .distance_range_min = 25.223f,

                    .find_out_layer = 3,
                    .find_out_step = 0.12f,
                },
                .target_type = BASE,
            };

            outpost = TargetInit(&outpost_config);
            base = TargetInit(&base_config);

			YL.num=yaw.serial_position*1.0f;
			LL.num=motor.serial_position*1.0f;
			PL.num=pitch.esc_back_position*1.0f;
			yaw_first_position = YL.num;
            flv_offset = 0.0f;
			flag_first_position=0;
            caled_flv = fire_speed;
		}
		
		game_model();
		//
		//yaw_task();
		//task3();
	}
	else if(RC_Ctl.rc.s1 == 2) // �󲦸�����
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
	
	//���������������ֵ
	if(motor->esc_back_position - motor->esc_back_position_last > 6000) 
		--motor->circle_num;
	else if(motor->esc_back_position - motor->esc_back_position_last < -6000) 
		++motor->circle_num;
	motor->serial_position = motor->esc_back_position + motor->circle_num * 8192.0f;
	
		//����ֵת���ɽǶ�
	    motor->position_angle = motor->serial_position / 8192.0f * 360.0f;
	
	motor->esc_back_position_last =  motor->esc_back_position;	
}

float angle_to_pitch(float angle)
{
	return angle / (43.2 - 34.5) * (722906 + 4860916);
}

float intermediate_data_compensation(int16_t dart_num, float measure_distance, int16_t ditl_state)
{
    float distance_max;
    float distance_min;
    uint8_t count;
    float cloest_distance;
    uint8_t dart_count;
    float delta_distance;
    float delta_rpm;
    float rate;

    dart_count = dart_list[dart_num].color * 3 - 1 + dart_list[dart_num].num;
    
    if (gymbal_state == 1){ // outpost
        cloest_distance = outpost_distance_list[dart_count][distance_list_lence / 2].distance;
        distance_max = outpost_distance_list[dart_count][distance_list_lence-1].distance;
        distance_min = outpost_distance_list[dart_count][0].distance;
    }
    else if (gymbal_state == 2){ // base
        cloest_distance = base_distance_list[dart_count][distance_list_lence / 2].distance;
        distance_max = base_distance_list[dart_count][distance_list_lence-1].distance;
        distance_min = base_distance_list[dart_count][0].distance;
    }

    if ((measure_distance > distance_max) || (measure_distance < distance_min)){ // ...
        return fire_speed;
    }

    count = distance_list_lence / 2;
    while (fabsf(measure_distance - cloest_distance) > 0.2f){ // ѭ��������measure����ĸ�� // ...ֵȡ���ڱ궨ʱ�ľ�����
        if (measure_distance > cloest_distance){
            count += count / 2 + 1;
        }
        else if (measure_distance < cloest_distance){
            count -= count / 2;
        }
        if (gymbal_state == 1){
            cloest_distance = outpost_distance_list[dart_count][count].distance;
        }
        else if (gymbal_state == 2){
            cloest_distance = base_distance_list[dart_count][count].distance;
        }
    }
    if (measure_distance == cloest_distance){
        if (gymbal_state == 1){
            return outpost_distance_list[dart_count][count].delta_FL;
        }
        else if (gymbal_state == 2){
            return base_distance_list[dart_count][count].delta_FL;
        }
    }
    else{
        if (measure_distance > cloest_distance){
            count++;
        }
        if (gymbal_state == 1){ // ���������Լ�������Ħ����ת��
            delta_distance = (outpost_distance_list[dart_count][count].distance - outpost_distance_list[dart_count][count-1].distance);
            delta_rpm = (outpost_distance_list[dart_count][count].delta_FL - outpost_distance_list[dart_count][count-1].delta_FL);
            rate = ((measure_distance - outpost_distance_list[dart_count][count-1].distance) / delta_distance);
            return  rate * delta_rpm + outpost_distance_list[dart_count][count-1].delta_FL;
        }
        else if (gymbal_state == 2){
            delta_distance = (base_distance_list[dart_count][count].distance - base_distance_list[dart_count][count-1].distance);
            delta_rpm = (base_distance_list[dart_count][count].delta_FL - base_distance_list[dart_count][count-1].delta_FL);
            rate = ((measure_distance - base_distance_list[dart_count][count-1].distance) / delta_distance);
            return  rate * delta_rpm + base_distance_list[dart_count][count-1].delta_FL;
            // return 7800.0f;
        }
    }
    
}
