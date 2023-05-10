/*
 * @Author: kaqi
 * @Date: 2023-05-08 18:52:15
 * @version: beta
 * @description: 
 */
#include "target_finder.h"

/*��ʼ��,����һ��ʵ��*/
TargetInstance *TargetInit(Target_Init_Config_s *config){
    TargetInstance *instance = (TargetInstance *)malloc(sizeof(TargetInstance));
    memset(instance, 0, sizeof(TargetInstance));

    instance->target_data = config->target_data;
    instance->target_type = config->target_type;
    instance->target_data.find_start_flag = 0;

    return instance;
}

/*���ݲ���Ǻ͵�����������ݵ������,���Ŀ��ת��λ��*/
void TargetFindOut(TargetInstance *target, float offset_angle, float laser_distance)
{
    float angle_find_max;
    float angle_find_min;
    float middle_angle_find;

    angle_find_max = target->target_data.angle_range_max;
    angle_find_min = target->target_data.angle_range_min;
    middle_angle_find = (angle_find_min + angle_find_max) / 2;
    if (target->target_data.find_start_flag == 0){ // ��ʼ��������ķ�Χ��Ե�ƶ�
        if ((offset_angle < angle_find_min-TARGET_DEAD_ANGLE) || \
            (offset_angle >= middle_angle_find) && \
            (offset_angle < angle_find_max-TARGET_DEAD_ANGLE)){ // �����Ե����
            target->active_angle = offset_angle + 0.1f;
            target->target_data.find_start_flag = 0;
        }
        else if ((offset_angle > angle_find_max+TARGET_DEAD_ANGLE) || \
                (offset_angle < middle_angle_find) && \
                (offset_angle > angle_find_min+TARGET_DEAD_ANGLE)){ // ���ұ�Ե����
            target->active_angle = offset_angle - 0.1f;
            target->target_data.find_start_flag = 0;
        }
        else { // ����Ŀ�ĵ�
            if ((offset_angle >= angle_find_min-TARGET_DEAD_ANGLE) && (offset_angle <= angle_find_min+TARGET_DEAD_ANGLE)){
                target->target_data.find_direct = 0; // �ƶ����ұ�Ե,��ʼ������ʱ�����
            }
            else if ((offset_angle >= angle_find_max-TARGET_DEAD_ANGLE) && (offset_angle <= angle_find_max+TARGET_DEAD_ANGLE)){
                target->target_data.find_direct = 1; // �ƶ������Ե,��ʼ����˳ʱ�����
            }
            else {
                while (1)
                    ; // δ֪ģʽ,ֹͣ����,���ָ��Խ��,�ڴ����������
                
            }
            target->target_data.find_start_flag = 1;
            target->target_distance = TARGET_DIS_INITIAL_VALUE; // ������������ֵ
        }
        target->target_state == TARGET_CONVERGING;
    }
    else{ // �������������ڵľ�����Сֵ�Լ����Ӧ��yaw�Ƕ�
        if ((laser_distance >= target->target_data.distance_range_min) && \
            (laser_distance <= target->target_data.distance_range_max) && \
            (laser_distance < target->target_distance)){ // ��ǰ����������config��Χ���ұ���ʷ����ֵС,���¾�����Сֵ�Լ����Ӧ��yaw�Ƕ�
            target->target_distance = laser_distance;
            target->target_angle = offset_angle;
        }
        if (target->target_data.find_direct == 0 && offset_angle < angle_find_max-TARGET_DEAD_ANGLE){ // ��ʱ�����
            
            target->active_angle = offset_angle + target->target_data.find_out_step;
        }
        else if (target->target_data.find_direct == 1 && offset_angle > angle_find_min+TARGET_DEAD_ANGLE){ // ˳ʱ�����

            target->active_angle = offset_angle - target->target_data.find_out_step;
        }
        else { // ����Ŀ�ĵ�
            if (target->target_distance == TARGET_DIS_INITIAL_VALUE){ // δ�ҵ�Ŀ��
                target->target_state == NO_TARGET;
                return;
            }
            else if (target->target_data.find_out_layer <= 0){ // �������
                target->target_state == READY_TO_FIRE;
                return;
            }
            else { // ������һ��ѭ��
                target->target_data.find_out_layer--;
                target->target_data.find_out_step /= 1.5;
                target->target_data.angle_range_max = target->target_angle + (angle_find_max-middle_angle_find)/2;
                target->target_data.angle_range_min = target->target_angle - (middle_angle_find-angle_find_min)/2;
                target->target_data.find_start_flag = 0;
            }
        }
        target->target_state == TARGET_CONVERGING;
    }
}

/*uint8����->floatת������*/
float u8Arry2float(uint8_t *data, float *result)
{
    if (*data == 0x80){
        *result = atof(data+3);
    }
}

    
