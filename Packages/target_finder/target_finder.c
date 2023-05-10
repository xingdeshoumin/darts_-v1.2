/*
 * @Author: kaqi
 * @Date: 2023-05-08 18:52:15
 * @version: beta
 * @description: 
 */
#include "target_finder.h"

/*初始化,返回一个实例*/
TargetInstance *TargetInit(Target_Init_Config_s *config){
    TargetInstance *instance = (TargetInstance *)malloc(sizeof(TargetInstance));
    memset(instance, 0, sizeof(TargetInstance));

    instance->target_data = config->target_data;
    instance->target_type = config->target_type;
    instance->target_data.find_start_flag = 0;

    return instance;
}

/*根据测距仪和电机编码器数据迭代求解,输出目标转动位置*/
void TargetFindOut(TargetInstance *target, float offset_angle, float laser_distance)
{
    float angle_find_max;
    float angle_find_min;
    float middle_angle_find;

    angle_find_max = target->target_data.angle_range_max;
    angle_find_min = target->target_data.angle_range_min;
    middle_angle_find = (angle_find_min + angle_find_max) / 2;
    if (target->target_data.find_start_flag == 0){ // 初始化向最近的范围边缘移动
        if ((offset_angle < angle_find_min-TARGET_DEAD_ANGLE) || \
            (offset_angle >= middle_angle_find) && \
            (offset_angle < angle_find_max-TARGET_DEAD_ANGLE)){ // 向左边缘步进
            target->active_angle = offset_angle + 0.1f;
            target->target_data.find_start_flag = 0;
        }
        else if ((offset_angle > angle_find_max+TARGET_DEAD_ANGLE) || \
                (offset_angle < middle_angle_find) && \
                (offset_angle > angle_find_min+TARGET_DEAD_ANGLE)){ // 向右边缘步进
            target->active_angle = offset_angle - 0.1f;
            target->target_data.find_start_flag = 0;
        }
        else { // 到达目的地
            if ((offset_angle >= angle_find_min-TARGET_DEAD_ANGLE) && (offset_angle <= angle_find_min+TARGET_DEAD_ANGLE)){
                target->target_data.find_direct = 0; // 移动到右边缘,初始化后逆时针遍历
            }
            else if ((offset_angle >= angle_find_max-TARGET_DEAD_ANGLE) && (offset_angle <= angle_find_max+TARGET_DEAD_ANGLE)){
                target->target_data.find_direct = 1; // 移动到左边缘,初始化后顺时针遍历
            }
            else {
                while (1)
                    ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
                
            }
            target->target_data.find_start_flag = 1;
            target->target_distance = TARGET_DIS_INITIAL_VALUE; // 给测距变量赋初值
        }
        target->target_state == TARGET_CONVERGING;
    }
    else{ // 迭代查找区域内的距离最小值以及其对应的yaw角度
        if ((laser_distance >= target->target_data.distance_range_min) && \
            (laser_distance <= target->target_data.distance_range_max) && \
            (laser_distance < target->target_distance)){ // 当前测量距离在config范围内且比历史测量值小,更新距离最小值以及其对应的yaw角度
            target->target_distance = laser_distance;
            target->target_angle = offset_angle;
        }
        if (target->target_data.find_direct == 0 && offset_angle < angle_find_max-TARGET_DEAD_ANGLE){ // 逆时针遍历
            
            target->active_angle = offset_angle + target->target_data.find_out_step;
        }
        else if (target->target_data.find_direct == 1 && offset_angle > angle_find_min+TARGET_DEAD_ANGLE){ // 顺时针遍历

            target->active_angle = offset_angle - target->target_data.find_out_step;
        }
        else { // 到达目的地
            if (target->target_distance == TARGET_DIS_INITIAL_VALUE){ // 未找到目标
                target->target_state == NO_TARGET;
                return;
            }
            else if (target->target_data.find_out_layer <= 0){ // 查找完毕
                target->target_state == READY_TO_FIRE;
                return;
            }
            else { // 进入下一层循环
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

/*uint8数组->float转换函数*/
float u8Arry2float(uint8_t *data, float *result)
{
    if (*data == 0x80){
        *result = atof(data+3);
    }
}

    
