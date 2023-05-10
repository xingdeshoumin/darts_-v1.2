#ifndef _TARGET_FINDER_H_
#define _TARGET_FINDER_H_

#include "struct_typedef.h"
#include "stdlib.h"

#define TARGET_DIS_INITIAL_VALUE 50.0f // 自瞄迭代距离变量的初值
#define TARGET_DEAD_ANGLE 0.05f // 查找死区
#define REDUCTION_RATIO_WHEEL 11.0f // 云台同步轮减速比
#define GIMBAL_OFFSET 120.0f // 云台与6020原点偏差 单位与6020返回值相同

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

// 目标查找计算用数据
typedef struct
{
    float distance_range_max;
    float distance_range_min;
    float angle_range_max;  // 目标的查找范围,随函数调用逐步缩小
    float angle_range_min;

    uint8_t find_out_layer;   // 瞄准的迭代层数，越高精度越高但会更耗时
    float find_out_step;    // 每次迭代的距离

    uint8_t find_direct;   // 查找方向 0逆时针,1顺时针
    uint8_t find_start_flag; // 是否处于查找递推
}Target_Data_s;

// 飞镖架瞄准实例
typedef struct
{
    float target_distance;  // 目标距离
    float target_angle;     // 目标yaw角度

    float active_angle; // 给yaw执行机构发送的目标位置

    Target_Data_s target_data; // 瞄准目标运算用数据
    Target_Type_e target_type; // 瞄准目标类型
    Target_State_e target_state; // 目标瞄准状态
}TargetInstance;

// 飞镖架瞄准位置设置
typedef struct
{
    Target_Data_s target_data; // 瞄准目标运算用数据
    Target_Type_e target_type; // 瞄准目标类型
}Target_Init_Config_s;


/**
 * @brief 初始化目标查找器
 * 
 * @param config 目标查找器初始化配置
 * @return TargetInstance* 目标查找器实例指针
 */
TargetInstance *TargetInit(Target_Init_Config_s *config);

/**
 * @description: 目标查找器迭代算法运行函数
 * @param {TargetInstance} target
 * @param {float} offset_angle
 * @param {float} laser_distance
 */
void TargetFindOut(TargetInstance *target, float offset_angle, float laser_distance);


/**
 * @description: uint8数组->float转换函数
 * @param {uint8_t} *data
 * @param {float} *result
 */
float u8Arry2float(uint8_t *data, float *result);

#endif // !_TARGET_FINDER_H_
