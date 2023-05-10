#ifndef _TARGET_FINDER_H_
#define _TARGET_FINDER_H_

#include "struct_typedef.h"
#include "stdlib.h"

#define TARGET_DIS_INITIAL_VALUE 50.0f // ���������������ĳ�ֵ
#define TARGET_DEAD_ANGLE 0.05f // ��������
#define REDUCTION_RATIO_WHEEL 11.0f // ��̨ͬ���ּ��ٱ�
#define GIMBAL_OFFSET 120.0f // ��̨��6020ԭ��ƫ�� ��λ��6020����ֵ��ͬ

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

// Ŀ����Ҽ���������
typedef struct
{
    float distance_range_max;
    float distance_range_min;
    float angle_range_max;  // Ŀ��Ĳ��ҷ�Χ,�溯����������С
    float angle_range_min;

    uint8_t find_out_layer;   // ��׼�ĵ���������Խ�߾���Խ�ߵ������ʱ
    float find_out_step;    // ÿ�ε����ľ���

    uint8_t find_direct;   // ���ҷ��� 0��ʱ��,1˳ʱ��
    uint8_t find_start_flag; // �Ƿ��ڲ��ҵ���
}Target_Data_s;

// ���ڼ���׼ʵ��
typedef struct
{
    float target_distance;  // Ŀ�����
    float target_angle;     // Ŀ��yaw�Ƕ�

    float active_angle; // ��yawִ�л������͵�Ŀ��λ��

    Target_Data_s target_data; // ��׼Ŀ������������
    Target_Type_e target_type; // ��׼Ŀ������
    Target_State_e target_state; // Ŀ����׼״̬
}TargetInstance;

// ���ڼ���׼λ������
typedef struct
{
    Target_Data_s target_data; // ��׼Ŀ������������
    Target_Type_e target_type; // ��׼Ŀ������
}Target_Init_Config_s;


/**
 * @brief ��ʼ��Ŀ�������
 * 
 * @param config Ŀ���������ʼ������
 * @return TargetInstance* Ŀ�������ʵ��ָ��
 */
TargetInstance *TargetInit(Target_Init_Config_s *config);

/**
 * @description: Ŀ������������㷨���к���
 * @param {TargetInstance} target
 * @param {float} offset_angle
 * @param {float} laser_distance
 */
void TargetFindOut(TargetInstance *target, float offset_angle, float laser_distance);


/**
 * @description: uint8����->floatת������
 * @param {uint8_t} *data
 * @param {float} *result
 */
float u8Arry2float(uint8_t *data, float *result);

#endif // !_TARGET_FINDER_H_
