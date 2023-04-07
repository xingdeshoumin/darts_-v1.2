/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_imu.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __MPU_H__
#define __MPU_H__

#include "main.h"
#include "mytype.h"

#define MPU_DELAY(x) HAL_Delay(x)

#define MPU6500_TEMP_FACTOR 333.87f
#define MPU6500_TEMP_OFFSET 21

#define MPU6500_ACCEL_2G_SEN 0.00059816278572954496902371288186285f
#define MPU6500_ACCEL_4G_SEN 0.00119632557145908993804742576373f
#define MPU6500_ACCEL_8G_SEN 0.00239265114291817987609485152745f
#define MPU6500_ACCEL_16G_SEN 0.0047853022858363597521897030549f

#define MPU6500_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define MPU6500_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define MPU6500_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define MPU6500_GYRO_2000_SEN 0.00106526443603169529841533860381f

#define GxOFFSET 0.00247530174f
#define GyOFFSET 0.000393082853f
#define GzOFFSET 0.000393082853f
#define gNORM 9.69293118f

// typedef struct
// {
// 	int16_t ax;
// 	int16_t ay;
// 	int16_t az;

// 	int16_t mx;
// 	int16_t my;
// 	int16_t mz;

// 	int16_t temp;

// 	int16_t gx;
// 	int16_t gy;
// 	int16_t gz;
	
// 	int16_t ax_offset;
// 	int16_t ay_offset;
// 	int16_t az_offset;

// 	int16_t gx_offset;
// 	int16_t gy_offset;
// 	int16_t gz_offset;
// } mpu_data_t;

// typedef struct
// {
// 	int16_t ax;
// 	int16_t ay;
// 	int16_t az;

// 	int16_t mx;
// 	int16_t my;
// 	int16_t mz;

// 	float temp;

// 	float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
// 	float wy;
// 	float wz;

// 	float vx;
// 	float vy;
// 	float vz;

// 	float rol;
// 	float pit;
// 	float yaw;
// } imu_t;

typedef struct
{
    float Accel[3];

    float Gyro[3];

    float TempWhenCali;
    float Temperature;

    float AccelScale;
    float GyroOffset[3];

    float gNorm;
} IMU_Data_t;

// extern mpu_data_t mpu_data;
// extern imu_t      imu;
extern IMU_Data_t MPU6500;

uint8_t mpu_device_init(SPI_HandleTypeDef *mpu6500_SPI, uint8_t calibrate);
void mpu_get_data(IMU_Data_t *mpu6500);

#endif


