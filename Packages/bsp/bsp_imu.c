/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.c
 * @brief      mpu6500 module driver, configurate MPU6500 and Read the Accelerator
 *             and Gyrometer data using SPI interface      
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#include "bsp_imu.h"
#include "ist8310_reg.h"
#include "mpu6500_reg.h"
#include "bsp_dwt.h"
#include <math.h>

// #define IST8310
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

float MPU6500_ACCEL_SEN = MPU6500_ACCEL_16G_SEN;
float MPU6500_GYRO_SEN = MPU6500_GYRO_2000_SEN;

static uint8_t        tx, rx;
static uint8_t        tx_buff[14] = { 0xff };
uint8_t               mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t               ist_buff[6];                           /* buffer to save IST8310 raw data */
float gyroDiff[3], gNormDiff;

int16_t caliCount = 0;

IMU_Data_t MPU6500;
SPI_HandleTypeDef *MPU6500_SPI;

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(MPU6500_SPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(MPU6500_SPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(MPU6500_SPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(MPU6500_SPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(MPU6500_SPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(MPU6500_SPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
	* @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
	* @retval   
  * @usage  call in ist8310_init() function
	*/
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* turn off slave 1 at first */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    /* turn on slave 1 with one byte transmitting */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    MPU_DELAY(10);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval 
  * @usage  call in ist8310_init() function
	*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
* @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note     
	*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    /* use slave0,auto read data */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    /* every eight mpu6500 internal samples one i2c master read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    /* enable slave 0 and 1 access delay */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    /* enable slave 1 auto transmit */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    MPU_DELAY(6); 
    /* enable slave 0 with data_num bytes reading */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

/**
	* @brief  Initializes the IST8310 device
	* @param  
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t ist8310_init()
{
	  /* enable iic master mode */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
	  /* enable iic 400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
    MPU_DELAY(10);

    /* turn on slave 1 for ist write and slave 4 to ist read */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

		/* soft reset */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
    MPU_DELAY(10);

		/* config as ready mode to access register */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    MPU_DELAY(10);

		/* normal state, no int */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    MPU_DELAY(10);
		
    /* config low noise mode, x,y,z axis 16 time 1 avg */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    MPU_DELAY(10);

    /* Set/Reset pulse duration setup,normal mode */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    MPU_DELAY(10);

    /* turn off slave1 & slave 4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    /* configure and turn on slave 0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}

/**
	* @brief  get the data of IST8310
  * @param  buff: the buffer to save the data of IST8310
	* @retval 
  * @usage  call in mpu_get_data() function
	*/
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6); 
}


/**
 * @description: get the data of imu
 * @param {IMU_Data_t} *mpu6500
 * @return {*}
 */
void mpu_get_data(IMU_Data_t *mpu6500)
{
    static int16_t mpu6500_raw_temp;
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu6500_raw_temp  = mpu_buff[0] << 8 | mpu_buff[1];
    mpu6500->Accel[0] = mpu6500_raw_temp * MPU6500_ACCEL_SEN * mpu6500->AccelScale;
    mpu6500_raw_temp  = mpu_buff[2] << 8 | mpu_buff[3];
    mpu6500->Accel[1] = mpu6500_raw_temp * MPU6500_ACCEL_SEN * mpu6500->AccelScale;
    mpu6500_raw_temp  = mpu_buff[4] << 8 | mpu_buff[5];
    mpu6500->Accel[2] = mpu6500_raw_temp * MPU6500_ACCEL_SEN * mpu6500->AccelScale;

    mpu6500_raw_temp = mpu_buff[8]  << 8 | mpu_buff[9];
    mpu6500->Gyro[0] = mpu6500_raw_temp * MPU6500_GYRO_SEN - mpu6500->GyroOffset[0];
    mpu6500_raw_temp = mpu_buff[10] << 8 | mpu_buff[11];
    mpu6500->Gyro[1] = mpu6500_raw_temp * MPU6500_GYRO_SEN - mpu6500->GyroOffset[1];
    mpu6500_raw_temp = mpu_buff[12] << 8 | mpu_buff[13];
    mpu6500->Gyro[2] = mpu6500_raw_temp * MPU6500_GYRO_SEN - mpu6500->GyroOffset[2];

#ifdef IST8310
    ist8310_get_data(ist_buff);
    memcpy(&mpu_data.mx, ist_buff, 6);
#endif // IST8310

	mpu6500_raw_temp = mpu_buff[6] << 8 | mpu_buff[7];
    if (mpu6500_raw_temp > 1023)
    {
        mpu6500_raw_temp -= 2048;
    }
    mpu6500->Temperature = MPU6500_TEMP_OFFSET + mpu6500_raw_temp / MPU6500_TEMP_FACTOR;

    // /* 2000dps -> rad/s */
    // imu.wx   = mpu_data.gx / 16.384f / 57.3f; 
    // imu.wy   = mpu_data.gy / 16.384f / 57.3f; 
    // imu.wz   = mpu_data.gz / 16.384f / 57.3f;
}


/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,卤250dps;1,卤500dps;2,卤1000dps;3,卤2000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,卤2g;1,卤4g;2,卤8g;3,卤16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}

uint8_t id;
static void Calibrate_MPU_Offset(IMU_Data_t *mpu6500);
/**
 * @description: initialize imu mpu6500 and magnet meter ist3810
 * @param {SPI_HandleTypeDef} *mpu6500_SPI
 * @param {uint8_t} calibrate
 * @return {*}
 */
uint8_t mpu_device_init(SPI_HandleTypeDef *mpu6500_SPI, uint8_t calibrate)
{
	MPU_DELAY(100);
    MPU6500_SPI = mpu6500_SPI;

	id                               = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i                        = 0;
	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
																			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
																			{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
																			{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */ 
																			{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
																			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
																			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */ 
																			{ MPU6500_USER_CTRL, 0x20 },};    /* Enable AUX */ 
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_DELAY(1);
	}

	mpu_set_gyro_fsr(3); 		
	mpu_set_accel_fsr(3);

#ifdef IST8310
    ist8310_init();
#endif // IST8310
    if (calibrate)
        Calibrate_MPU_Offset(&MPU6500);
    else
    {
        MPU6500.GyroOffset[0] = GxOFFSET;
        MPU6500.GyroOffset[1] = GyOFFSET;
        MPU6500.GyroOffset[2] = GzOFFSET;
        MPU6500.gNorm = gNORM;
        MPU6500.AccelScale = 9.81f / MPU6500.gNorm;
        MPU6500.TempWhenCali = 40;
    }
	return 0;
}

// 较准零飘
void Calibrate_MPU_Offset(IMU_Data_t *mpu6500)
{
    static float startTime;
    static uint16_t CaliTimes = 1000; // 需要足够多的数据才能得到有效陀螺仪零偏校准结果
    int16_t mpu6500_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;

    startTime = DWT_GetTimeline_s();
    do
    {
        if (DWT_GetTimeline_s() - startTime > 10)
        {
            // 校准超时
            mpu6500->GyroOffset[0] = GxOFFSET;
            mpu6500->GyroOffset[1] = GyOFFSET;
            mpu6500->GyroOffset[2] = GzOFFSET;
            mpu6500->gNorm = gNORM;
            mpu6500->TempWhenCali = 40;
            break;
        }

        DWT_Delay(0.0005);
        mpu6500->gNorm = 0;
        mpu6500->GyroOffset[0] = 0;
        mpu6500->GyroOffset[1] = 0;
        mpu6500->GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; i++)
        {
            mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
            mpu6500_raw_temp  = mpu_buff[0] << 8 | mpu_buff[1];
            mpu6500->Accel[0] = mpu6500_raw_temp * MPU6500_ACCEL_SEN;
            mpu6500_raw_temp  = mpu_buff[2] << 8 | mpu_buff[3];
            mpu6500->Accel[1] = mpu6500_raw_temp * MPU6500_ACCEL_SEN;
            mpu6500_raw_temp  = mpu_buff[4] << 8 | mpu_buff[5];
            mpu6500->Accel[2] = mpu6500_raw_temp * MPU6500_ACCEL_SEN;
            gNormTemp = sqrtf(mpu6500->Accel[0] * mpu6500->Accel[0] +
                              mpu6500->Accel[1] * mpu6500->Accel[1] +
                              mpu6500->Accel[2] * mpu6500->Accel[2]);
            mpu6500->gNorm += gNormTemp;

            mpu6500_raw_temp = mpu_buff[8]  << 8 | mpu_buff[9];
            mpu6500->Gyro[0] = mpu6500_raw_temp * MPU6500_GYRO_SEN;
            mpu6500->GyroOffset[0] += mpu6500->Gyro[0];
            mpu6500_raw_temp = mpu_buff[10] << 8 | mpu_buff[11];
            mpu6500->Gyro[1] = mpu6500_raw_temp * MPU6500_GYRO_SEN;
            mpu6500->GyroOffset[1] += mpu6500->Gyro[1];
            mpu6500_raw_temp = mpu_buff[12] << 8 | mpu_buff[13];
            mpu6500->Gyro[2] = mpu6500_raw_temp * MPU6500_GYRO_SEN;
            mpu6500->GyroOffset[2] += mpu6500->Gyro[2];

            // 记录数据极差
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyroMax[j] = mpu6500->Gyro[j];
                    gyroMin[j] = mpu6500->Gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (mpu6500->Gyro[j] > gyroMax[j])
                        gyroMax[j] = mpu6500->Gyro[j];
                    if (mpu6500->Gyro[j] < gyroMin[j])
                        gyroMin[j] = mpu6500->Gyro[j];
                }
            }

            // 数据差异过大认为收到外界干扰，需重新校准
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
                break;
            DWT_Delay(0.005);
        }

        // 取平均值得到标定结果
        mpu6500->gNorm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; i++)
            mpu6500->GyroOffset[i] /= (float)CaliTimes;

        // 记录标定时IMU温度
        mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
        mpu6500_raw_temp = mpu_buff[6] << 8 | mpu_buff[7];
        mpu6500->TempWhenCali = MPU6500_TEMP_OFFSET + mpu6500_raw_temp / MPU6500_TEMP_FACTOR;

        caliCount++;
    } while (gNormDiff > 1.0f ||
             fabsf(mpu6500->gNorm - 9.8f) > 1.0f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f ||
             fabsf(mpu6500->GyroOffset[0]) > 0.03f ||
             fabsf(mpu6500->GyroOffset[1]) > 0.03f ||
             fabsf(mpu6500->GyroOffset[2]) > 0.03f);

    // 根据标定结果校准加速度计标度因数
    mpu6500->AccelScale = 9.81f / mpu6500->gNorm;
}
