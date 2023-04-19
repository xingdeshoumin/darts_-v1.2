#include "debugTask.h"
#include "cmsis_os.h"
#include "STMGood.h"
#include "bsp_usart.h"
#include "bsp_can.h"
#include "chassis_control.h"
#include "motorTask.h"
#include "chassis_behavior.h"
#include "bsp_dubs.h"
#include "oled.h" 
#include "bsp_imu.h"
#include "ins_task.h"
#include "controller.h"


void StartdebugTask(void const * argument)
{
  /* USER CODE BEGIN StartDebugTask */
    static uint32_t count = 0;
	OLED_Init(); 
	OLED_ColorTurn(0);//0正常显示，1 反色显示 
	OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示 
	OLED_Clear();

    INS_Init();

  /* Infinite loop */
  	for(;;)
  	{
		uart_dma_printf(&huart7, "%4.3f, %d, %d, %d, %4.3f, %d, %4.3f, %d, %4.3f, %d, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f\n", 
            fire_l.average_speed, 
            fire_l.out_current,
            fire_l.esc_back_temperature,
            fire_l.esc_back_position / 100,
            YL.V,
            yaw.serial_position,
            YAW_P_PID.PIDout,
            yaw.esc_back_speed,
            YAW_S_PID.PIDout,
            motor.circle_num,
            INS.Yaw,
            INS.Pitch,
            INS.Roll,
            INS.Accel[0],
            INS.Accel[1],
            INS.Accel[2],
            MPU6500.Temperature
		);


        if ((count % 100) == 0)
        {
            // 10HZ
            OLED_Printf(0, 5, 24, "FL: ");
            OLED_Printf(0, 35, 24, "YL: ");

            OLED_DrawSquare(45, 5, 128, 29);
            OLED_DrawSquare(45, 37, 128, 60);

            OLED_Printf(45, 5, 24, "%4d", fire_l.esc_back_speed);
            OLED_Printf(45, 40, 24, "%4d", yaw.serial_position);

            OLED_Refresh(); //每次更改后，需要刷新
        }

        if ((count % 10) == 0)
        {
            // 100HZ
            INS_Task();
        }

        count++;
		osDelay(1);
  	}
  /* USER CODE END StartDebugTask */
}

