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
		uart_dma_printf(&huart7, "%4.3f, %d, %d, %d, %4.3f, %d, %4.3f, %d, %4.3f, %d, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %d\n", 
            fire_l.average_speed, 
            fire_l.out_current,
            fire_l.esc_back_temperature,
            fire_l.esc_back_position / 100,
            YL.num,
            yaw.serial_position,
            YAW_P_PID.PIDout,
            yaw.esc_back_speed,
            YAW_S_PID.PIDout,
            motor.circle_num,
            INS.Yaw,
            180.0f + INS.Pitch,
            INS.Roll,
            INS.Accel[0],
            INS.Accel[1],
            INS.Accel[2],
            switch_state
		);


        if ((count % 100) == 0)
        {
            // 10HZ
            OLED_Printf(2, 0, 16, "PT: ");
            OLED_Printf(2, 17, 16, "FT: ");
            OLED_Printf(2, 33, 16, "YL: ");
            OLED_Printf(2, 48, 16, "DS: ");

            OLED_DrawSquare(35, 0, 128, 16);
            OLED_DrawSquare(35, 17, 128, 32);
            OLED_DrawSquare(35, 33, 128, 48);
            OLED_DrawSquare(35, 48, 128, 64);

            OLED_Printf(35, 0, 16, "%.1f", 180.0f + INS.Pitch);
            OLED_Printf(96, 0, 16, "%.0f", tmp_correct);
            OLED_Printf(35, 17, 16, "%.1f", FL.V);
            OLED_Printf(35, 33, 16, "%.2f", caled_yaw);
            OLED_Printf(88, 33, 16, "%4d", fire_l.esc_back_temperature);
            OLED_Printf(35, 48, 16, "%.3f", measure_distance);
            
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

