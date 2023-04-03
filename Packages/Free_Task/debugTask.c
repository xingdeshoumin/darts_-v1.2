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


void StartdebugTask(void const * argument)
{
  /* USER CODE BEGIN StartDebugTask */
	OLED_Init(); 
	OLED_ColorTurn(0);//0������ʾ��1 ��ɫ��ʾ 
	OLED_DisplayTurn(0);//0������ʾ 1 ��Ļ��ת��ʾ 
	OLED_Clear();

  /* Infinite loop */
  	for(;;)
  	{
		uart_dma_printf(&huart7, "%d, %d, %d, %d, %d, %d, %4.3f, %d, %4.3f, %d, %4.3f, %4.3f, %d\n", 
		fire_l.esc_back_speed, 
		fire_l.out_current,
		-fire_r.esc_back_speed,
		-fire_r.out_current,
		fire_l.esc_back_temperature,
		fire_r.esc_back_temperature,
		YL.num,
		yaw.serial_position,
		YAW_P_PID.PIDout,
		yaw.esc_back_speed,
		YAW_S_PID.PIDout,
		PL.num,
		motor.circle_num
		);

		OLED_Printf(0, 5, 24, "FL: ");
		OLED_Printf(0, 35, 24, "YL: ");

		OLED_DrawSquare(45, 5, 128, 29);
		OLED_DrawSquare(45, 37, 128, 60);

		OLED_Printf(45, 5, 24, "%4d", fire_l.esc_back_speed);
		OLED_Printf(45, 40, 24, "%4d", yaw.serial_position);

		OLED_Refresh(); //ÿ�θ��ĺ���Ҫˢ��
		osDelay(30);
  	}
  /* USER CODE END StartDebugTask */
}

