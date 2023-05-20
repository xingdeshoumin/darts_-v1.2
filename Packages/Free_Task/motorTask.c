////////////////////////////////////////////////////////////////
//2022.6.16����
//�������⼰�Ż�����1.Ħ�����ѽ� 2.Ħ�����¶ȶԷ����ٶȵıջ�û�� 
//3.û�����ݴ洢��Ӧ�ã�flash�ȣ� 4.���ڼ��ȶ��ԣ�Ħ���ֳ����ȣ�
//5.yaw��С�ݴ����������Ԥ����������С��϶ 6.���������ؼ�����
////////////////////////////////////////////////////////////////
// 15962
// 25439

#include "cmsis_os.h"
#include "chassis_control.h"
#include "chassis_behavior.h"
#include "bsp_can.h"
#include "bsp_dubs.h"
#include "bsp_flash.h"


void Startmotor_task(void const * argument)
{
  /* USER CODE BEGIN Startmotor_task */

	chassis_control_loop_init();
  /* Infinite loop */
	
	osDelay(6000);//��can�ж�
	// 6020��ֵbug
	YL.num=yaw.serial_position*1.0f;
	LL.num=motor.serial_position*1.0f;
	PL.num=pitch.esc_back_position*1.0f;

	// uint32_t *message;
	// flash_read(0x080E000,message,2);
	
  	for(;;)
	{
		chassis_control_loop_reset(); //����pid
		rc_to_task();//ң������λ��������
		chassis_motor_control_loop_pid_control();//���㷵�ص���������
		// CAN_cmd_chassis(motor.out_current,pitch.out_current);
		// CAN_cmd_yaw(yaw.out_current,yaw.out_current);

		
		
		if(flag_zero_all==1)
		{
			CANTx_SendCurrent(&hcan1,0x200,0,0,0,0);
			CANTx_SendCurrent(&hcan1,0x1FF,0,0,0,0);

		}
		else if(flag_zero==1)
		{
			// CANTx_SendCurrent(&hcan1,0x200,motor.out_current,0,pitch.out_current,0);
			CANTx_SendCurrent(&hcan1,0x200,motor.out_current,0,0,0);
			CANTx_SendCurrent(&hcan1,0x1FF,0,yaw.out_current,0,0);
            // CANTx_SendCurrent(&hcan1,0x1FF,0,0,0,0);

		}
		else
		{
			// CANTx_SendCurrent(&hcan1,0x200,motor.out_current,fire_l.out_current,pitch.out_current,fire_r.out_current);
			CANTx_SendCurrent(&hcan1,0x200,motor.out_current,fire_l.out_current,0,fire_r.out_current);
			CANTx_SendCurrent(&hcan1,0x1FF,0,yaw.out_current,0,0);
            // CANTx_SendCurrent(&hcan1,0x1FF,0,0,0,0);
		}
		
		
		osDelay(1);
	
  	}
  /* USER CODE END Startmotor_task */
}
