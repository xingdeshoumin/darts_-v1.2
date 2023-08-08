#include "bsp_dubs.h"
#include "usart.h"
#include "bsp_usart.h"
#include "stdlib.h"

RC_Ctl_t RC_Ctl = {0};


/**
 * @brief Receive data from remote device
 * @param *s_rc-- the remote data
 * @return *Usart1buff --the data Usart1buff address from uart
 * @attention  None
 */
void dubs_data_init(void)
{
	RC_Ctl.rc.ch0 = 1024;
	RC_Ctl.rc.ch1 = 1024;
	RC_Ctl.rc.ch2 = 1024;
	RC_Ctl.rc.ch3 = 1024;
	RC_Ctl.rc.ditl = 1024;
	RC_Ctl.rc.s1 = DOWN;
	RC_Ctl.rc.s2 =MID;
}


/**
 * @brief Receive data from remote device
 * @param *s_rc-- the remote data
 * @return *Usart1buff --the data buff address from uart
 * @attention  None
 */
void get_dbus_data (uint8_t *pData)
{
	static uint16_t cnt_l = 0;
	static int16_t last_mouse_y = 0;
	
	if(pData == NULL)
	{
		
	return;
	}
	else{
		RC_Ctl.rc.ch0 = (usart1_rx_buffer[0]| (usart1_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
		RC_Ctl.rc.ch1 = ((usart1_rx_buffer[1] >> 3) | (usart1_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
		RC_Ctl.rc.ch2 = ((usart1_rx_buffer[2] >> 6) | (usart1_rx_buffer[3] << 2) | (usart1_rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
		RC_Ctl.rc.ch3 = ((usart1_rx_buffer[4] >> 1) | (usart1_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
		RC_Ctl.rc.s1 = ((usart1_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
		RC_Ctl.rc.s2 = ((usart1_rx_buffer[5] >> 4)& 0x0003); //!< Switch right9 / 9
		RC_Ctl.mouse.x = usart1_rx_buffer[6] | (usart1_rx_buffer[7] << 8); //!< Mouse X axis
		RC_Ctl.mouse.y = usart1_rx_buffer[8] | (usart1_rx_buffer[9] << 8); //!< Mouse Y axis
		RC_Ctl.mouse.z = usart1_rx_buffer[10] | (usart1_rx_buffer[11] << 8); //!< Mouse Z axis
		RC_Ctl.mouse.press_l = usart1_rx_buffer[12]; //!< Mouse Left Is Press ?
		RC_Ctl.mouse.press_r = usart1_rx_buffer[13]; //!< Mouse Right Is Press ?
		RC_Ctl.key.v = usart1_rx_buffer[14] | (usart1_rx_buffer[15] << 8); //!< KeyBoard value
		RC_Ctl.rc.ditl = (((int16_t)usart1_rx_buffer[16] | ((int16_t)usart1_rx_buffer[17] << 8)) & 0x07FF);
		RC_Ctl.rc.ch0 = (abs(RC_Ctl.rc.ch0 - 1024) > 3 ? RC_Ctl.rc.ch0 : 1024);
		RC_Ctl.rc.ch1 = (abs(RC_Ctl.rc.ch1 - 1024) > 3 ? RC_Ctl.rc.ch1 : 1024);
		RC_Ctl.rc.ch2 = (abs(RC_Ctl.rc.ch2 - 1024) > 3 ? RC_Ctl.rc.ch2 : 1024);
		RC_Ctl.rc.ch3 = (abs(RC_Ctl.rc.ch3 - 1024) > 3 ? RC_Ctl.rc.ch3 : 1024);
		
		/*******deal with mouse.y*************/
		if(abs(RC_Ctl.mouse.y - last_mouse_y) > 150)
			RC_Ctl.mouse.y = last_mouse_y;
		static uint16_t same_value_cnt = 0;
		if(RC_Ctl.mouse.y == last_mouse_y && RC_Ctl.mouse.y != 0)
			same_value_cnt ++;
		if(same_value_cnt > 50)
		{
			same_value_cnt = 0;
			RC_Ctl.mouse.y = 0;
		}
		last_mouse_y = RC_Ctl.mouse.y;
		/**************deal with mouse.l is really pressed*********/  
		if(RC_Ctl.mouse.press_l == 1)
			cnt_l ++;
		else 
			cnt_l = 0;
		
		if(cnt_l > 6)
			RC_Ctl.mouse.press_l = 1;
		else
			RC_Ctl.mouse.press_l = 0;
	}
}



