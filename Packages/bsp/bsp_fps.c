#include "bsp_fps.h"
#include "bsp_buzzer.h"
#include "gpio.h"

s_fps_t   s_fps;

  s_err_t dbus;
	s_err_t chassis[4];
	s_err_t fps_upleft;
	s_err_t fps_upright;
	s_err_t fps_clleft;
	s_err_t fps_clright;
	s_err_t fps_ore_box;
	s_err_t b_board;

void calculate_fps_per_second(uint16_t delay_value, s_fps_t *fps)
{
	
	fps->chassis[0] *= (1000/delay_value);
	fps->chassis[1] *= (1000/delay_value);
	fps->chassis[2] *= (1000/delay_value);
	fps->chassis[3] *= (1000/delay_value);
	fps->dbus       *= (1000/delay_value);
	fps->clleft     *= (1000/delay_value);
	fps->pitch      *= (1000/delay_value);
	fps->yaw        *= (1000/delay_value);
	fps->b_board    *= (1000/delay_value);
}

void fps_data_judge(int fps,s_err_t *err)
{
int error;
error = fps - err->last_cnt;
	if(error >= 1)
	err->is_err = 0;
	else
	err->is_err = 1;
	err->last_cnt = fps ;
}

void fps_data(void)
{
	
fps_data_judge(s_fps.chassis[0],&chassis[0]);
fps_data_judge(s_fps.chassis[1],&chassis[1]);
fps_data_judge(s_fps.chassis[2],&chassis[2]);
fps_data_judge(s_fps.chassis[3],&chassis[3]);
fps_data_judge(s_fps.dbus,&dbus);
fps_data_judge(s_fps.upleft,&fps_upleft);
fps_data_judge(s_fps.upright,&fps_upright);
fps_data_judge(s_fps.clleft,&fps_clleft);
fps_data_judge(s_fps.clright,&fps_clright);
fps_data_judge(s_fps.ore_box,&fps_ore_box);
fps_data_judge(s_fps.b_board,&b_board);

}

void fps_buzzer(void)
{
if(dbus.is_err == 1)

	Di();

else if(b_board.is_err == 1)
  Di_Di();
else if(chassis[0].is_err == 1||chassis[1].is_err == 1||chassis[2].is_err == 1||chassis[3].is_err == 1||fps_ore_box.is_err == 1)
Di_Di_Di_Di();
else if(fps_upleft.is_err == 1||fps_upright.is_err == 1||fps_clleft.is_err == 1||fps_clright.is_err == 1)
Di_Di_Di();
}

void fps_LED(void)
{
if(dbus.is_err == 1
	  ||b_board.is_err == 1
     ||chassis[0].is_err == 1
     ||chassis[1].is_err == 1
      ||chassis[2].is_err == 1
        ||chassis[3].is_err == 1
       ||fps_upleft.is_err == 1
        ||fps_upright.is_err == 1
       ||fps_clleft.is_err == 1
       ||fps_clright.is_err == 1
       ||fps_ore_box.is_err == 1)
HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
else
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);

if(chassis[0].is_err == 1)
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
	
	if(chassis[1].is_err == 1)
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
	
	if(chassis[2].is_err == 1)
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
	
	if(chassis[3].is_err == 1)
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
	
	if(fps_upleft.is_err == 1)
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_RESET);
	
	if(fps_upright.is_err == 1)
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);


	if(fps_clleft.is_err == 1)
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
	
	if(fps_clright.is_err == 1)
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_RESET);
	
	if(fps_ore_box.is_err == 1)
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);
	else
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
}

