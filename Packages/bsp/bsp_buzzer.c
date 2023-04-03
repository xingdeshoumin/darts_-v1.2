#include "bsp_buzzer.h"
#include "tim.h"

void Di(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 2 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 2 && count < 62 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);
	else if(count > 62)
	count = 0;

}

void Di_Di(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 2 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 2 && count < 6 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 6 && count < 8 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 8 && count < 68)
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	else if(count > 68)
	count = 0;
}

void Di_Di_Di(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 2 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 2 && count < 6 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 6 && count < 8 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 8 && count < 12)
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 12 && count < 14 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 14 && count < 74 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	else if(count > 74)
	count = 0;
}

void Di_Di_Di_Di(void)
{
	static uint16_t count = 0;
	count ++;
	if(count > 0 && count < 2 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 2 && count < 6 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 6 && count < 8 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 8 && count < 12)
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 12 && count < 14 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400) ;
	if(count > 14 && count < 18 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	if(count > 18 && count < 20 )
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	if(count > 20 && count < 60)
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
	else if(count > 60)
	count = 0;
}

void DoNot_Di(void)
{
	__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
}

void long_Di(void)
{
__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	

}

void open_Di(void)
{
__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
	HAL_Delay(2000);

}

