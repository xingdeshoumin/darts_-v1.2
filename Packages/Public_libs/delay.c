#include "delay.h"

/**
  * @brief          ʹ��ѭ�������ӳ�һ��ʱ��
  * @param[in]      us:us΢��
  * @retval         none
  */
void user_delay_us(uint16_t us)
{
	for(; us>0; us--)
	{
		for(uint8_t i= 50; i>0; i--)
		{
			;
		}
	}
}
/**
  * @brief          ʹ��user_delay_us�����ӳ�һ��ʱ��
  * @param[in]      ms:ms����
  * @retval         none
  */
void user_delay_ms(uint16_t ms)
{
	for(; ms>0; ms--)
	{
		user_delay_us(1000);
	}
}
/**
  * @brief          ʹ��nop�����ӳ�һ��ʱ��
  * @param[in]      us:us΢��
  * @retval         none
  */
//void nop_delay_us(uint16_t us)
//{
//    for(; us > 0; us--)
//    {
//        for(uint8_t i = 10; i > 0; i--)
//       {
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//       }
//    }
//}
/**
  * @brief          ʹ��nop_delay_us�����ӳ�һ��ʱ��
  * @param[in]      ms:ms����
  * @retval         none
  */
//void nop_delay_ms(uint16_t ms)
//{
//    for(; ms > 0; ms--)
//    {
//        nop_delay_us(1000);
//    }
//}
