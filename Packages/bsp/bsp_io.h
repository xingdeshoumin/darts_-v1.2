#ifndef BSP_IO_H
#define BSP_IO_H

#include "stm32f4xx.h"

void io_init(void);
void led_show(uint8_t num);
void led_show_color(uint8_t color);

#endif

