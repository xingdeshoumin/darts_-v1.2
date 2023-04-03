#ifndef BSP_FPS_H
#define BSP_FPS_H

#include "bsp_fps.h"
#include "stdint.h"
#include "stdbool.h"

typedef struct
{
	uint8_t last_cnt;
	bool    is_err;
}s_err_t;


typedef struct
{
  uint16_t dbus;
  uint16_t chassis[4];
  uint16_t yaw;
	uint16_t pitch;
	uint16_t upleft;
	uint16_t upright;
	uint16_t clleft;
	uint16_t clright;
	uint16_t ore_box;
	uint16_t b_board;
} s_fps_t;



extern s_fps_t   s_fps;

void fps_data(void);
void fps_buzzer(void);
void fps_LED(void);
#endif

