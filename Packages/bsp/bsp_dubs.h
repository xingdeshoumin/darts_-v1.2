#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H



#include "stdint.h"
#define UP 1
#define MID 3
#define DOWN 2

/***************PC Key Definition***************/
#define KEY_W      ((uint16_t)0x0001)  //W=1
#define KEY_S      ((uint16_t)0x0002)  //S=2
#define KEY_A      ((uint16_t)0x0004)  //A=4
#define KEY_D      ((uint16_t)0x0008)  //D=8
#define KEY_SHIFT  ((uint16_t)0x0010)  //SHIFT=16
#define KEY_CTRL   ((uint16_t)0x0020)  //CTRL=32
#define KEY_Q      ((uint16_t)0x0040)  //Q=64
#define KEY_E      ((uint16_t)0x0080)  //E=128
#define KEY_R      ((uint16_t)0x0100)  //R=256
#define KEY_F      ((uint16_t)0x0200)  //F=512
#define KEY_G      ((uint16_t)0x0400)  //G=1024
#define KEY_Z      ((uint16_t)0x0800)  //Z=2048
#define KEY_X      ((uint16_t)0x1000)  //X=4096
#define KEY_C      ((uint16_t)0x2000)  //C=8192
#define KEY_V      ((uint16_t)0x4000)  //V=16384
#define KEY_B      ((uint16_t)0x8000)  //B=32768
/***********************************************/

typedef struct
{
struct
{
uint16_t ch0;
uint16_t ch1;
uint16_t ch2;
uint16_t ch3;
uint8_t s1;
uint8_t s2;
int16_t ditl;
}rc;
struct
{
int16_t x;
int16_t y;
int16_t z;
uint8_t press_l;
uint8_t press_r;
}mouse;
struct
{
uint16_t v;
}key;
}RC_Ctl_t;





extern RC_Ctl_t RC_Ctl;
void get_dbus_data (uint8_t *pData);
void dubs_data_init(void);

#endif
