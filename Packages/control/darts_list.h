#ifndef DARTS_LIST_H
#define DARTS_LIST_H

#include "struct_typedef.h"



typedef struct
{
	uint8_t color;			// 0 blue 1 red
	uint8_t num;			// num
	float delta_YL;
	float delta_FL;
    float distance;
}dart_struct;

#endif // !DARTS_LIST_H