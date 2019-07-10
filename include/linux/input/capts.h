/*
 * capts.h
 *
 *  Created on: Nov 19, 2012
 *      Author: demirelv
 */

#ifndef CAPTS_H_
#define CAPTS_H_


enum
{
	None = 0,
	TOUCH_SCREEN,
	TOUCH_KEY
};

struct muti_touch_info
{
	int strength;
	int width;
	int posX;
	int posY;
	int id;
};

struct capts_platform_data {
	uint32_t version;
	int max_x;
	int max_y;
	int max_pressure;
	int max_width;


};



#endif /* CAPTS_H_ */
