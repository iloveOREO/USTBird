#ifndef _RC_H
#define	_RC_H

#include "stm32f4xx.h"

void Fly_Ready(float T);
void RC_Duty(float , u16 * );
void Feed_Rc_Dog(u8 ch_mode);
void Mode(void);

extern float CH_filter[];
extern s16 CH[];
extern u8 fly_ready,NS ;
extern u8 height_ctrl_mode ;
extern u16 RX_CH[];

#endif

