#ifndef __FILTER_H
#define __FILTER_H

#include "parameter.h"

#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

#define LPF_1(A,B,C,D) LPF_1_((A),(B),(C),*(D));

//float Moving_Average(u8 item,u8 width_num,float in);
void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
float Moving_Median(u8 item,u8 width_num,float in);
#define _xyz_f_t xyz_f_t
void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out);

#endif
