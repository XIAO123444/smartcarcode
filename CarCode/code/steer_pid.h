#ifndef STEER_PID_H__
#define STEER_PID_H__

#include "zf_common_headfile.h"
struct steer_pid
{
    float p;
    float i;
    float d;
    float outputmax ;
    float outputmin;
    uint8 target;
};

void PID_init(void);
void S_PID_CAL_init(void);


struct steer_pid* SPID_vget_param(void);
int S_PID_CAL();








void S_PIDadd_p(void);
void S_PIDadd_i(void);
void S_PIDadd_d(void);
void S_PIDadd_outputmax(void);

// 参数减少函数
void S_PIDsub_p(void);
void S_PIDsub_i(void);
void S_PIDsub_d(void);
void S_PIDsub_outputmax(void);

#endif