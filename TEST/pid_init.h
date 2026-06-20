#ifndef __pid_init_H__
#define __pid_init_H__
#include "pid.h"


 extern pid_Motor all_motor_pid;

void pid_init(void);

 void pid_assignment(fp32 arr[3], fp32 p, fp32 i,fp32 d);
extern void pid_all_init(void);

#endif
