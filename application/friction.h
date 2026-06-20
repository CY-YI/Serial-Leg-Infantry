#ifndef friction_H
#define friction_H
#include "main.h"

#define ball_ecd  -49256
typedef enum
{
  Friction_OFF,   //摩擦轮关闭
													
  Friction_ON,    //摩擦轮打开

	shoot_mode,			//射击模式

} friction_mode_e;

void friction_task(void *argument);

void friction_set_mode(friction_mode_e *friction_mode);
void friction_set_control(friction_mode_e *friction_mode);



#endif
