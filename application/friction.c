#include "friction.h"
#include "main.h"
#include "pid.h"
#include "navigation.h"
#include "gimbal.h"
// 定义全局变量
fp32 i_1 = 0;               // 电机速度1
fp32 i_2 = 0;               // 电机速度2
fp32 i_3 = 0;               // 电机速度3
fp32 i_1_angle = 0;         // 电机角度1

extern pid_Motor all_motor_pid;  // 所有电机的PID控制器

const RC_ctrl_t *RC_f;           // 遥控器控制指针
const motor_system_t *f;         // 电机系统控制指针
const FrameData *fire;          //导航开火指针
const GPS_MODE *G_mode_fire;   //导航控制模式指针

friction_mode_e friction_mode;   // 定义射击模式
uint16_t shoot_heat_now;         // 当前枪口热量
uint16_t shoot_heat_max;         // 枪口热量最大值

int32_t pop_up_wheel_ecd = 0;    // 弹轮编码器值


// 函数声明
void motor_Stalled(const motor_system_t *f, fp32 *speed);
void friction_task(void *argument);
void friction_set_mode(friction_mode_e *friction_mode);
void friction_set_control(friction_mode_e *friction_mode);

// 主任务函数：摩擦轮任务
void friction_task(void *argument) {
	 
   
    // 获取遥控器和电机控制地址
		fire = get_frame_data_point();
    RC_f = get_remote_control_point();
    f = get_motor_control_point();
    G_mode_fire = get_gps_mode_data_point();
	
    while (1) {

		
        // 设置摩擦轮模式和控制
        friction_set_mode(&friction_mode);
			
        friction_set_control(&friction_mode);
        motor_Stalled(f, &i_1);

        // 如果遥控器出错，停止电机
        if (toe_is_error(DBUS_TOE)) {
            CAN_cmd_gimbal(0, 0, 0);
        } else {
         // 发送电机控制命令
            CAN_cmd_gimbal(i_1, i_2, i_3);
        }
			
        osDelay(2);  // 延迟2毫秒
    }
}

// 设置摩擦轮模式
void friction_set_mode(friction_mode_e *friction_mode) {
    // 根据遥控器状态设置摩擦轮模式
	
    if (RC_f->rc.s[1] == 0x01||RC_f->rc.s[1] == 0x03) {
        if (RC_f->rc.s[0] == 0x02){
					*friction_mode = Friction_OFF;
					
				}
        if (RC_f->rc.s[0] == 0x03) {
				*friction_mode = Friction_ON;
			  }
    } else if(RC_f->rc.s[1] == 0x02 ){
//			
//        if ((RC_f->key.v & KEY_PRESSED_OFFSET_Q) && (*friction_mode != Friction_OFF)) {
//            *friction_mode = Friction_OFF;
//        }
//        if ((RC_f->key.v & KEY_PRESSED_OFFSET_E) && (*friction_mode != Friction_ON)) {
//            *friction_mode = Friction_ON;
//				}
//				else{
			
					if( (fire->fire_cmd == 0x68) || (fire->fire_cmd == 0x45) ) {
					*friction_mode = Friction_ON;
					}
			    else
          {
						*friction_mode = Friction_OFF;
            
					}
    } 

}
// 根据摩擦轮模式设置电机速度
void friction_set_control(friction_mode_e *friction_mode) {
    // 摩擦轮关闭时，设置电机速度为0
    if (*friction_mode == Friction_OFF) {
        i_2 = PID_Calc(&all_motor_pid.pid_friction_right, f->can2_motors.friction_left.speed_rpm, 0);
        i_3 = PID_Calc(&all_motor_pid.pid_friction_left, f->can2_motors.friction_right.speed_rpm, 0);
    }
    // 摩擦轮开启时，设置电机速度为7000或-7000
    if (*friction_mode == Friction_ON) {
        i_2 = PID_Calc(&all_motor_pid.pid_friction_right, f->can2_motors.friction_left.speed_rpm, -7000);
        i_3 = PID_Calc(&all_motor_pid.pid_friction_left, f->can2_motors.friction_right.speed_rpm, 7000);
    }
}
	    
		
			
// 电机堵转检测函数
// 该函数用于检测电机是否堵转，并根据检测结果控制弹轮
	  	uint8_t single_Flag=0;    // 单发标志
  
void motor_Stalled(const motor_system_t *f, fp32 *speed) {
				
		uint8_t join_Flag=0 ;  // 连发标志
    fp32 bullet_num = 0;   // 弹轮步进数

		//判断弹轮是否到位
if   ((pop_up_wheel_ecd - f->can2_motors.pop_up_wheel.ecd_value)> -100) {
    join_Flag = 1;	//开启连发						
}

// 连发逻辑
if (friction_mode == Friction_ON && (RC_f->rc.ch[4] == 660||RC_f->mouse.press_r||fire->fire_cmd==0x68 || fire->fire_cmd==0x45) && join_Flag == 1 ) {
    // 每次连发前先重置基准为当前编码器值（避免残留）
    pop_up_wheel_ecd = f->can2_motors.pop_up_wheel.ecd_value;
    bullet_num = -1;
    join_Flag = 0;
    pop_up_wheel_ecd += bullet_num * ball_ecd;
}

//// 单发逻辑（RC_f->rc.s[1] != 0x02）
//if (RC_f->rc.s[1] != 0x02) {
//    // 新增：第一次进入前，先初始化pop_up_wheel_ecd为当前编码器值（避免默认值/连发残留）
//    static int first_single_init = 1; // 静态变量，仅初始化一次
//    if (first_single_init) {
//        pop_up_wheel_ecd = f->can2_motors.pop_up_wheel.ecd_value;
//        first_single_init = 0;
//    }

//    if (friction_mode == Friction_ON && RC_f->rc.ch[4] >= 7000&& single_Flag == 1) {
//        // 单发模式：基准是当前编码器值，仅叠加1倍ball_ecd（退弹步长）
//        bullet_num = 1;
//        // 关键修改：不再重新赋值pop_up_wheel_ecd为当前值，而是基于已有基准叠加
//        pop_up_wheel_ecd += bullet_num * ball_ecd;
//        single_Flag = 0;
//        if (*speed < 200) {
//                bullet_num = 1;
//            }
//        }
//    }
//    // 单发Flag复位：ch[4]==0时，不仅复位single_Flag，还重置基准为当前值
//    if (RC_f->rc.ch[4] == 0) {
//        single_Flag = 1;
//        
//    }

//if (RC_f->rc.s[1] == 0x02) {	
//				
//	      if (friction_mode == Friction_ON && RC_f->mouse.press_l==1 && single_Flag == 1) {
//				pop_up_wheel_ecd = f->can2_motors.pop_up_wheel.ecd_value;
//            bullet_num = 1;
//            single_Flag = 0;
//            if (*speed < -6000) {
//                bullet_num = -1;
//            }
//            pop_up_wheel_ecd += bullet_num * ball_ecd;
//        }
//        if (RC_f->mouse.press_l==0) {
//            single_Flag = 1;
//        }
//			}

  
    // 计算弹轮角度和速度，并更新速度值
    i_1_angle = (float)PID_Calc(&all_motor_pid.pid_pop_up_wheel_angle,f->can2_motors.pop_up_wheel.ecd_value, pop_up_wheel_ecd);
    *speed= (float)PID_Calc(&all_motor_pid.pid_pop_up_wheel_speed, f->can2_motors.pop_up_wheel.speed_rpm, i_1_angle);
		 
}    
