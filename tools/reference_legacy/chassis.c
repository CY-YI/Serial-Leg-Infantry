#include "chassis.h"
#include "main.h"
#include "pid.h"
#include "navigation.h"
// 全局变量：速度和偏航角
fp32 vy;
fp32 vx; 
int16_t yaw_angle;

//extern PidTypeDef fallow_angle_pid;  // 6020 角度跟随PID
//extern PidTypeDef fallow_speed_pid;	// 6020 速度跟随PID	

fp32 vy1;
fp32 vx1;

fp32 vy2;
fp32 vx2;

fp32 v1;			// PID处理后的四轮速度
fp32 v2;
fp32 v3;
fp32 v4;

float Klimit = 1;
float Plimit = 0;
double Scaling1, Scaling2, Scaling3, Scaling4;

// 功率控制相关变量
double Chassis_pidout;
float Chassis_pidout_max;

fp32 Power;
fp32 Power_Buffer;

fp32 power_v1;			// 四轮的功率控制
fp32 power_v2;
fp32 power_v3;
fp32 power_v4;

fp32 chassis_set_v1;
fp32 chassis_set_v2;
fp32 chassis_set_v3;
fp32 chassis_set_v4;


extern pid_Motor all_motor_pid;
fp32 A6020_angle;
fp32 A6020_speed;
fp32 A6020_M1;

chassis_mode_e chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; // 默认模式

const RC_ctrl_t *RC; 
const motor_system_t *c;       
const imu_angle_t *imu;
const GPS_MODE *G_mode_chassis;
const GPS_Control_Command *G_control_command_chassic;//导航控制方向指令指针

 ChassisParam_Struct chassisparam ;
  RemoteParam_Struct remoteparam;

void ChassisPidCtr(int16_t LefUpTarspd,int16_t LefDoTarspd,int16_t RigUpTarspd,int16_t RigDoTarspd);
int16_t Ramp_function(int16_t delta_Min,int16_t target,int16_t now, int16_t ramp_Coeff, uint8_t mode);
void chassis_set_control(void);
void ChassisController(void) ;

void chassis_task(void *argument)
{
    c = get_motor_control_point();   // 获取电机控制点
    RC = get_remote_control_point();
    imu = gyroscope_control_point();
    G_mode_chassis = get_gps_mode_data_point();
	  G_control_command_chassic = get_gps_command_data_point();
    while (1)
    {
        // 设置底盘模式和控制
        chassis_set_mode(&chassis_mode);
        chassis_set_control();
        chassis_set_v();
				ChassisController();

//        // 检查遥控器是否按下特定按钮
			
        if (toe_is_error(DBUS_TOE))
        {
            CAN_cmd_chassis(0, 0, 0, 0);  // 如果按下按钮，停止电机
        }
        else
        {
					if(RC->rc.s[1] == 0x02){
           CAN_cmd_chassis(0,
														0,
														0,
														0);  // 向电机发送计算后的速度
					}else{
					 CAN_cmd_chassis(chassisparam.LefUpFinalOutValue,
														chassisparam.RigUpFinalOutValue,
														chassisparam.LefDoFinalOutValue,
														chassisparam.RigDoFinalOutValue);  // 向电机发送计算后的速度
					
					}
        }
        
        osDelay(2);  // 延时，避免过载
    }
}

void chassis_set_mode(chassis_mode_e *chassis_mode)
{
    // 根据遥控器输入切换底盘模式
    if (RC->rc.s[1] == 0x01)
    {
        *chassis_mode = CHASSIS_VECTOR_revolve;
    }
    else if (RC->rc.s[1] == 0x03)
    {
        *chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else
    {
        if ((RC->key.v & KEY_PRESSED_OFFSET_F) && ((*chassis_mode) != CHASSIS_VECTOR_revolve))
        {
            *chassis_mode = CHASSIS_VECTOR_revolve;
        }
        if ((RC->key.v & KEY_PRESSED_OFFSET_R) && ((*chassis_mode) != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW))
        {
            *chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
        }
				if( G_mode_chassis->Chassis_Mode == 0x0A){
					*chassis_mode = CHASSIS_VECTOR_revolve;
				}else 
				{
				  *chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
				}
    }
}

void chassis_set_control(void)
{
	
		int x = 0;                // 初始化 x 方向的速度
    static int x2 = 0;        // x 方向的平滑速度（使用 static 保持值，避免每次调用时重置）
    int y = 0;                // 初始化 y 方向的速度
    static int y2 = 0;        // y 方向的平滑速度（使用 static 保持值，避免每次调用时重置）
    double sin = 0.0;         // 用于存储正弦值（方向上的正弦值）
    double cos = 0.0;         // 用于存储余弦值（方向上的余弦值）
    int pwm1 = 0;             // 电机1的PWM信号（左前）
    int pwm2 = 0;             // 电机2的PWM信号（右前）
    int pwm3 = 0;             // 电机3的PWM信号（左后）
    int pwm4 = 0;             // 电机4的PWM信号（右后）
    int pwm0 = 0;             // 用于计算最终PWM值的变量


    // 根据遥控器输入设置系统速度
   if (chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        A6020_M1 = c->can1_motors.yaw.ecd - A6020_M;
        if (A6020_M1 >= 4046 || A6020_M1 <= -4046)
        {
            if (A6020_M1 <= -4045)
                A6020_M1 = A6020_M1 + 8191;
            if (A6020_M1 >= 4046)
                A6020_M1 = A6020_M1 - 8191;
        }
				

        // 角度和速度的PID控制
        A6020_angle = PID_Calc(&all_motor_pid.pid_follow_angle, A6020_M1, 0);
        A6020_speed = PID_Calc(&all_motor_pid.pid_follow_speed, c->can1_motors.yaw.speed_rpm, A6020_angle);
				
//				if(A6020_speed<=-200)A6020_speed = -200;
				
				
      if(RC->rc.s[1] != 0x02){
			x =  RC->rc.ch[2]-FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_A) * HIGH_RmeoteSpeed 
            + FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_D) * HIGH_RmeoteSpeed ; 
      y = RC->rc.ch[3]+FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_W) * HIGH_RmeoteSpeed 
            - FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_S) * HIGH_RmeoteSpeed ;
			}
//			}else{
//				
//			if(G_control_command_chassic->chassic_command.forward==0x00) y=550;
//			if(G_control_command_chassic->chassic_command.backward==0x00) y=-550;
//			if(G_control_command_chassic->chassic_command.turn_left==0x00) x=-550;
//			if(G_control_command_chassic->chassic_command.turn_right==0x00) x=550;
//				
//				
//				
//			}
    }
		
		
		 // 通过 Ramp_function 对速度进行平滑处理，避免瞬时变化导致底盘不稳定
    // 第一个参数 66 是速度平滑的时间常数，x 和 y 分别是当前目标速度，x2 和 y2 是上次的平滑值
    x2 = Ramp_function(66, x, x2, 6, 0);  // 计算平滑后的 x 方向速度
    y2 = Ramp_function(66, y, y2, 6, 0);  // 计算平滑后的 y 方向速度
 /****************************************/
    // 计算底盘方向上的正弦值和余弦值
    // sin 和 cos 分别表示方向的变化量，决定电机的速度分配
    sin = y2 / sqrt(x2 * x2 + y2 * y2);  // 计算 y 方向的比例
    cos = x2 / sqrt(x2 * x2 + y2 * y2);  // 计算 x 方向的比例		
// 选择较大的速度值来设置最大 PWM，确保电机有足够的输出功率
    if (ABS(y2) > ABS(x2)) {
        pwm0 = ABS(y2);  // 如果 y 方向速度更大，选择 y 方向的绝对值作为最大速度
    } else {
        pwm0 = ABS(x2);  // 如果 x 方向速度更大，选择 x 方向的绝对值作为最大速度
    }		
	
		
		pwm1 = pwm0 * (sin + cos);  // 左前电机
    pwm2 = pwm0 * (sin - cos);  // 右前电机
    pwm3 = pwm0 * (sin - cos);  // 左后电机
    pwm4 = pwm0 * (sin + cos);  // 右后电机

    // 将计算出的 PWM 值映射到遥控器的速度指令中，并进行最大速度限制
    remoteparam.RemoteLefUpSpeed = pwm1 * MaxGivenSpeed / 660;
    remoteparam.RemoteRigUpSpeed = pwm2 * MaxGivenSpeed / 660;
    remoteparam.RemoteLefDoSpeed = pwm3 * MaxGivenSpeed / 660;
    remoteparam.RemoteRigDoSpeed = pwm4 * MaxGivenSpeed / 660;
		



   }

///**
//  * @brief          一阶低通滤波计算
//  * @author         RM
//  * @param[in]      一阶低通滤波结构体
//  * @param[in]      间隔的时间，单位 s
//  * @retval         返回空
//  */
//void first_order_filter_cali(fp32 input2, fp32 input)
//{
//   input2 = input;
//   out =
//       0.1666666667f / (0.1666666667f
//	+ 0.002f) * first_order_filter_type->out
//	+0.002f/ (first_order_filter_type->num[0] 
//	+ 0.002f) * first_order_filter_type->input;
//} 
//	

 	static float last_vx = 0, last_vy = 0;
void chassis_set_v(void)
{
    // 根据模式计算每个轮子的速度

    

    if (chassis_mode == CHASSIS_VECTOR_revolve)
   {
				if(RC->rc.s[1] != 0x02){
	
		vx = 0.8 * last_vx + 0.2 * (RC->rc.ch[2] * 5);
		vy = 0.8 * last_vy + 0.2 * (RC->rc.ch[3] * 5);
		last_vx = vx;
		last_vy = vy;

	}else{                                     
			vy = 0;			vx = 0;
		if(RC->key.v & KEY_PRESSED_OFFSET_W){
				vy = v_max;
			
		}
		if(RC->key.v & KEY_PRESSED_OFFSET_S){
				vy = -v_max;
				
		}
		if(RC->key.v & KEY_PRESSED_OFFSET_A){
			
				vx = -v_max;
		}
		if(RC->key.v & KEY_PRESSED_OFFSET_D){
				
				vx = v_max;
		}
	}	
     
	
			
		//		yaw_angle = -(c->can1_motors.yaw.ecd - A6020_M) / 22.766666;
        // 根据偏航角转换速度
		
	
    //    vy1 = vy * (float)(cos(yaw_angle / 57.3f)) - vx * (float)(sin(yaw_angle / 57.3f));
    //  vx1 = vy * (float)(sin(yaw_angle / 57.3f)) + vx * (float)(cos(yaw_angle / 57.3f));


// 计算最终速度
chassis_set_v1 = vy1 + vx1;
chassis_set_v2 = -vy1 + vx1;
chassis_set_v3 = vy1 - vx1;
chassis_set_v4 = -vy1 - vx1;

// 叠加旋转速度（动态调整）
chassis_set_v1 += v_revolve;
chassis_set_v2 += v_revolve;
chassis_set_v3 += v_revolve;
chassis_set_v4 += v_revolve;	
	
	
      


    }else{
				 chassis_set_v1 =0;
        chassis_set_v2 =0;
        chassis_set_v3 = 0;
        chassis_set_v4 =0;
		
		
		}
}






/**
  * @brief  底盘控制
  * @param  无
  * @retval 无
 **/
void ChassisController(void)
{
	if(remoteparam.RemoteLefUpSpeed>MaxGivenSpeed)  remoteparam.RemoteLefUpSpeed =MaxGivenSpeed;
	if(remoteparam.RemoteRigUpSpeed>MaxGivenSpeed)  remoteparam.RemoteRigUpSpeed =MaxGivenSpeed;
	if(remoteparam.RemoteLefDoSpeed>MaxGivenSpeed)  remoteparam.RemoteLefDoSpeed =MaxGivenSpeed;
	if(remoteparam.RemoteRigDoSpeed>MaxGivenSpeed)  remoteparam.RemoteRigDoSpeed =MaxGivenSpeed;
	
	if(remoteparam.RemoteLefUpSpeed<-MaxGivenSpeed) remoteparam.RemoteLefUpSpeed =-MaxGivenSpeed;
	if(remoteparam.RemoteRigUpSpeed<-MaxGivenSpeed) remoteparam.RemoteRigUpSpeed =-MaxGivenSpeed;
	if(remoteparam.RemoteLefDoSpeed<-MaxGivenSpeed) remoteparam.RemoteLefDoSpeed =-MaxGivenSpeed;
	if(remoteparam.RemoteRigDoSpeed<-MaxGivenSpeed) remoteparam.RemoteRigDoSpeed =-MaxGivenSpeed;
		
	ChassisPidCtr(remoteparam.RemoteLefUpSpeed,remoteparam.RemoteRigUpSpeed,    //单个电机方向改这里
	-remoteparam.RemoteLefDoSpeed,-remoteparam.RemoteRigDoSpeed);//需具体调试方向
}
              
void ChassisPidCtr(int16_t LefUpTarspd,int16_t LefDoTarspd,int16_t RigUpTarspd,int16_t RigDoTarspd)
{
			  get_chassis_power_and_buffer(&Power, &Power_Buffer);
	
					Power_Buffer=50.0f;
			  if (chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
   {
	      chassisparam.LefUpFinalOutValue = PID_Calc(&all_motor_pid.pid_front_right, c->can1_motors.front_left.speed_rpm,LefUpTarspd+ A6020_speed/2 );
        chassisparam.RigUpFinalOutValue = PID_Calc(&all_motor_pid.pid_front_left, c->can1_motors.front_right.speed_rpm, LefDoTarspd+ A6020_speed/2 );  
        chassisparam.LefDoFinalOutValue = PID_Calc(&all_motor_pid.pid_back_left,  c->can1_motors.back_left.speed_rpm,RigUpTarspd+ A6020_speed/2 );  
        chassisparam.RigDoFinalOutValue = PID_Calc(&all_motor_pid.pid_back_right, c->can1_motors.back_right.speed_rpm,RigDoTarspd+ A6020_speed/2 );  
	 }
	  if (chassis_mode == CHASSIS_VECTOR_revolve)
   {
		 	  chassisparam.LefUpFinalOutValue = PID_Calc(&all_motor_pid.pid_front_right, c->can1_motors.front_left.speed_rpm,chassis_set_v1/2);
        chassisparam.RigUpFinalOutValue = PID_Calc(&all_motor_pid.pid_front_left, c->can1_motors.front_right.speed_rpm,chassis_set_v2/2);  
        chassisparam.LefDoFinalOutValue = PID_Calc(&all_motor_pid.pid_back_left,  c->can1_motors.back_left.speed_rpm,  chassis_set_v3/2);  
        chassisparam.RigDoFinalOutValue = PID_Calc(&all_motor_pid.pid_back_right, c->can1_motors.back_right.speed_rpm,chassis_set_v4/2);  

	 
	 
	 }
// 判断电源缓冲功率是否小于 50.0f
if (Power_Buffer < 50.0f) {
    // 计算四个电机控制输出的总绝对值，合并为一个值（表示当前的功率需求）
    chassisparam.MergeFinalOutValue = 
        ABS(chassisparam.LefUpFinalOutValue) +
        ABS(chassisparam.RigUpFinalOutValue) +
        ABS(chassisparam.LefDoFinalOutValue) +
        ABS(chassisparam.RigDoFinalOutValue);

    // 计算功率限制值，`chassisPowerBuffer` 的平方值乘以预设的功率限制参数
    chassisparam.LimtOutValue = 
        Power_Buffer *
        Power_Buffer *
        PowerLimtParam;

    // 调整每个电机的最终控制值，根据功率限制值进行缩放
    chassisparam.LefUpFinalOutValue = 
        chassisparam.LimtOutValue * chassisparam.LefUpFinalOutValue / chassisparam.MergeFinalOutValue;
    chassisparam.RigUpFinalOutValue = 
        chassisparam.LimtOutValue * chassisparam.RigUpFinalOutValue / chassisparam.MergeFinalOutValue;
    chassisparam.LefDoFinalOutValue = 
        chassisparam.LimtOutValue * chassisparam.LefDoFinalOutValue / chassisparam.MergeFinalOutValue;
    chassisparam.RigDoFinalOutValue = 
        chassisparam.LimtOutValue * chassisparam.RigDoFinalOutValue / chassisparam.MergeFinalOutValue;
	}
}

/**
  * @brief  速度缓冲函数
  * @param  1、最小缓冲值2、目标速度3、当下速度4、缓冲增量5、缓冲模式
  * @retval 无
 **/
int16_t Ramp_function(int16_t delta_Min,int16_t target,int16_t now, int16_t ramp_Coeff, uint8_t mode)
{
	int16_t Lenght=target-now;	
	if(ABS(Lenght)<delta_Min)
		now=target;
	else
	{
		if(mode == 0)
		{
			if(target < 0)
			{
				if(ABS(Lenght) > HIGH_RmeoteSpeed)
				{
					return target / 3;
				}
			}
		}
		if(Lenght>0)
			now+=ramp_Coeff;
		else if(Lenght<0)
			now-=ramp_Coeff;
	}
	return now;
}
