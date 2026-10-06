
#ifndef GIMBA_H
#define GIMBA_H



#define RC_YAW_MAX      660.0f      // 遥控器yaw通道最大行程(-660~660)
#define YAW_MAX_SPEED   30.0f       // 云台最大转速(°/s)，可调（越快转得越猛）
#define CTRL_YAW_PERIOD     0.010      // 控制周期2ms = 0.002s

#define RC_PITCH_MAX      660.0f      // 遥控器yaw通道最大行程(-660~660)
#define PITCH_MAX_SPEED   30.0f       // 云台最大转速(°/s)，可调（越快转得越猛）
#define CTRL_PITCH_PERIOD     0.002      // 控制周期2ms = 0.002s
extern int test_out;
extern float test_angle_flag;
extern float fake_set;
extern float test_iput_angle;

extern float yaw_flag;
extern float pitch_flag;  // 云台的俯仰角标志
extern int pitch_out2 ;
extern float yaw2_flag ;  // 云台的偏航角标志
extern float pitch_out1;
 extern float pitch_final_output;
 extern int  jieguo;
extern float pitch_out_lv;
extern int is_autoaim_started ; // 标记是否已进入自瞄模式并开始计时（0=未开始，1=已开始）
extern double autoaim_duration ;

void gimbal_task(void const *pvParameters);
void gimbal_set_control(void);		//云台速度设定
void gimbal_set_v(void); 					//云台速度控制

//int16_t map_range(int16_t x);
//int16_t wrap_angle(int16_t angle);
#endif
