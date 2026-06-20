
#ifndef GIMBA_H
#define GIMBA_H



#define RC_YAW_MAX      660.0f      // ң����yawͨ������г�(-660~660)
#define YAW_MAX_SPEED   30.0f       // ��̨���ת��(��/s)���ɵ���Խ��ת��Խ�ͣ�
#define CTRL_YAW_PERIOD     0.010      // ��������2ms = 0.002s

#define RC_PITCH_MAX      660.0f      // ң����yawͨ������г�(-660~660)
#define PITCH_MAX_SPEED   30.0f       // ��̨���ת��(��/s)���ɵ���Խ��ת��Խ�ͣ�
#define CTRL_PITCH_PERIOD     0.002      // ��������2ms = 0.002s
extern int test_out;
extern float test_angle_flag;
extern float fake_set;
extern float test_iput_angle;

extern float yaw_flag;
extern float pitch_flag;  // ��̨�ĸ����Ǳ�־
extern int pitch_out2 ;
extern float yaw2_flag ;  // ��̨��ƫ���Ǳ�־
extern float pitch_out1;
 extern float pitch_final_output;
 extern int  jieguo;
extern float pitch_out_lv;
extern int is_autoaim_started ; // ����Ƿ��ѽ�������ģʽ����ʼ��ʱ��0=δ��ʼ��1=�ѿ�ʼ��
extern double autoaim_duration ;

void gimbal_task(void const *pvParameters);
void gimbal_set_control(void);		//��̨�ٶ��趨
void gimbal_set_v(void); 					//��̨�ٶȿ���

//int16_t map_range(int16_t x);
//int16_t wrap_angle(int16_t angle);
#endif
