#include "bsp_can.h"
#include "main.h"

static CAN_TxHeaderTypeDef  chassis_tx_message;
 uint8_t              chassis_can_send_data[8];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];

static CAN_TxHeaderTypeDef  friction_tx_message;
static uint8_t              friction_can_send_data[8];

motor_system_t motor;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* 在文件顶部加全局变量 */
extern volatile uint32_t gimbal_curr_tick;   /* gimbal.c 里定义的 */
extern volatile uint32_t gimbal_loop_count ;            /* CAN 接收帧计数 */
extern  volatile uint32_t gimbal_last_tick ;
static uint32_t loop_unchanged_count = 0;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->diff = (ptr)->ecd - (ptr)->last_ecd;      \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];         \
        (ptr)->angle = (ptr)->ecd * 360.0f / 8192.0f;         \
    if ((ptr)->angle > 180.0f) (ptr)->angle -= 360.0f;                                  \
    }         \


    void get_motor_ecd(motor_measure_temp *v){
//
		
	v->diff = v->ecd - v->last_ecd;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->total_round++;
		//	v->ecd_raw_rate = v->diff + 8191;
	}
	else if(v->diff>4096)
	{
		v->total_round--;
		//v->ecd_raw_rate = v->diff - 8191;
	}		
	else
	{
		//v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->total_ecd = v->ecd + v->total_round * 8192;

    

}
    
    
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

        
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}

/*can1的中断回调函数*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[10];
  if (hcan->Instance == hcan1.Instance){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

			 if(rx_header.StdId==0x201)
		 {
			get_motor_measure(&motor.can1_motors.front_left, rx_data);
		 }
		  if(rx_header.StdId==0x202)
		 {
			get_motor_measure(&motor.can1_motors.front_right, rx_data);
		 }
		 	if(rx_header.StdId==0x203)
		 {
			get_motor_measure(&motor.can1_motors.back_left, rx_data);
		 }
		  if(rx_header.StdId==0x204)
		 {
			get_motor_measure(&motor.can1_motors.back_right, rx_data);
		 }
		 if(rx_header.StdId==0x205)
		 {
			get_motor_measure(&motor.can1_motors.yaw, rx_data);
			get_motor_ecd(&motor.can1_motors.yaw); 
		 }
		 if(rx_header.StdId==0x208)
		 {
			get_motor_measure(&motor.can1_motors.super_cap, rx_data);
		 }
	 }
 
  	if (hcan->Instance == hcan2.Instance){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
		if(rx_header.StdId==0x206)
		 {
			get_motor_measure(&motor.can2_motors.pitch, rx_data);
		 }
		 if(rx_header.StdId==0x205)
		 {
			get_motor_measure(&motor.can2_motors.friction_left, rx_data);
		 }
		 if(rx_header.StdId==0x206){
			get_motor_measure(&motor.can2_motors.friction_right, rx_data);
		 }
		  if(rx_header.StdId==0x207){
			get_motor_measure(&motor.can2_motors.pop_up_wheel, rx_data);
			get_motor_ecd(&motor.can2_motors.pop_up_wheel);	
		 }
	}
}
//电调C620  0x200为id 1到4 0x1FF为id 4到8
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    //id1
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    //id2
    chassis_can_send_data[2] = motor2 >> 8;    
    chassis_can_send_data[3] = motor2;
    //id3
    chassis_can_send_data[4] = motor3 >> 8;   
    chassis_can_send_data[5] = motor3;
    //id5
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    /* 每 100 次循环检查一次 gimbal_task 心跳 */
    if (gimbal_loop_count % 100 == 0) {
        if ((HAL_GetTick() - gimbal_curr_tick) >= 10) {
            /* gimbal_task 超过 10ms 未更新 */
            loop_unchanged_count++;

            /* 连续 2 次检测都没变化(=200ms task 停跑)→ 强制发零 */
            if (loop_unchanged_count >= 2) {
                /* 直接在这里发零指令,不走 gimbal_task */
                for (int i = 0; i < 8; i++) {
                    chassis_can_send_data[i] = 0;
                }

                HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
                return;
            }
        } else {
            /* task 还在跑,重置计数 */
            gimbal_last_tick = gimbal_curr_tick;
            loop_unchanged_count = 0;
        }
    }

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
//GM6020 0x1FF为id 1到4 0x2FF为id 4到8  0x1FE为流控
void CAN_cmd_yaw(int16_t motor1)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x1FE;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = motor1 >> 8;
    gimbal_can_send_data[1] = motor1;
    
   
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_cmd_pitch(int16_t motor2)
{
   uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x1FE;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[2] = motor2 >> 8;
    gimbal_can_send_data[3] = motor2;
   
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

}




//电调C610,C620,820R  0x200为id 1到4 0x1FF为id 4到8
void CAN_cmd_friction(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    
uint32_t send_mail_box;
    friction_tx_message.StdId = 0x1FF;
    friction_tx_message.IDE = CAN_ID_STD;
    friction_tx_message.RTR = CAN_RTR_DATA;
    friction_tx_message.DLC = 0x08;
    friction_can_send_data[0] = motor1 >> 8;
    friction_can_send_data[1] = motor1;
    friction_can_send_data[2] = motor2 >> 8;
    friction_can_send_data[3] = motor2;
    friction_can_send_data[4] = motor3 >> 8;
    friction_can_send_data[5] = motor3;
    friction_can_send_data[6] = motor4 >> 8;
    friction_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &friction_tx_message, friction_can_send_data, &send_mail_box);

}




const motor_system_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor;
}


