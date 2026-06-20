#include "stm32f4xx_hal.h"  // 必须先包含HAL库总头文件

#include "filtering.h"
#include "bsp_usart.h"
#include "Serial_test.h"
#include "INS_task.h"
#include "bsp_can.h"
#include "gimbal.h"

#include <string.h>    /* strlen, memcpy */
#include <stdio.h>     /* snprintf  ← 关键 */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;


volatile uint8_t g_usart1_tx_busy = 0;                             // 发送状态标志
volatile uint8_t g_current_tx_idx = 0;                            // 当前DMA正在发送的缓冲区索引（0/1）


uint8_t usart1_dma_tx_buf[2][MAX_TX_LEN];
uint8_t usart1_dma_rx_buf[2][MAX_TX_LEN];

LowPassFilter gy_lp_filter;
integrator_t gy_integrator;

const imu_angle_t *imu_test;
const motor_system_t *motor_test;
uint16_t pack_gyro_data_to_buf(uint8_t *tx_buf);
void usart1_send_gyro_init(void);
 


void Serial_test_int(void)
{
  usart1_txrx_dma_init(usart1_dma_tx_buf[0],usart1_dma_tx_buf[1],usart1_dma_rx_buf[0],usart1_dma_rx_buf[1],MAX_TX_LEN);
//    LowPassFilter_Init(&gy_lp_filter , 1000);
//     DWT_Init();                      
    integrator_init(&gy_integrator);  
  usart1_send_gyro_init();
  
}
    

void usart1_send_gyro_init(void)
{
    if (g_usart1_tx_busy == 0)
    {
        uint16_t len = pack_gyro_data_to_buf(usart1_dma_tx_buf[g_current_tx_idx]);
        g_usart1_tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart1, usart1_dma_tx_buf[g_current_tx_idx], len);
    }
}



uint16_t pack_gyro_data_to_buf(uint8_t *tx_buf)
{
    imu_test = get_imu_angle_point();
    motor_test = get_pitch_gimbal_motor_measure_point();
    
    float a = motor_test->can1_motors.back_left.speed_rpm;
    float b = test_angle_flag;
    float c = motor_test->can1_motors.back_left.angle;
 //积分   float d = integrator_update(&gy_integrator, c);   
    float d = test_iput_angle;
    char *buf = (char *)tx_buf;
    uint16_t len = 0;
    
    /* 发送 4 个数据:pitch, gy原始, gy滤波, gy积分得到的角度 */
    len += float_to_str(buf + len, MAX_TX_LEN - len, a, 3);
    if (len < MAX_TX_LEN) buf[len++] = ',';
    len += float_to_str(buf + len, MAX_TX_LEN - len, b, 3);
    if (len < MAX_TX_LEN) buf[len++] = ',';
    len += float_to_str(buf + len, MAX_TX_LEN - len, c, 3);
    if (len < MAX_TX_LEN) buf[len++] = ',';
    len += float_to_str(buf + len, MAX_TX_LEN - len, d, 3);
    
    if (len + 2 <= MAX_TX_LEN) {
        buf[len++] = '\r';
        buf[len++] = '\n';
    }
    
    return len;
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1 && g_usart1_tx_busy == 1)
    {
        // 1. 计算待更新的缓冲区索引（与当前发送索引相反，避免覆盖）
        uint8_t update_idx = (g_current_tx_idx == 0) ? 1 : 0;

        // 2. 中断内更新「非当前发送」的缓冲区（关键：DMA正在发送g_current_tx_idx，更新update_idx无冲突）
       uint16_t len = pack_gyro_data_to_buf(usart1_dma_tx_buf[update_idx]);
       
        // 3. 切换索引：下一次DMA循环自动发送更新后的缓冲区
       g_current_tx_idx = update_idx;
    	HAL_UART_Transmit_DMA(&huart1, usart1_dma_tx_buf[update_idx], len);	
        
    }
}    
    








    
