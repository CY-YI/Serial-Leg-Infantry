#ifndef chassis_H
#define chassis_H
#include "main.h"

#define A6020_M 1300    //赂煤甓ㄒピ喬�鲁卢前时拢卢麓丝潭020卤芦戮酶露詵碌	
#define v_revolve 5000	
#define v_max 4000

/*---------------------閬ユ帶鍙傛暟---------------------*/
#define HIGH_RmeoteSpeed		660
#define NORMAL_RemoteSpeed		200
/*---------------------浜戝彴鍙傛暟---------------------*/
/*---------------------搴曠洏鍙傛暟---------------------*/
#define IS_CHASSIS_PID_DEBUG	0

#define PowerLimtParam			6.0f
#define MaxGivenSpeed			7500

#define ChassisP				6.0f   //3.0  //搴曠洏pid
#define ChassisI				0.1f   //0.08
#define ChassisD				0.0f
#define ChassisILim				4000
#define MaxOutValue				12000  //鏈�澶ц緭鍑洪噺


#define ABS(x) ((x)>0? (x):(-(x)))
#define FlagCheck(z) ((z)>0 ? 1 : 0)


typedef struct
{
	int16_t RemoteLefUpSpeed;    //宸︿笂缁欏畾閫熷害
	int16_t RemoteLefDoSpeed;    //宸︿笅缁欏畾閫熷害
	int16_t RemoteRigUpSpeed;    //鍙充笂缁欏畾閫熷害
	int16_t RemoteRigDoSpeed;    //鍙充笅缁欏畾閫熷害
}RemoteParam_Struct;

typedef struct
{
	
	int16_t LimtOutValue;//闄愬埗鍔熺巼鍚庣殑鎵�鏈夌數鏈鸿緭鍑洪噺鐨勬�诲拰
	float 	MergeFinalOutValue;//鎵�鏈夌數鏈鸿緭鍑洪噺鐨勬�诲拰
	int16_t LefUpFinalOutValue;		//宸︿笂鏈�鍚庤緭鍑哄��
	int16_t LefDoFinalOutValue;		//宸︿笅鏈�鍚庤緭鍑哄��
	int16_t RigUpFinalOutValue;		//鍙充笂鏈�鍚庤緭鍑哄��
	int16_t RigDoFinalOutValue;		//鍙充笅鏈�鍚庤緭鍑哄��
	
}ChassisParam_Struct;

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //碌脳脜脤禄谩赂煤脣忙脭脝脤篓脧脿露脭陆脟露脠

  CHASSIS_VECTOR_revolve,       			//碌脳脜脤脨隆脥脫脗脻


} chassis_mode_e;

void chassis_task(void *argument) ;

void chassis_set_control(void);	//碌脳脜脤脣脵露脠脡猫露篓
void chassis_set_v(void); //碌脳脜脤脣脵露脠驴脴脰脝

void chassis_set_mode(chassis_mode_e *chassis_mode);    //碌脳脜脤脛拢脢陆驴脴脰脝



#endif

