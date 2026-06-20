#include "pid_init.h"
#include "pid.h"
#include "main.h"

pid_Motor all_motor_pid;


void pid_assignment(fp32 arr[3], fp32 p, fp32 i,fp32 d)
{
    
    arr[0]=p;
    arr[1]=i;
	arr[2]=d;
 
}


void pid_all_init(void){
	
			PID_clear(&all_motor_pid.pid_yaw_angle);
			PID_clear(&all_motor_pid.pid_yaw_speed);
			PID_clear(&all_motor_pid.pid_pitchI_angle);
			PID_clear(&all_motor_pid.pid_pitchI_speed);
	        
    
    
    
    
    
    
			pid_assignment(all_motor_pid.pid_front_right.pid_parameter,5,0.2,1);
			pid_assignment(all_motor_pid.pid_front_left.pid_parameter,5,0.2,1);
			pid_assignment(all_motor_pid.pid_back_left.pid_parameter,5,0.2,1);
			pid_assignment(all_motor_pid.pid_back_right.pid_parameter,5,0.2,1);
	

	    pid_assignment(all_motor_pid.pid_pitchI_angle.pid_parameter,2.8,	0 , 0.1) ;
	    pid_assignment(all_motor_pid.pid_pitchI_speed.pid_parameter,95,4.5 ,0 ) ;
	
	
	    pid_assignment(all_motor_pid.pid_yaw_angle.pid_parameter,0.25,  0, 1.3) ;
	    pid_assignment(all_motor_pid.pid_yaw_speed.pid_parameter,80,80, 0) ;
	
    
			pid_assignment(all_motor_pid.pid_follow_angle.pid_parameter,0.1,		0,	0) ;
			pid_assignment(all_motor_pid.pid_follow_speed.pid_parameter,25 ,	3.5, 0) ;
	
			pid_assignment(all_motor_pid.pid_friction_left.pid_parameter,5,		0,	0) ;
			pid_assignment(all_motor_pid.pid_friction_right.pid_parameter,5 ,	0, 0) ;
			
			
			pid_assignment(all_motor_pid.pid_pop_up_wheel_angle.pid_parameter,0.2,		0,	0) ;
			pid_assignment(all_motor_pid.pid_pop_up_wheel_speed.pid_parameter,11,		2,	0) ;
//--------------------------------------------------------test--------------------------------------------------	
      

        pid_assignment(all_motor_pid.pid_test_angle.pid_parameter, 1.15, 0.0, 0.26) ;
	    pid_assignment(all_motor_pid.pid_test_speed.pid_parameter, 30.0, 0.05, 0.03) ;
    
    PID_init(&all_motor_pid.pid_test_angle,
		 PID_POSITION,
		 all_motor_pid.pid_test_angle.pid_parameter,
		 700,
		 50);
         
    PID_init(&all_motor_pid.pid_test_speed,
		 PID_POSITION,
		 all_motor_pid.pid_test_speed.pid_parameter,
		 1000,
		 700);     
//----------------------------------------------------  
    
    //底盘
	PID_init(&all_motor_pid.pid_front_right,
		 PID_POSITION,
		 all_motor_pid.pid_front_right.pid_parameter,
		 10000,
		 100);
		
	PID_init(&all_motor_pid.pid_front_left,
		 PID_POSITION,
		 all_motor_pid.pid_back_left.pid_parameter,
		 10000,
		 100);
	
	PID_init(&all_motor_pid.pid_back_left,
		 PID_POSITION,
		 all_motor_pid.pid_back_left.pid_parameter,
		 10000,
		 100);
		 
	PID_init(&all_motor_pid.pid_back_right,
		 PID_POSITION,
		 all_motor_pid.pid_back_left.pid_parameter,
		 10000,
		 100);
		 
         
         
         
         //云台
         
PID_init(&all_motor_pid.pid_pitchI_angle,
		 PID_DELTA,
		 all_motor_pid.pid_pitchI_angle.pid_parameter,
		 400,
		 0);	
 		 
PID_init(&all_motor_pid.pid_pitchI_speed,
		 PID_POSITION,
		 all_motor_pid.pid_pitchI_speed.pid_parameter,
		 12000,                                               
		 18000);	
		 
PID_init(&all_motor_pid.pid_yaw_angle,
		 PID_POSITION,
		 all_motor_pid.pid_yaw_angle.pid_parameter,
		400,
		 0);	
 		 
PID_init(&all_motor_pid.pid_yaw_speed,
		 PID_POSITION,
		 all_motor_pid.pid_yaw_speed.pid_parameter,
		 12000, 
		 16000);		 



		PID_init(&all_motor_pid.pid_follow_angle,
		 PID_DELTA,
		 all_motor_pid.pid_follow_angle.pid_parameter,
		 1000,
		 2);	
 		 
		PID_init(&all_motor_pid.pid_follow_speed,
		 PID_POSITION,
		 all_motor_pid.pid_follow_speed.pid_parameter,
		 5000, 
		 10);		 
		

//发射机构
		 PID_init(&all_motor_pid.pid_friction_left,
		 PID_POSITION,
		 all_motor_pid.pid_friction_left.pid_parameter,
		 13000, 
		 10);		 
		 
		 PID_init(&all_motor_pid.pid_friction_right,
		 PID_POSITION,
		 all_motor_pid.pid_friction_right.pid_parameter,
		 13000, 
		 10);		 
		 
		 PID_init(&all_motor_pid.pid_pop_up_wheel_angle,
		 PID_DELTA,
		 all_motor_pid.pid_pop_up_wheel_angle.pid_parameter,
		 3000, 
		 10);	

		PID_init(&all_motor_pid.pid_pop_up_wheel_speed,
		 PID_POSITION,
		 all_motor_pid.pid_pop_up_wheel_speed.pid_parameter,
		 7000, 
		 10);		

}

