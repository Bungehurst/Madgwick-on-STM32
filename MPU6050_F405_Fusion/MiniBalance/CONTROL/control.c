#include "control.h"	
#include "filter.h"	
#include "MPU6050.h"
#include "inv_mpu.h"

uint8_t data_ready_flag_ = 0;
FusionEulerAngles EulerAngle;
extern FusionAhrs fusionAhrs;
extern struct quaternion q_est;
int EXTI15_10_IRQHandler(void) 
{    
	if(INT==0)		
	{  
   		EXTI->PR=1<<15;			//����жϱ�־λ   
		Read_Data_Directly();	//����ԭʼ����
		#ifndef IMU_Calib_Mode
		#ifdef Use_Complementary_Filter
		EulerAngle = Run_AHRS_Without_Mag();
		
		send_data[0] = calibratedGyroscope.axis.x*(float)1e+3; 
        send_data[1] = calibratedGyroscope.axis.y*(float)1e+3;
        send_data[2] = calibratedGyroscope.axis.z*(float)1e+3;

		// send_data[3] = calibratedAccelerometer.axis.x*(float)1e+3;
        // send_data[4] = calibratedAccelerometer.axis.y*(float)1e+3;
        // send_data[5] = calibratedAccelerometer.axis.z*(float)1e+3;

		send_data[3] = fusionAhrs.linearAcceleration.axis.x*G2MPS2*(float)1e+3;
        send_data[4] = fusionAhrs.linearAcceleration.axis.y*G2MPS2*(float)1e+3;
        send_data[5] = fusionAhrs.linearAcceleration.axis.z*G2MPS2*(float)1e+3;

		send_data[6] = -EulerAngle.angle.roll*100.0f;
		send_data[7] = EulerAngle.angle.pitch*100.0f;
		send_data[8] = EulerAngle.angle.yaw*100.0f;

		send_data[9]  = Quaternion_Data.element.x*(float)1e+4;
		send_data[10] = Quaternion_Data.element.y*(float)1e+4;
		send_data[11] = Quaternion_Data.element.z*(float)1e+4;
		send_data[12] = Quaternion_Data.element.w*(float)1e+4;
		#endif
		#ifdef Use_Madgwick_Filter
		send_data[0] = calibratedGyroscope.axis.x*(float)1e+3; 
        send_data[1] = calibratedGyroscope.axis.y*(float)1e+3;
        send_data[2] = calibratedGyroscope.axis.z*(float)1e+3;

		send_data[3] = calibratedAccelerometer.axis.x*(float)1e+3;
        send_data[4] = calibratedAccelerometer.axis.y*(float)1e+3;
        send_data[5] = calibratedAccelerometer.axis.z*(float)1e+3;

		// send_data[3] = fusionAhrs.linearAcceleration.axis.x*G2MPS2*(float)1e+3;
        // send_data[4] = fusionAhrs.linearAcceleration.axis.y*G2MPS2*(float)1e+3;
        // send_data[5] = fusionAhrs.linearAcceleration.axis.z*G2MPS2*(float)1e+3;

		// send_data[6] = -EulerAngle.angle.roll*100.0f;
		// send_data[7] = EulerAngle.angle.pitch*100.0f;
		// send_data[8] = EulerAngle.angle.yaw*100.0f;

		send_data[9]  = q_est.q1*(float)1e+4;
		send_data[10] = q_est.q2*(float)1e+4;
		send_data[11] = q_est.q3*(float)1e+4;
		send_data[12] = q_est.q4*(float)1e+4;	
		#endif 
		Send_Euler_Data();
		Send_Quaternion_Data();
		#endif
		Send_Raw_Data();
		data_ready_flag_ = 1;
		LED = ~LED;			 
	}       	  
	return 0;	  
} 