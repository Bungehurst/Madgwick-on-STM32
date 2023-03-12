#include "stm32f4xx.h"
#include "sys.h"  

float Pitch,Roll,Yaw;
int Temp;
uint8_t dma_buffer[59];
extern volatile int32_t send_data[13];
// FusionEulerAngles EulerAngle;
extern uint8_t data_ready_flag_;
int main(void)
{
	delay_init(168);                
	uart_init(2000000);              
	LED_Init();                    
	delay_ms(100);                  
	IIC_Init();                     

  	MPU6050_initialize();           	
  	MPU_Init();      

	MiniBalance_EXTI_Init();       
	while(1)
	{
		DMA_Send_Data(dma_buffer,59);
	}
}
