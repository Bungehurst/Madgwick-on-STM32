#include "show.h"
extern uint8_t dma_buffer[59];
void Send_Raw_Data(void)
{   
	u8 buf[31];
	u8 sumcheck = 0;
	u8 addcheck = 0;

	buf[0]=0xAA;
	buf[1]=0xFF; 
	buf[2]=0x01; 
	buf[3]=25; 	
	
	//acc
	buf[4]=send_data[3];
	buf[5]=send_data[3]>>8;
	buf[6]=send_data[3]>>16;
	buf[7]=send_data[3]>>24;

	buf[8]=send_data[4];
	buf[9]=send_data[4]>>8;
	buf[10]=send_data[4]>>16;
	buf[11]=send_data[4]>>24;

	buf[12]=send_data[5];
	buf[13]=send_data[5]>>8;
	buf[14]=send_data[5]>>16;
	buf[15]=send_data[5]>>24;
	//gyro
	buf[16]=send_data[0];
	buf[17]=send_data[0]>>8;
	buf[18]=send_data[0]>>16;
	buf[19]=send_data[0]>>24;

	buf[20]=send_data[1];
	buf[21]=send_data[1]>>8;
	buf[22]=send_data[1]>>16;
	buf[23]=send_data[1]>>24;

	buf[24]=send_data[2];
	buf[25]=send_data[2]>>8;
	buf[26]=send_data[2]>>16;
	buf[27]=send_data[2]>>24;
	buf[28]=1; //Shock Status
	for(u8 i=0;i < buf[3]+4;i++)
	{
		sumcheck += buf[i];
		addcheck += sumcheck;
	}
	
	buf[29] = sumcheck;
	buf[30] = addcheck;

	memcpy(dma_buffer,buf,31);
}
void Send_Euler_Data(void){
	u8 buf[13];
	u8 sumcheck = 0;
	u8 addcheck = 0;

	buf[0]=0xAA; 
	buf[1]=0xFF; 
	buf[2]=0x03; 
	buf[3]=7; 	
	
	buf[4]=send_data[6];
	buf[5]=send_data[6]>>8;
	buf[6]=send_data[7];
	buf[7]=send_data[7]>>8;
	buf[8]=send_data[8];
	buf[9]=send_data[8]>>8;
	buf[10]=1; //Fusion Status

	for(u8 i=0;i < buf[3]+4;i++)
	{
		sumcheck += buf[i];
		addcheck += sumcheck;
	}
	
	buf[11] = sumcheck;
	buf[12] = addcheck;

	memcpy(dma_buffer+31,buf,13);

}

void Send_Quaternion_Data(void){
	u8 buf[15];
	u8 sumcheck = 0;
	u8 addcheck = 0;

	buf[0]=0xAA; //֡ͷ
	buf[1]=0xFF; //Ŀ���ַ
	buf[2]=0x04; //������
	buf[3]=9; 	//���ݳ���
	
	buf[4]=send_data[12];
	buf[5]=send_data[12]>>8;
	buf[6]=send_data[9];
	buf[7]=send_data[9]>>8;
	buf[8]=send_data[10];
	buf[9]=send_data[10]>>8;
	buf[10]=send_data[11];
	buf[11]=send_data[11]>>8;
	buf[12]=1; //Fusion Status

	for(u8 i=0;i < buf[3]+4;i++)
	{
		sumcheck += buf[i];
		addcheck += sumcheck;
	}
	
	buf[13] = sumcheck;
	buf[14] = addcheck;

	memcpy(dma_buffer+31+13,buf,15);

}
