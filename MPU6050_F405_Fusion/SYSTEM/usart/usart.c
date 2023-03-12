#include "sys.h"
#include "usart.h"	
#include "stm32f4xx_dma.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//Flag_Show!=0  ʹ�ô���1   
	USART1->DR = (u8) ch;      	
	return ch;
}
#endif
//////////////////////////////////////////////////////////////////
#define TX_BUF_LEN 100
uint32_t DMA_Tx_Buf[100];
//����1�жϷ������
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	//---------------------���ڹ�������---------------------
	 //DMA_Cmd(DMA1_Channel4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
    //����DMAʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_InitStructure.DMA_Channel             = DMA_Channel_4;               /* ����DMAͨ�� */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(USART1->DR));   /* Ŀ�� */
    DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)DMA_Tx_Buf;             /* Դ */
    DMA_InitStructure.DMA_DIR                 = DMA_DIR_MemoryToPeripheral;    /* ���� */
    DMA_InitStructure.DMA_BufferSize          = TX_BUF_LEN;                    /* ���� */                  
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;    /* �����ַ�Ƿ����� */
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;         /* �ڴ��ַ�Ƿ����� */
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_MemoryDataSize_Byte;      /* Ŀ�����ݴ��� */
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;      /* Դ���ݿ��� */
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;              /* ���δ���ģʽ/ѭ������ģʽ */
    DMA_InitStructure.DMA_Priority            = DMA_Priority_High;             /* DMA���ȼ� */
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;          /* FIFOģʽ/ֱ��ģʽ */
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull; /* FIFO��С */
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;       /* ���δ��� */
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;

    /* 3. ����DMA */
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    /* 4.ʹ��DMA�ж� */
    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

    /* 5.ʹ�ܴ��ڵ�DMA���ͽӿ� */
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    /* 6. ����DMA�ж����ȼ� */
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 7.��ʹ��DMA */                  
    DMA_Cmd(DMA2_Stream7, DISABLE);
   
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
		
	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 

}
void DMA_Send_Data(uint8_t *data,uint16_t size)  
{  
    /* �������� */
    memcpy(DMA_Tx_Buf,data,size);  
    /* ���ô������ݳ��� */  
    DMA_SetCurrDataCounter(DMA2_Stream7,size);  
    /* ��DMA,��ʼ���� */  
    DMA_Cmd(DMA2_Stream7,ENABLE);  
} 
void DMA2_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET)   
    {  
        /* �����־λ */
        DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
        /* �ر�DMA */
        DMA_Cmd(DMA2_Stream7,DISABLE);
        /* �򿪷�������ж�,ȷ�����һ���ֽڷ��ͳɹ� */
        USART_ITConfig(USART1,USART_IT_TC,ENABLE);  
    }  
}


void USART1_IRQHandler(void)                	//����1�жϷ������
{


} 
