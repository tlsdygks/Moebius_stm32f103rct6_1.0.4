#include "motor.h"

/**************************************************************************
���ߣ�ī��˹�Ƽ�
�ҵ��Ա�С�꣺https://moebius.taobao.com/
**************************************************************************/

void MiniBalance_Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 				//Disable the JTAG debug interface
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//Pull-up input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			//Push-pull output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			//50M 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;					//Port configuration
	GPIO_Init(GPIOD, &GPIO_InitStructure);						//Initialize GPIO according to the set parameters
	

	INA1=0;INB1=0;INC1=0;IND1=0;
	INA2=0;INB2=0;INC2=0;IND2=0;
	
	Set_MotorPWMA(0);
	Set_MotorPWMB(0);
	Set_MotorPWMC(0);
	Set_MotorPWMD(0);
}

void MiniBalance_PWM_Init(u16 arr,u16 psc)
{
	MiniBalance_Motor_Init();  //Initialize the IO required for motor control
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
	
	RCC->APB2ENR|=1<<13;       //Enable the TIM8 clock
	RCC->APB2ENR|=1<<4;        //PORTCʱ��ʹ��     
	GPIOC->CRH&=0XFFFFFF00;    //PORTC8�������
	GPIOC->CRH|=0X000000BB;    //PORTC8�������
	
	GPIOC->CRL&=0X00FFFFFF;    //PORTC6 7�������
	GPIOC->CRL|=0XBB000000;    //PORTC6 7�������
	TIM8->ARR=arr;             //�趨�������Զ���װֵ 
	TIM8->PSC=psc;             //Ԥ��Ƶ������Ƶ
	TIM8->CCMR1|=6<<4;         //CH1 PWM1ģʽ	
	TIM8->CCMR1|=6<<12;        //CH2 PWM1ģʽ	
	TIM8->CCMR2|=6<<4;         //CH3 PWM1ģʽ	
	TIM8->CCMR2|=6<<12;        //CH4 PWM1ģʽ	
	
	TIM8->CCMR1|=1<<3;         //CH1Ԥװ��ʹ��	  
	TIM8->CCMR1|=1<<11;        //CH2Ԥװ��ʹ��	 
	TIM8->CCMR2|=1<<3;         //CH3Ԥװ��ʹ��	  
	TIM8->CCMR2|=1<<11;        //CH4Ԥװ��ʹ��	  
//	
	TIM8->CCER|=1<<0;         //CH1���ʹ��	
	TIM8->CCER|=1<<4;         //CH2���ʹ��	   
	TIM8->CCER|=1<<8;         //CH3���ʹ��	 
	TIM8->CCER|=1<<12;        //CH4���ʹ��
	TIM8->CCR1=0;
	TIM8->CCR2=0;
	TIM8->CCR3=0;
	TIM8->CCR4=0;
	TIM8->BDTR |= 1<<15;       //TIM8����Ҫ��仰�������PWM
	TIM8->CR1=0x8000;          //ARPEʹ�� 
	TIM8->CR1|=0x01;          //ʹ�ܶ�ʱ��1 
} 

