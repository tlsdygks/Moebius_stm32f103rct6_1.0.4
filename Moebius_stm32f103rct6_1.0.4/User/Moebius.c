#include "sys.h"
/**************************************************************************
Author: Moebius Technology
My Taobao Shop: https://moebius.taobao.com/
**************************************************************************/ 
u8 Flag_Left, Flag_Right, Flag_Direction = 0;		//Bluetooth remote control related variables
u8 Flag_Stop = 1, Flag_Show = 0;					//ֹͣStop flag and display flag, default stop, display open
int Encoder_A, Encoder_B, Encoder_C, Encoder_D;		//Encoder's pulse counting
long int Position_A, Position_B, Position_C, Position_D, Rate_A, Rate_B, Rate_C, Rate_D; //Variables related to PID control
int Encoder_A_EXTI;									//ͨEncoder data read through external interrupt                       
long int Motor_A, Motor_B, Motor_C, Motor_D;		//Motor PWM variable
long int Target_A = 6, Target_B = 6, Target_C = 6, Target_D = 6;	//Motor target values
int Voltage;										//Variables related to battery voltage sampling
float Show_Data_Mb;									//Global display variable, for showing data to be viewed             
u8 delay_50, delay_flag;							//Variables related to delay
u8 Run_Flag = 0;  									// Variables related to Bluetooth remote control and running state flag
u8 rxbuf[8], Urxbuf[8], CAN_ON_Flag = 0, Usart_ON_Flag = 0, PS2_ON_Flag = 0, Usart_Flag, PID_Send, Flash_Send;  // Variables related to CAN and serial port control
u8 txbuf[8], txbuf2[8], Turn_Flag;					//Variables related to CAN transmission
float Pitch, Roll, Yaw, Move_X, Move_Y, Move_Z;		//Three-axis angles and XYZ axis target speeds
u16 PID_Parameter[10], Flash_Parameter[10];			//Arrays related to Flash
float	Position_KP = 6, Position_KI = 0, Position_KD = 3;	// Position control PID parameters
float Velocity_KP = 10, Velocity_KI = 10;			//Velocity control PID parameters
int RC_Velocity = 30, RC_Position = 1000;			//Set remote control velocity and position values
int PS2_LX, PS2_LY, PS2_RX, PS2_RY, PS2_KEY;
int Gryo_Z;
/*************************************************
Function: Peripheral_Init()
Description: Init system peripheral
Input:  void
Output: void
Return: void
*************************************************/
void Peripheral_Init()
{
	LED_Init();						//=====Initialize the hardware interface connected to the LED
	KEY_Init();						//=====Button initialization

	USART1_Init(9600);				//=====���ڳ�ʼ��
	USART3_Init(9600);				//=====��������
	
	MiniBalance_PWM_Init(7199,0);	//=====�������

	Adc_Init();
	
	PS2_Init();

#if MPU6xxx_ENABLE
	IIC_Init();
	MPU6050_initialize();           //=====MPU6050��ʼ��	
	DMP_Init();                     //=====��ʼ��DMP   
#endif
	
#if OLED_DISPLAY_ENABLE
	OLED_Init();					//=====OLED��ʼ��
#endif
	
#if ENCODER_ENABLE					//=====��������ʼ��
	Encoder_Init_TIM2();
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
	Encoder_Init_TIM5();
#endif
	EXTI15_Init();
}

/*************************************************
Function: main()
Description: Main function entry
Input:  void
Output: void
Return: void
*************************************************/

int main()
{
	
	Peripheral_Init();

		while (1) {

			EXTI15_10_IRQHandler();
			delay_ms(1000);
		}
}

int main__(void)
{
	uint8_t PS2_Hold;
	Peripheral_Init();

	while (1)
	{	
		if(PS2_KEY)
		{
			do
			{
				if(PS2_Hold)	break;
				PS2_Hold = 1;
				printf("PS2 Key = %d",PS2_KEY);	
				switch(PS2_KEY)		//PS2��������
				{
					case 1:						break;		//select
					case 2:						break;		//Lҡ�˰���
					case 3:						break;		//Rҡ�˰���
					case 4:	PS2_BLU = 0;		break;		//start
					case 5:						break;		//Lǰ
					case 6:						break;		//L��
					case 7:						break;		//L��
					case 8:						break;		//L��
					case 9:						break;		//L2��
					case 10:					break;		//R2��
					case 11:					break;		//L1��
					case 12:					break;		//R1��
					case 13:	if(PS2_Hold && RC_Velocity < 25) RC_Velocity += 5;		break;		//R��
					case 14:					break;		//R��
					case 15:	if(PS2_Hold && RC_Velocity > 5) RC_Velocity -= 5;		break;		//R��
					case 16:					break;		//R��
				}
			}while(0);
		}
		else	{
			PS2_Hold = 0;
		}
			
		
		//PS2_Receive();

		delay_ms(10);

		//GPIO_WriteBit(GPIOC, GPIO_Pin_13, 1);

		#if OLED_DISPLAY_ENABLE
		//oled_show();
		#endif
	}
}
