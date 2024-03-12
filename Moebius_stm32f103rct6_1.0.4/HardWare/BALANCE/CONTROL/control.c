#include "control.h"	
#include "filter.h"
#include "config.h"

  /**************************************************************************
���ߣ�ī��˹�Ƽ�
�ҵ��Ա�С�꣺https://moebius.taobao.com/
**************************************************************************/
u8 Target_Z;
u8 Flag_Target,Flag_Change;				//��ر�־λ
u8 PS2_BLU;
u8 temp1;								//��ʱ����
float Voltage_Count,Voltage_All;		//��ѹ������ر���
float Gyro_K=-0.6;						//�����Ǳ���ϵ��
int Gyro_Bias;
int j;
unsigned int TimClk = 200;
#define a_PARAMETER          (0.311f)               
#define b_PARAMETER          (0.3075f)         
/**************************************************************************
Function: Mathematical model of the car's movement
Input Parameters: X Y Z axis speed or position
Return Value: None
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{

#if	AXLE_Z_RESTRAIN
	int temp;
	if(!KEY1)	Gyro_Bias = Yaw;
	temp = Yaw - Gyro_Bias;
	if (temp > 180)
		temp = 360-temp;
	if (temp < -180)
		temp = 360 + temp;
	if(temp > 1 || temp < -1)
		Vz += Gyro_K * temp;
#endif
	Target_A   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
	Target_B   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
	Target_C   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
	Target_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}
/**************************************************************************
Function: All control codes are inside here.
A 5ms timed interrupt is triggered by the INT pin of MPU6050.
Strictly ensures the synchronization of sampling and data processing time.			 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	int LX,LY,RX,RY;

	int Yuzhi=20;
	if(INT==0)		
	{     
		EXTI->PR=1<<15;                                                      //���LINE5�ϵ��жϱ�־λ
		if(TimClk)
		{
			TimClk--;
			if(TimClk == 0)
			{
				TimClk = 200;
				LED = ~LED;
			}
		}
		Flag_Target=!Flag_Target;
		if(delay_flag==1)
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //���������ṩ50ms�ľ�׼��ʱ
		}
																					//===10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
#if   ENCODER_DIRECTION
		Encoder_A	=	-Read_Encoder(2);                                          //===��ȡ��������ֵ
		Position_A	+=	Encoder_A;                                                 //===���ֵõ�λ�� 
		Encoder_B	=	+Read_Encoder(3);                                          //===��ȡ��������ֵ
		Position_B	+=	Encoder_B;                                                 //===���ֵõ�λ�� 
		Encoder_C	=	+Read_Encoder(4);                                         //===��ȡ��������ֵ
		Position_C	+=	Encoder_C;                                                 //===���ֵõ�λ��  
		Encoder_D	=	-Read_Encoder(5);                                       //===��ȡ��������ֵ
		Position_D	+=	Encoder_D;                                                 //===���ֵõ�λ��    
#else
		Encoder_A	=	+Read_Encoder(2);                                          //===��ȡ��������ֵ
		Position_A	+=	Encoder_A;                                                 //===���ֵõ�λ�� 
		Encoder_B	=	-Read_Encoder(3);                                          //===��ȡ��������ֵ
		Position_B	+=	Encoder_B;                                                 //===���ֵõ�λ�� 
		Encoder_C	=	-Read_Encoder(4);                                         //===��ȡ��������ֵ
		Position_C	+=	Encoder_C;                                                 //===���ֵõ�λ��  
		Encoder_D	=	+Read_Encoder(5);                                       //===��ȡ��������ֵ
		Position_D	+=	Encoder_D;                                                 //===���ֵõ�λ��    
#endif

		Read_DMP();                                                            //===������̬	
		Voltage_All+=Get_battery_volt();                                       //��β����ۻ�
		if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ ��ȡ��ص�ѹ	       

#if		ENCODER_ENABLE
		Motor_A=Incremental_PI_A(Encoder_A,Target_A);                         //===�ٶȱջ����Ƽ�����A����PWM
		Motor_B=Incremental_PI_B(Encoder_B,Target_B);                         //===�ٶȱջ����Ƽ�����B����PWM
		Motor_C=Incremental_PI_C(Encoder_C,Target_C);                         //===�ٶȱջ����Ƽ�����C����PWM
		Motor_D=Incremental_PI_D(Encoder_D,Target_D);                         //===�ٶȱջ����Ƽ�����C����PWM
#else
		Motor_A = Target_A;                         //===�ٶȱջ����Ƽ�����A����PWM
		Motor_B = Target_B;                         //===�ٶȱջ����Ƽ�����B����PWM
		Motor_C = Target_C;                         //===�ٶȱջ����Ƽ�����C����PWM
		Motor_D = Target_D;                         //===�ٶȱջ����Ƽ�����C����PWM
#endif
		if(InspectQueue())
		{
			Flag_Direction=OutQueue();  
		}
		else
		{
			if((PS2_LX > 250 && PS2_LY > 250 &&PS2_RX > 250 &&PS2_RY > 250)
				|| (PS2_LX == 0 && PS2_LY == 0 &&PS2_RX == 0 &&PS2_RY == 0))
			{
				PS2_LX = 128;
				PS2_LY = 128;
				PS2_RX = 128;
				PS2_RY = 128;
			}
			LX=PS2_LX-128;
			LY=PS2_LY-128; 
			RX=PS2_RX-128;
			RY=PS2_RY-128;		
			if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
			if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
			if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
			if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
			
			Move_X=LX*RC_Velocity/(400 + RY);
			Move_Y=-LY*RC_Velocity/(400 + RY);	
			if(RX != 0)	Gyro_Bias = Yaw;
			Move_Z=-RX*RC_Velocity/(400 + RY);
		}
		
		
		Get_RC(0);

		
		Xianfu_Pwm(6900);                     //===PWM�޷�
		Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);      //===��ֵ��PWM�Ĵ���  
	}
	return 0;	 
} 


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
	if(motor_a<0)		INA2=1,			INA1=0;
	else				INA2=0,			INA1=1;
	PWMA=myabs(motor_a);

	if(motor_b<0)		INB2=1,			INB1=0;
	else				INB2=0,			INB1=1;
	PWMB=myabs(motor_b);

	if(motor_c>0)		INC2=1,			INC1=0;
	else				INC2=0,			INC1=1;
	PWMC=myabs(motor_c);

	if(motor_d>0)		IND2=1,			IND1=0;
	else				IND2=0,			IND1=1;
	PWMD=myabs(motor_d);
}

/**************************************************************************
Function: Limit PWM assignment
Input Parameter: Amplitude
Return Value: None
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
	if(Motor_A<-amplitude) Motor_A=-amplitude;	
	if(Motor_A>amplitude)  Motor_A=amplitude;	
	if(Motor_B<-amplitude) Motor_B=-amplitude;	
	if(Motor_B>amplitude)  Motor_B=amplitude;		
	if(Motor_C<-amplitude) Motor_C=-amplitude;	
	if(Motor_C>amplitude)  Motor_C=amplitude;		
	if(Motor_D<-amplitude) Motor_D=-amplitude;	
	if(Motor_D>amplitude)  Motor_D=amplitude;		
}


/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	u8 temp;
	if(voltage<2000||EN==0)//��ص�ѹ����22.2V�رյ��
	{	                                                
		temp=1;      
		PWMA=0;
		PWMB=0;
		PWMC=0;
		PWMD=0;							
	}
	else
		temp=0;
	return temp;			
}

/**************************************************************************
Function: Absolute value function
Input Parameter: long int
Return Value: unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	u32 temp;
		if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
/**************************************************************************
Function: Incremental PI Controller
Input Parameters: Encoder measurement, Target speed
Return Value: Motor PWM
According to the incremental discrete PID formula:
pwm += Kp[e(k) - e(k-1)] + Kie(k) + Kd[e(k) - 2e(k-1) + e(k-2)]
where e(k) represents the current deviation,
e(k-1) represents the previous deviation, and so on.
pwm represents the incremental output.
In our speed control closed-loop system, we only use PI control:
pwm += Kp[e(k) - e(k-1)] + Kie(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)	Pwm=7200;
	 if(Pwm<-7200)	Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)	Pwm=7200;
	 if(Pwm<-7200)	Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)	Pwm=7200;
	 if(Pwm<-7200)	Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_D (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)	Pwm=7200;
	 if(Pwm<-7200)	Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
/**************************************************************************
Function: Remotely control the car through serial port commands
Input Parameter: Serial port command
Return Value: None
**************************************************************************/
void Get_RC(u8 mode)
{
	float step=10;  //Set the speed control step value.
	u8 Flag_Move=1;

switch(Flag_Direction)   //Direction control
	{
		case 'A':	Move_X=0;		Move_Y+=step;	         	Flag_Move=1;	break;
		case 'B':	Move_X+=step/2;	Move_Y+=step/2;				Flag_Move=1;	break;
		case 'C':	Move_X+=step;	Move_Y=0;					Flag_Move=1;	break;
		case 'D':	Move_X+=step/2;	Move_Y-=step/2;				Flag_Move=1;	break;
		case 'E':	Move_X=0;		Move_Y-=step;				Flag_Move=1;	break;
		
		case 'F':	Move_X-=step/2;	Move_Y-=step/2;				Flag_Move=1;	break;
		case 'G':	Move_X-=step;	Move_Y=0;					Flag_Move=1;	break;
		case 'H':	Move_X-=step/2;	Move_Y+=step/2;				Flag_Move=1;	break; 
		case 'Z':	Move_X = 0;		Move_Y=0;		Move_Z=0;					break;
		case 'L':	RC_Velocity = 30;											break;
		case 'M':	RC_Velocity = 10;											break;
		case 'a':	Move_X = 0;		Move_Y=0;		Move_Z+=step; break;
		case 'b':	Move_X = 0;		Move_Y=0;		Move_Z-=step; /*Move_Z-=step;		Gyro_Bias = Yaw;*/	break;
		case 'c':	break;
		case 'd':	/*Move_Z+=step;		Gyro_Bias = Yaw;*/	break;

		case 'O':	break;	//����ͷ�� 
		case 'N': 	break;	//����ת��
		case 'I': 	break;	//ң��
		case 'J': 	break;	//����
		case 'K': 	break;	//����
		case 'P': 	break;	//LED_on
		case 'p': 	break;	//LED_off
		default: 	Flag_Move=0;        Move_X=Move_X/2;	 Move_Y=Move_Y/2;  break;	 
	}	
     
	if(Flag_Move==1)		Flag_Left=0, Flag_Right=0; //Move_Z=0;
	if(Move_X<-RC_Velocity)	Move_X=-RC_Velocity;	   //�ٶȿ����޷�
	if(Move_X>RC_Velocity)	Move_X=RC_Velocity;	     
	if(Move_Y<-RC_Velocity)	Move_Y=-RC_Velocity;	
	if(Move_Y>RC_Velocity)	Move_Y=RC_Velocity;	 
	if(Move_Z<-RC_Velocity)	Move_Z=-RC_Velocity;	
	if(Move_Z>RC_Velocity)	Move_Z=RC_Velocity;	 
	
	Kinematic_Analysis(Move_X,Move_Y,Move_Z); //�õ�����Ŀ��ֵ�������˶�ѧ����
}
