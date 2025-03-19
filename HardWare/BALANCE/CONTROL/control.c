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
int speedMultiplier = 2;
int speedMultiplierLeftRight = 1;
#define a_PARAMETER          (0.311f)               
#define b_PARAMETER          (0.3075f)         
/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
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
	Target_A   = -Vx+Vy+Vz;//*(a_PARAMETER+b_PARAMETER);
	Target_B   = +Vx+Vy-Vz;//*(a_PARAMETER+b_PARAMETER);
	Target_C   = -Vx+Vy-Vz;//*(a_PARAMETER+b_PARAMETER);
	Target_D   = +Vx+Vy+Vz;//*(a_PARAMETER+b_PARAMETER);
}
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ�����ʱ��ͬ��				 
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
				//printf(" - IRQ() ... LED\r\n");
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
		
		

		if(Incoming_Command_Movement_Legacy==1){
			// if we're getting legacy WASD commands, call Get_RC() which will convert these to motor XY 
			
			Get_RC();
		}else{
			// otherwise we have already called Kinematic_Analysis() from BlueTooth.c with the args.
			// printf(" - IRQ() ... doing joystick\r\n");
		}
		
		Xianfu_Pwm(6900);                     //===PWM�޷�
		//printf(" - IRQ() Setting PWM to: %d, %d, %d, %d\r\n", Motor_A, Motor_B, Motor_C, Motor_D);
		Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);     //===��ֵ��PWM�Ĵ��� 
		
		//USART_SendData(USART1,':');
	//		USART_SendData(USART1,Motor_A);
	//		USART_SendData(USART1,Motor_B);
	// USART_SendData(USART1,Motor_C);
	//		USART_SendData(USART1,Motor_D);
	//	USART_SendData(USART1,':');
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
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
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
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	u32 temp;
		if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
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
�������ܣ�ͨ������ָ���С������ң��
��ڲ���������ָ��
����  ֵ����
**************************************************************************/
void Get_RC()
{

	if(InspectQueue()){
		Flag_Direction = OutQueue();
		// printf(" - Get_RC() New Flag_Direction %d\r\n", Flag_Direction);
	}else{
		//printf(" - Get_RC() OLD Flag_Direction %d\r\n", Flag_Direction);
	}

	float step=0.25;  //�����ٶȿ��Ʋ���ֵ��
	float stepSide=0.04;  //�����ٶȿ��Ʋ���ֵ��
	u8 Flag_Move=1;
	

	// speed stuff
	switch(Flag_Direction) 
	{
		case '1':	speedMultiplier = 1;	speedMultiplierLeftRight = 1;	break;
		case '2':	speedMultiplier = 2;	speedMultiplierLeftRight = 1;	break;
		case '3':	speedMultiplier = 3;	speedMultiplierLeftRight = 2;	break;
		case '4':	speedMultiplier = 4;	speedMultiplierLeftRight = 3;	break;
		case '5':	speedMultiplier = 6;	speedMultiplierLeftRight = 3;	break;
		case '6':	speedMultiplier = 7;	speedMultiplierLeftRight = 3;	break;
		case '7':	speedMultiplier = 9;	speedMultiplierLeftRight = 3;	break;
		case '8':	speedMultiplier = 11;	speedMultiplierLeftRight = 3;	break;
		case '9':	speedMultiplier = 13;	speedMultiplierLeftRight = 3;	break;
	}
	
	// just hard coding this to 25 for now
	RC_Velocity = 25;

	float moveForwardSpeed = 2 * speedMultiplier;
	float moveZSpeed = 1 * speedMultiplier;
	float moveLeftRightSpeed = 0.5 * speedMultiplierLeftRight;

		
	switch(Flag_Direction)   //�������
	{
		
		
		// speed increase
		case 'e':	RC_Velocity+=2;	Flag_Move=1;Flag_Direction = NULL; 									break;
		case 'q':	RC_Velocity-=2;	Flag_Move=1;Flag_Direction = NULL; 								break;
		// speed stop
		case 'r':	Move_X = 0;		Move_Y=0;		Move_Z=0;					break;

		// Motors:
		// [B] [A]
		// [C] [D]
		// Targets: (A, B, C, D)
		// 10, 10, 10, 10 = forward
		// -10, -10, -10, -10 = back
		// -10 10 -10 10 = slide right
		// 10 -10 10 -10 = slide left
		// 5, 10, 10, 5 = forward slight right
		// 10, 5, 5, 10 = forward slide left
		// -5, -10, -10, -5 = back slight right
		// -10, -5, -5, -10 = back slight left
		// -10, 10, 10, -10 = rotate right
		// 10, -10, -10, 10 = rotate left
		
		
		//wasd first:
		case 'w':	Move_X=0;		Move_Y=moveForwardSpeed;	Move_Z=0;			Flag_Move=1;	break; // North 
		case 's':	Move_X=0;		Move_Y=-moveForwardSpeed;	Move_Z=0;			Flag_Move=1;	break; // South
		case 'a':	Move_X=0;	Move_Y=0;		Move_Z=moveZSpeed;		Flag_Move=1;	break; // rotate left
		case 'd':	Move_X=0;	Move_Y=0;		Move_Z=-moveZSpeed;		Flag_Move=1;	break; // rotate right
		
		case 'y':	Move_X=0;	Move_Y=moveZSpeed;		Move_Z=-moveZSpeed;		Flag_Move=1;	break; // NE
		case 'g':	Move_X=0;	Move_Y=-moveZSpeed;		Move_Z=-moveZSpeed;		Flag_Move=1;	break; // SE
		case 't':	Move_X=0;	Move_Y=-moveZSpeed;		Move_Z=moveZSpeed;		Flag_Move=1;	break; // SW
		case 'h':	Move_X=0;	Move_Y=moveZSpeed;	Move_Z=moveZSpeed;		Flag_Move=1;	break; // NW
		
		case 'c':	Move_X=-moveZSpeed * 2;	Move_Y=0;	Move_Z=0;			Flag_Move=1;	break; // slide left
		case 'v':	Move_X=moveZSpeed * 2; Move_Y=0;  Move_Z=0;     Flag_Move=1;	break; // slide right
		

		default: Flag_Move=0;        Move_X=Move_X/1.04;	Move_Y=Move_Y/1.04;	Move_Z=Move_Z/1.04;	  break;	 
	}
	
	
	if(RC_Velocity > 25)RC_Velocity = 25;
	if(RC_Velocity < 1)RC_Velocity = 1;
	//if(Flag_Move==1)		Flag_Left=0,Flag_Right=0;//Move_Z=0;
	if(Move_X<-RC_Velocity)	Move_X=-RC_Velocity;	   //�ٶȿ����޷�
	if(Move_X>RC_Velocity)	Move_X=RC_Velocity;	     
	if(Move_Y<-RC_Velocity)	Move_Y=-RC_Velocity;	
	if(Move_Y>RC_Velocity)	Move_Y=RC_Velocity;	 
	if(Move_Z<-RC_Velocity)	Move_Z=-RC_Velocity;	
	if(Move_Z>RC_Velocity)	Move_Z=RC_Velocity;	 
	
	Kinematic_Analysis(Move_X,Move_Y,Move_Z);//�õ�����Ŀ��ֵ�������˶�ѧ����
}
