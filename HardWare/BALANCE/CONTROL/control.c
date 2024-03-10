#include "control.h"	
#include "filter.h"
#include "config.h"

  /**************************************************************************
作者：墨比斯科技
我的淘宝小店：https://moebius.taobao.com/
**************************************************************************/
u8 Target_Z;
u8 Flag_Target,Flag_Change;				//相关标志位
u8 PS2_BLU;
u8 temp1;								//临时变量
float Voltage_Count,Voltage_All;		//电压采样相关变量
float Gyro_K=-0.6;						//陀螺仪比例系数
int Gyro_Bias;
int j;
unsigned int TimClk = 200;
int speedMultiplier = 2;
int speedMultiplierLeftRight = 1;
#define a_PARAMETER          (0.311f)               
#define b_PARAMETER          (0.3075f)         
/**************************************************************************
函数功能：小车运动数学模型
入口参数：X Y Z 三轴速度或者位置
返回  值：无
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
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	int LX,LY,RX,RY;

	int Yuzhi=20;
	if(INT==0)		
	{     
		EXTI->PR=1<<15;                                                      //清除LINE5上的中断标志位
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
			if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //给主函数提供50ms的精准延时
		}
																					//===10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
#if   ENCODER_DIRECTION
		Encoder_A	=	-Read_Encoder(2);                                          //===读取编码器的值
		Position_A	+=	Encoder_A;                                                 //===积分得到位置 
		Encoder_B	=	+Read_Encoder(3);                                          //===读取编码器的值
		Position_B	+=	Encoder_B;                                                 //===积分得到位置 
		Encoder_C	=	+Read_Encoder(4);                                         //===读取编码器的值
		Position_C	+=	Encoder_C;                                                 //===积分得到位置  
		Encoder_D	=	-Read_Encoder(5);                                       //===读取编码器的值
		Position_D	+=	Encoder_D;                                                 //===积分得到位置    
#else
		Encoder_A	=	+Read_Encoder(2);                                          //===读取编码器的值
		Position_A	+=	Encoder_A;                                                 //===积分得到位置 
		Encoder_B	=	-Read_Encoder(3);                                          //===读取编码器的值
		Position_B	+=	Encoder_B;                                                 //===积分得到位置 
		Encoder_C	=	-Read_Encoder(4);                                         //===读取编码器的值
		Position_C	+=	Encoder_C;                                                 //===积分得到位置  
		Encoder_D	=	+Read_Encoder(5);                                       //===读取编码器的值
		Position_D	+=	Encoder_D;                                                 //===积分得到位置    
#endif

		Read_DMP();                                                            //===更新姿态	
		Voltage_All+=Get_battery_volt();                                       //多次采样累积
		if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值 获取电池电压	       

#if		ENCODER_ENABLE
		Motor_A=Incremental_PI_A(Encoder_A,Target_A);                         //===速度闭环控制计算电机A最终PWM
		Motor_B=Incremental_PI_B(Encoder_B,Target_B);                         //===速度闭环控制计算电机B最终PWM
		Motor_C=Incremental_PI_C(Encoder_C,Target_C);                         //===速度闭环控制计算电机C最终PWM
		Motor_D=Incremental_PI_D(Encoder_D,Target_D);                         //===速度闭环控制计算电机C最终PWM
#else
		Motor_A = Target_A;                         //===速度闭环控制计算电机A最终PWM
		Motor_B = Target_B;                         //===速度闭环控制计算电机B最终PWM
		Motor_C = Target_C;                         //===速度闭环控制计算电机C最终PWM
		Motor_D = Target_D;                         //===速度闭环控制计算电机C最终PWM
#endif
		if(InspectQueue())
		{
		//printf("Debug: InspectQueue\r\n");
			Flag_Direction=OutQueue();
		}else{
			
		//printf("Debug: InspectQueue (skipping)\r\n");
		}
		/*else
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
		}*/

		Get_RC(0);
		
		Xianfu_Pwm(6900);                     //===PWM限幅
		Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);     //===赋值给PWM寄存器 
		
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
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
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
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
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
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
	u8 temp;
	if(voltage<2000||EN==0)//电池电压低于22.2V关闭电机
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
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	u32 temp;
		if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)	Pwm=7200;
	 if(Pwm<-7200)	Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)	Pwm=7200;
	 if(Pwm<-7200)	Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)	Pwm=7200;
	 if(Pwm<-7200)	Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_D (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)	Pwm=7200;
	 if(Pwm<-7200)	Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
/**************************************************************************
函数功能：通过串口指令对小车进行遥控
入口参数：串口指令
返回  值：无
**************************************************************************/
void Get_RC(u8 mode)
{
	float step=0.25;  //设置速度控制步进值。
	float stepSide=0.04;  //设置速度控制步进值。
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
		
	switch(Flag_Direction)   //方向控制
	{
		
		
		// speed increase
		case 'e':	RC_Velocity+=2;	Flag_Move=1;Flag_Direction = NULL; 									break;
		case 'q':	RC_Velocity-=2;	Flag_Move=1;Flag_Direction = NULL; 								break;
		// speed stop
		case 'r':	Move_X = 0;		Move_Y=0;		Move_Z=0;					break;
		
		//case "N": serialMessage = 'w'; break;
		//case "E": serialMessage = 'd'; break;
		//case "S": serialMessage = 's'; break;
		//case "W": serialMessage = 'a'; break;
		//case "C": serialMessage = 'o'; break;
		//case "NW": serialMessage = 'h'; break;
		//case "NE": serialMessage = 'y'; break;
		//case "SE": serialMessage = 'g'; break;
		//case "SW": serialMessage = 't'; break;
		//default: serialMessage = 'o'; break;

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
		
		
		//wasd first:
		//case 'w':	Move_X=0;		Move_Y+=step;				Flag_Move=1;	break;
		//case 's':	Move_X=0;		Move_Y-=step;				Flag_Move=1;	break;
		//case 'a':	Move_X-=stepSide;	Move_Y=0;					Flag_Move=1;	break;
		//case 'd':	Move_X+=stepSide;	Move_Y=0;					Flag_Move=1;	break;
		//case 't':	Move_X+=stepSide;	Move_Y-=stepSide;				Flag_Move=1;	break;
		//case 'y':	Move_X+=stepSide;	Move_Y+=stepSide;				Flag_Move=1;	break;
		//case 'g':	Move_X-=stepSide;	Move_Y-=stepSide;				Flag_Move=1;	break;
		//case 'h':	Move_X-=stepSide;	Move_Y+=stepSide;				Flag_Move=1;	break; 
		
		//only moving single direction
		//case 'w':	Move_Y+=step;				Flag_Move=1;	break;
		//case 's':	Move_Y-=step;				Flag_Move=1;	break;
		//case 'a':	Move_X-=stepSide;					Flag_Move=1;	break;
		//case 'd':	Move_X+=stepSide;					Flag_Move=1;	break;
		//case 't':	Move_X+=stepSide;	Move_Y-=step;				Flag_Move=1;	break;
		//case 'y':	Move_X+=stepSide;	Move_Y+=step;				Flag_Move=1;	break;
		//case 'g':	Move_X-=stepSide;	Move_Y-=step;				Flag_Move=1;	break;
		//case 'h':	Move_X-=stepSide;	Move_Y+=step;				Flag_Move=1;	break; 
		
		//wasd first:
		//case 'w':	Move_X=0;		Move_Y+=step;				Flag_Move=1;	break;
		//case 's':	Move_X=0;		Move_Y-=step;				Flag_Move=1;	break;
		//case 'a':	Move_X-=step;	Move_Y=0;					Flag_Move=1;	break;
		//case 'd':	Move_X+=step;	Move_Y=0;					Flag_Move=1;	break;
		//case 't':	Move_X+=step;	Move_Y-=step;				Flag_Move=1;	break;
		//case 'y':	Move_X+=step;	Move_Y+=step;				Flag_Move=1;	break;
		//case 'g':	Move_X-=step;	Move_Y-=step;				Flag_Move=1;	break;
		//case 'h':	Move_X-=step;	Move_Y+=step;				Flag_Move=1;	break; 
		
		//case 'u':	Move_Z-=step;		Gyro_Bias = Yaw;	break;
		//case 'i':	Move_Z+=step;		Gyro_Bias = Yaw;	break;

		default: Flag_Move=0;        Move_X=Move_X/1.04;	Move_Y=Move_Y/1.04;	Move_Z=Move_Z/1.04;	  break;	 
	}
	
	
	if(RC_Velocity > 25)RC_Velocity = 25;
	if(RC_Velocity < 1)RC_Velocity = 1;
	//if(Flag_Move==1)		Flag_Left=0,Flag_Right=0;//Move_Z=0;
	if(Move_X<-RC_Velocity)	Move_X=-RC_Velocity;	   //速度控制限幅
	if(Move_X>RC_Velocity)	Move_X=RC_Velocity;	     
	if(Move_Y<-RC_Velocity)	Move_Y=-RC_Velocity;	
	if(Move_Y>RC_Velocity)	Move_Y=RC_Velocity;	 
	if(Move_Z<-RC_Velocity)	Move_Z=-RC_Velocity;	
	if(Move_Z>RC_Velocity)	Move_Z=RC_Velocity;	 
	
	Kinematic_Analysis(Move_X,Move_Y,Move_Z);//得到控制目标值，进行运动学分析
}
