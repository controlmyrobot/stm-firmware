#ifndef _Sys_H
#define _Sys_H

//========================================================================
//ͷ�ļ�����

#include "stm32f10x.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "Systick.h"
#include "define.h"
#include "usart.h"
#include "usartx.h"
#include "LED.h"
#include "KEY.h"
#include "oled.h"
#include "show.h"
#include "motor.h"
#include "ioi2c.h"
#include "BlueTooth.h"
#include "control.h"
#include "exti.h"
#include "encoder.h"
#include "adc.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "pstwo.h"
#include "config.h"

extern int Encoder_A,Encoder_B,Encoder_C,Encoder_D;                    //���������������
extern long int Motor_A,Motor_B,Motor_C,Motor_D;                   //���PWM����
extern u8 Flag_Left,Flag_Right,Flag_sudu,Flag_Direction; //����ң����صı���
extern u8 Incoming_Command;
extern u8 Flag_Stop,Flag_Show;                               //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
extern long int Target_A,Target_B,Target_C,Target_D,Rate_A,Rate_B,Rate_C,Rate_D;                      //���Ŀ���ٶ�
extern  int Voltage,Voltage_Zheng,Voltage_Xiao;                //��ص�ѹ������صı���
extern float Angle_Balance,Gyro_Balance,Gyro_Turn;           //ƽ����� ƽ�������� ת��������
extern float Show_Data_Mb;                                    //ȫ����ʾ������������ʾ��Ҫ�鿴������
extern int Temperature;
extern u32 Distance;                                           //���������
extern u8 Bi_zhang,delay_50,delay_flag;
extern float Acceleration_Z;
extern int RC_Velocity,RC_Position;
extern int Encoder_A_EXTI;
extern u8 Run_Flag,PID_Send,Flash_Send,Turn_Flag;
extern u8 rxbuf[8],Urxbuf[8],txbuf[8],txbuf2[8],CAN_ON_Flag,Usart_ON_Flag,Usart_Flag,PS2_ON_Flag;
extern float Pitch,Roll,Yaw,Move_X,Move_Y,Move_Z; 
extern long int Position_A,Position_B,Position_C,Position_D;
extern u16 PID_Parameter[10],Flash_Parameter[10];
extern float	Position_KP,Position_KI,Position_KD;  //λ�ÿ���PID����
extern float Velocity_KP,Velocity_KI;	                    //�ٶȿ���PID����
extern int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern int Gryo_Z;
extern u8 PS2_BLU;
extern unsigned char Usart1_Buf;						//��λ��������Ϣ
extern unsigned char Usart3_Buf;

//Ex_NVIC_Configר�ö���
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6 

#define FTIR   1  //�½��ش���
#define RTIR   2  //�����ش���

void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);//����ƫ�Ƶ�ַ
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);//����NVIC����
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);//�����ж�
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);//�ⲿ�ж����ú���(ֻ��GPIOA~G)

#endif

