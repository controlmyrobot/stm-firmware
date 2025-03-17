#include "usartx.h"

/**************************ʵ�ֺ���**********************************************
*��    ��:		usart4����һ���ֽ�
*********************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ�����pclk2:PCLK2 ʱ��Ƶ��(Mhz)    bound:������
����  ֵ����
**************************************************************************/
void uart2_init(u32 pclk2,u32 bound)
{  	 
float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
	mantissa<<=4;
	mantissa+=fraction; 

	RCC->APB2ENR|=1<<0;    //��������ʱ��
	RCC->APB2ENR|=1<<4;   //ʹ��PORTC��ʱ��  
	RCC->APB1ENR|=1<<18;  //ʹ�ܴ���ʱ�� 
	GPIOC->CRH&=0XFFFF00FF; 
	GPIOC->CRH|=0X00008B00;//IO״̬����
	GPIOC->ODR|=1<<10;	 
	AFIO->MAPR|=1<<4;      //������ӳ��

	RCC->APB1RSTR|=1<<18;   //��λ����1
	RCC->APB1RSTR&=~(1<<18);//ֹͣ��λ	   	   
	//����������
 	USART2->BRR=mantissa; // ����������	 
	USART2->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART2->CR1|=1<<8;    //PE�ж�ʹ��
	USART2->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(0,1,USART2_IRQn,2);//��2��������ȼ� 
	printf("USART2_Init done\r\n");
}

// Command definitions
#define CMD_START_BYTE    0xFF
#define CMD_JOYSTICK     0x01
#define CMD_GET_VOLTAGE  0x02
#define CMD_GET_GYRO     0x03
#define CMD_GET_STATS    0x04

// Response definitions
#define RESP_START_BYTE  0xAA
#define RESP_OK         0x00
#define RESP_ERROR      0xFF

// Buffer for receiving commands
#define MAX_BUFFER_SIZE 16
static uint8_t cmdBuffer[MAX_BUFFER_SIZE];

// Function to send response packet
void send_response(uint8_t cmd_type, uint8_t *data, uint8_t data_len) {
    usart2_send(RESP_START_BYTE);
    usart2_send(cmd_type);
    usart2_send(data_len);
    
    for(uint8_t i = 0; i < data_len; i++) {
        usart2_send(data[i]);
    }
}

// Process received command
void process_command(uint8_t *cmd_buffer, uint8_t len) {
	printf("process_command\r\n");
    switch(cmd_buffer[0]) {  // Command type
        case CMD_JOYSTICK: {
            // Expecting X, Y, Z values in next 3 bytes
            if(len >= 4) {
                uint8_t x = cmd_buffer[1];
                uint8_t y = cmd_buffer[2];
                uint8_t z = cmd_buffer[3];
                // TODO: Handle joystick values
                uint8_t resp = RESP_OK;
                send_response(CMD_JOYSTICK, &resp, 1);
            }
            break;
        }
        
        case CMD_GET_VOLTAGE: {
            // TODO: Get voltage reading from your ADC
            uint16_t voltage = 1; //get_voltage(); // You'll need to implement this
            uint8_t resp[2] = {
                (voltage >> 8) & 0xFF,
                voltage & 0xFF
            };
            send_response(CMD_GET_VOLTAGE, resp, 2);
            break;
        }
        
        case CMD_GET_GYRO: {
            // TODO: Get gyro readings
            int16_t gyro_data[3] = {0}; // Assuming X, Y, Z
            //get_gyro_data(gyro_data);   // You'll need to implement this
            uint8_t resp[6];
            for(int i = 0; i < 3; i++) {
                resp[i*2] = (gyro_data[i] >> 8) & 0xFF;
                resp[i*2+1] = gyro_data[i] & 0xFF;
            }
            send_response(CMD_GET_GYRO, resp, 6);
            break;
        }
        
        case CMD_GET_STATS: {
            // TODO: Implement your statistics gathering
            uint8_t stats[4] = {0}; // Example stats packet
            //get_system_stats(stats); // You'll need to implement this
            send_response(CMD_GET_STATS, stats, 4);
            break;
        }
    }
}

/**************************************************************************
ܣ3ж
ڲ
  ֵ����
**************************************************************************/
int USART2_IRQHandler(void)
{
	if (USART2->SR & (1 << 5))//���յ�����
	{
		static uint8_t count = 0;
		static uint8_t cmd_started = 0;
		uint8_t temp = USART2->DR;
		
		// Check for start byte
		if(temp == CMD_START_BYTE && !cmd_started) {
			cmd_started = 1;
			count = 0;
			return 0;
		}
		
		// If we're receiving a command, store it
		if(cmd_started && count < MAX_BUFFER_SIZE) {
			cmdBuffer[count++] = temp;
			
			// Process command if we have enough bytes
			// First byte after start byte is command type
			// Second byte is data length
			if(count >= 2) {
				uint8_t expected_len = cmdBuffer[1];
				if(count >= expected_len + 2) {
					process_command(cmdBuffer, count);
					cmd_started = 0;
					count = 0;
				}
			}
		}
	}
	return 0;
}



