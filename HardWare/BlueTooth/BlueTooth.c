#include "BlueTooth.h"

/**************************************************************************
���ߣ�ī��˹�Ƽ�
�ҵ��Ա�С�꣺https://moebius.taobao.com/
**************************************************************************/

#define MAXQSIZE	10

LinkQueue *Uart_Queue;
int QueueSize;

#define MAX_BUFFER_SIZE 64
static uint8_t rxBuffer[MAX_BUFFER_SIZE];
static uint8_t rxIndex = 0;

//========================================================================
//
void InitQueue()
{
	Uart_Queue = (LinkQueue*)malloc(sizeof(LinkQueue));
	Uart_Queue->Hand = (QNode*)malloc(sizeof(QNode));
	Uart_Queue->Tail = Uart_Queue->Hand;
	if (!Uart_Queue->Hand)	printf("Er 0001: Init queue error!\r\n");
	Uart_Queue->Hand->next = NULL;
}

//========================================================================
//���
void InQueue(u8 Data)		//ͷ�巨��������
{
	if (QueueSize >= MAXQSIZE)
	{
		printf("Er 0002: Queue overflow!\r\n");
		return;
	}
	else
	{
		//printf("Debug: Adding queue data\r\n");
		//printf("%u", Data);
		QNode* NewNode = (QNode*)malloc(sizeof(QNode));
		NewNode->data = Data;
		NewNode->next = Uart_Queue->Hand;
		Uart_Queue->Hand = NewNode;
		QueueSize++;
	}
}

//========================================================================
//����
u8 OutQueue()
{
	QNode* Temp;
	Temp = Uart_Queue->Hand;
	if (Uart_Queue->Hand == Uart_Queue->Tail)
	{
		printf("Er 0002: Queue empty!\r\n");
		return 0;
	}
	while (Temp->next != Uart_Queue->Tail)
	{
		Temp = Temp->next;
	}
	Temp->next = NULL;
	free(Uart_Queue->Tail);
	Uart_Queue->Tail = Temp;
	QueueSize--;
	return Temp->data;
}

//========================================================================
//�������Ƿ�Ϊ��
u8 InspectQueue()
{
	if(Uart_Queue->Tail == Uart_Queue->Hand)
		return 0;
	else
		return 1;
}

//========================================================================
//�������
void PrintQueue()
{
	QNode* Temp;
	Temp = Uart_Queue->Hand;
	while (Temp->next != NULL)
	{
		printf("%c, ",Temp->data);
		Temp = Temp->next;
	}
	printf("Queue size %d \r\n",QueueSize);
}

void USART3_Send_Data(u8 Dat)
{ 
	USART_SendData(USART3,Dat); 
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET);      
	USART_ClearFlag(USART3,USART_FLAG_TC);
}
/*******************************************************************************
* �� �� ��         : USART1_Init
* ��������		   : USART1��ʼ������
* ��    ��         : bound:������
* ��    ��         : ��
*******************************************************************************/ 
void USART3_Init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE); 				//Uart3��ӳ��
 
	/*  ����GPIO��ģʽ��IO�� */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//TX				//�������PA2
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;			//�����������
	GPIO_Init(GPIOC,&GPIO_InitStructure);					/* ��ʼ����������IO */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;//RX			//��������PA3
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;		//ģ������
	GPIO_Init(GPIOC,&GPIO_InitStructure);					/* ��ʼ��GPIO */
	
	//USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;						//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;				//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure);				//��ʼ������1
	  
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
		
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			//��������ж�
	USART_Cmd(USART3, ENABLE);								//ʹ�ܴ���1 

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;	//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	InitQueue();
	printf("USART3_Init done\r\n");
}

/*******************************************************************************
* �� �� ��         : USART1_IRQHandler
* ��������		   : USART1�жϺ���
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/ 

void USART3_IRQHandler(void) {
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		printf("USART3_IRQHandler\r\n");
        uint8_t receivedByte = USART_ReceiveData(USART3);
        USART_SendData(USART3, receivedByte); // Echo back for confirmation

        // Store the received byte in the buffer
        if (rxIndex < MAX_BUFFER_SIZE) {
            rxBuffer[rxIndex++] = receivedByte;
        }

        // Check for end of message (e.g., newline character or specific length)
        if (receivedByte == '\n' || rxIndex >= MAX_BUFFER_SIZE) {
            // Process the complete message
            processReceivedMessage(rxBuffer, rxIndex);
            rxIndex = 0; // Reset index for next message
        }

        //while (USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET);
    }
    //USART_ClearFlag(USART3, USART_FLAG_TC);
}

#define CMD_MOVE 0x01

void processReceivedMessage(uint8_t *buffer, uint8_t length) {
    // Example: Check the first byte for command type
    switch (buffer[0]) {
        case CMD_MOVE:
            if (length >= 4) { // Ensure there are enough bytes for X, Y, Z
                int Move_X = (int)buffer[1];
                int Move_Y = (int)buffer[2];
                int Move_Z = (int)buffer[3];

                // Ensure values are within the range of 0 to 25
                if (Move_X < 0) Move_X = 0;
                if (Move_X > 25) Move_X = 25;
                if (Move_Y < 0) Move_Y = 0;
                if (Move_Y > 25) Move_Y = 25;
                if (Move_Z < 0) Move_Z = 0;
                if (Move_Z > 25) Move_Z = 25;

                Kinematic_Analysis((float)Move_X, (float)Move_Y, (float)Move_Z);

                EXTI15_10_IRQHandler();
            }
            break;
        
        // Handle other commands

        default:
            // Handle unknown command
            break;
    }
} 	
