#ifndef __USRATX_H
#define __USRATX_H 
#include "sys.h"	  	
void usart2_send(u8 data);
void USART2_Init(u32 bound);
int USART2_IRQHandler(void);

#endif

