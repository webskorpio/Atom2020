#include "stm32f0xx_hal.h"
#include "stdio.h"
//#include "usbd_cdc_if.h"
//#include "usb_device.h"

extern UART_HandleTypeDef huart2;
struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;
FILE __stderr;

int fputc(int c, FILE *f) {
 uint8_t data [1] = {c};
 // USART_SendData(USART1, (uint16_t) c);
	HAL_UART_Transmit(&huart2, &data[0], 1, 1000);
		//while ((USART1->ISR & UART_FLAG_TC)==0)
		//{
		//} 
	//uint8_t a = 0;
	//a = c;
	//while(CDC_Transmit_FS(&data[0], 1)!=USBD_OK);
	
	//USART1->TDR = (c & (uint16_t)0x01FF);
	UART_WaitOnFlagUntilTimeout(&huart2, UART_FLAG_TC, RESET, 0U, 100U); 

//		{
//    };
	return c;
}


int fgetc(FILE *f) {
  uint16_t c;
/*	c=USART_ReceiveData(USART1);
	USART_SendData(USART1,c);
	while (!USART_GetFlagStatus(USART1, USART_FLAG_TC)){};*/
	return (c);
}


int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}

