/*
 * Author: Atakan Atanur
 * Date: 06.09.2019
 * Last edited: 09.09.2019
 */

#include "UARTCom.h"

extern UART_HandleTypeDef huart6;

/*This function initializes the UART properties*/
void uartInit(void)
{
	USART6->CR1 |= 1UL << 0;							//Enables the UART communication
	USART6->CR1 &= ~(1UL << 28);						//Makes the length of the transmitted data 8 bits
	USART6->CR2 &= ~((uint32_t)0x3 << 12);				//Makes the stop bit 1 bit
	USART6->CR1 |= 1UL << 2;							//Enables the receiver
	USART6->CR1 |= 1UL << 3;							//Enables the transmitter
	USART6->CR1 |= 1UL << 5;							//Enables the receive interrupt
}

/*This function sends a single byte using UART communication*/
void uartTransmitByte(uint8_t tra_byte)
{
	USART6->TDR = tra_byte;								//The byte that is wanted to be transmitted is written into the registers
	while(!(USART6->ISR & ((uint32_t)1 << 6)));			//There is a delay until the data is send
}

/*This function sends a specific amount of bytes using UART communication*/
void uartTransmitBytes(uint8_t* byteArray, int size)
{
	for(int i = 0; i <= size - 1; i++)					//The wanted amount of bytes are being send
	{
		uartTransmitByte(byteArray[i]);
		HAL_Delay(1);
	}
}

/*This is being done in the interrupt*/
uint8_t uartReceive(void)
{
	return 0;
}
