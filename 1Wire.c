/*
 * Author: Atakan Atanur
 * Date: 28.08.2019
 * Last edited: 04.09.2019
 */

#include <1Wire.h>
extern TIM_HandleTypeDef htim1;

uint32_t level, level2;

											// PG7 is going to be used for pin output and input//

void delay25us(void)						//Creates a delay for 2.5us
{
	HAL_TIM_Base_Start(&htim1);
	TIM1->CNT = 0x0;
	while(TIM1->CNT <= 4000 / 20);
	HAL_TIM_Base_Stop(&htim1);

}

void delay115us(void)						//Creates a delay for 11.5 us
{
	HAL_TIM_Base_Start(&htim1);
	TIM1->CNT = 0x0;
	while(TIM1->CNT <= 4000 / 2);
	HAL_TIM_Base_Stop(&htim1);
}

void One_Wire_Init(void)					//This function initializes or resets the sensor//
{
	GPIOG->MODER |= 1UL<<14; 				//Makes the pin output
	GPIOG->ODR &= ~(1UL<<7); 				//Sends 0 for initialization of the sensor
	for(int i = 0; i < 42; i++)
	{
		delay115us();
	}										//Sends it for 480us
	GPIOG->MODER &= ~(1UL<<14);				//Makes the pin input
	for(int i = 0; i < 6; i++)
	{
		delay115us();
	}
	level = GPIOG->IDR & GPIO_IDR_IDR_7;	//Test variable 1 - Checks the signal sent by the sensor

	for(int i = 0; i < 22; i++)
	{
		delay115us();
	}
	level2 = GPIOG->IDR & GPIO_IDR_IDR_7;	//Test variable 2 - Checks the signal sent by the sensor
}

void One_Wire_Write_Bit(uint8_t wr_bit)		//This function sends a specific bit to the thermometer//
{
	GPIOG->MODER |= 1UL<<14; 				//Makes the pin output

	if(wr_bit & 0x01)							//Writing 1
	{
		GPIOG->ODR &= ~(1UL<<7); 			//Sends 0 to start sending process
		delay25us();
		delay25us();
		delay25us();						//7.5us delay
		GPIOG->MODER &= ~(1UL<<14);
		for(int i = 0; i < 7; i++)
		{
			delay115us();
		}									//80.5us delay
		delay25us();
		delay25us();						//5us delay
	}
	else									//Writing 0
	{
		GPIOG->ODR &= ~(1UL<<7); 			//Sends 0 to start sending process
		delay25us();
		delay25us();
		delay25us();						//7.5us delay
		for(int i = 0; i < 7; i++)
		{
			delay115us();
		}									//80.5us delay
		GPIOG->MODER &= ~(1UL<<14);
		delay25us();
		delay25us();						//5us delay
	}
}

void One_Wire_Write_Byte(uint8_t wr_byte) 		//This function sends 1 byte to the thermometer
{
	for(int i = 0; i <= 7; i++)
	{
		One_Wire_Write_Bit(wr_byte & 0x01);
		wr_byte >>= 1;
	}
}

uint8_t One_Wire_Read_Bit(void)				//This function reads one bit from the thermometer
{
	uint8_t re_bit = 0;
	GPIOG->MODER |= 1UL<<14; 				//Makes the pin output
	GPIOG->ODR &= ~(1UL<<7); 				//Sends 0 to start sending process
	delay25us();

	GPIOG->MODER &= ~(1UL<<14);				//Makes the pin input
	delay115us();
	re_bit |= ((GPIOG->IDR) & (1UL<<7))>>7;

	for(int i = 0; i < 6; i++)
	{
		delay115us();
	}

	return re_bit;
}

uint8_t One_Wire_Read_Byte(void)			//This function reads 1 byte from the thermometer
{
	uint8_t re_byte = 0;
	for(int i = 0; i <= 7; i++)
	{
		re_byte >>= 1;
		re_byte |= One_Wire_Read_Bit() << 7;
		 					//This is the old shift
//		re_byte = (byte >> 1) | (byte << (sizeof(byte) - 1));	//This is the new shift to the right
	}
	return re_byte;
}

