/*
 * Author: Atakan Atanur
 * Date: 28.08.2019
 * Last edited: 05.09.2019
 */

#include <1Wire.h>
extern TIM_HandleTypeDef htim1;

uint32_t level, level2;

/* PG7(#1), PG6(#2) and PB4(#3) are going to be used for pin outputs and inputs*/

/*Creates a delay for 2.5 us*/
void delay25us(void)
{
	HAL_TIM_Base_Start(&htim1);
	TIM1->CNT = 0x0;
	while(TIM1->CNT <= 4000 / 20);
	HAL_TIM_Base_Stop(&htim1);

}

/*Creates a delay for 11.5 us*/
void delay115us(void)
{
	HAL_TIM_Base_Start(&htim1);
	TIM1->CNT = 0x0;
	while(TIM1->CNT <= 4000 / 2);
	HAL_TIM_Base_Stop(&htim1);
}

/*This function initializes or resets the sensor*/
void One_Wire_Init(int sensorNum)
{
	if(sensorNum == 1)							//for PG7
	{
		GPIOG->MODER |= 1UL<<14; 				//Makes the PG7 pin output
		GPIOG->ODR &= ~(1UL<<7); 				//Sends 0 for initialization of the sensor
		for(int i = 0; i < 42; i++)
		{
			delay115us();
		}										//Sends it for 480us
		GPIOG->MODER &= ~(1UL<<14);				//Makes the PG7 pin input
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
	else if(sensorNum == 2)						//for PG6
	{
		GPIOG->MODER |= 1UL<<12; 				//Makes the PG6 pin output
		GPIOG->ODR &= ~(1UL<<6); 				//Sends 0 for initialization of the sensor
		for(int i = 0; i < 42; i++)
		{
			delay115us();
		}										//Sends it for 480us
		GPIOG->MODER &= ~(1UL<<12);				//Makes the PG6 pin input
		for(int i = 0; i < 6; i++)
		{
			delay115us();
		}
		level = GPIOG->IDR & GPIO_IDR_IDR_6;	//Test variable 1 - Checks the signal sent by the sensor

		for(int i = 0; i < 22; i++)
		{
			delay115us();
		}
		level2 = GPIOG->IDR & GPIO_IDR_IDR_6;	//Test variable 2 - Checks the signal sent by the sensor
	}
	else										// forPB4
	{
		GPIOB->MODER |= 1UL<<8; 				//Makes the PB4 pin output
		GPIOB->ODR &= ~(1UL<<4); 				//Sends 0 for initialization of the sensor
		for(int i = 0; i < 42; i++)
		{
			delay115us();
		}										//Sends it for 480us
		GPIOB->MODER &= ~(1UL<<8);				//Makes the PB4 pin input
		for(int i = 0; i < 6; i++)
		{
			delay115us();
		}
		level = GPIOB->IDR & GPIO_IDR_IDR_4;	//Test variable 1 - Checks the signal sent by the sensor

		for(int i = 0; i < 22; i++)
		{
			delay115us();
		}
		level2 = GPIOB->IDR & GPIO_IDR_IDR_4;	//Test variable 2 - Checks the signal sent by the sensor
	}
}

/*This function sends a specific bit to the thermometer*/
void One_Wire_Write_Bit(uint8_t wr_bit, int sensorNum)
{
	if(sensorNum == 1)
	{
		GPIOG->MODER |= 1UL<<14; 				//Makes the PG7 pin output

		if(wr_bit & 0x01)						//Writing 1
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
	else if(sensorNum == 2)
	{
		GPIOG->MODER |= 1UL<<12; 				//Makes the PG6 pin output

		if(wr_bit & 0x01)						//Writing 1
		{
			GPIOG->ODR &= ~(1UL<<6); 			//Sends 0 to start sending process
			delay25us();
			delay25us();
			delay25us();						//7.5us delay
			GPIOG->MODER &= ~(1UL<<12);
			for(int i = 0; i < 7; i++)
			{
				delay115us();
			}									//80.5us delay
			delay25us();
			delay25us();						//5us delay
		}
		else									//Writing 0
		{
			GPIOG->ODR &= ~(1UL<<6); 			//Sends 0 to start sending process
			delay25us();
			delay25us();
			delay25us();						//7.5us delay
			for(int i = 0; i < 7; i++)
			{
				delay115us();
			}									//80.5us delay
			GPIOG->MODER &= ~(1UL<<12);
			delay25us();
			delay25us();						//5us delay
		}
	}
	else
	{
		GPIOB->MODER |= 1UL<<8; 				//Makes the PB4 pin output

		if(wr_bit & 0x01)						//Writing 1
		{
			GPIOB->ODR &= ~(1UL<<4); 			//Sends 0 to start sending process
			delay25us();
			delay25us();
			delay25us();						//7.5us delay
			GPIOB->MODER &= ~(1UL<<8);
			for(int i = 0; i < 7; i++)
			{
				delay115us();
			}									//80.5us delay
			delay25us();
			delay25us();						//5us delay
		}
		else									//Writing 0
		{
			GPIOB->ODR &= ~(1UL<<4); 			//Sends 0 to start sending process
			delay25us();
			delay25us();
			delay25us();						//7.5us delay
			for(int i = 0; i < 7; i++)
			{
				delay115us();
			}									//80.5us delay
			GPIOB->MODER &= ~(1UL<<8);
			delay25us();
			delay25us();						//5us delay
		}
	}
}

/*This function sends 1 byte to the thermometer*/
void One_Wire_Write_Byte(uint8_t wr_byte, int sensorNum)
{
	for(int i = 0; i <= 7; i++)
	{
		One_Wire_Write_Bit(wr_byte & 0x01, sensorNum);
		wr_byte >>= 1;
	}
}

/*This function reads one bit from the thermometer*/
uint8_t One_Wire_Read_Bit(int sensorNum)
{
	if(sensorNum == 1)
	{
		uint8_t re_bit = 0;
		GPIOG->MODER |= 1UL<<14; 				//Makes the PG7 pin output
		GPIOG->ODR &= ~(1UL<<7); 				//Sends 0 to start sending process
		delay25us();

		GPIOG->MODER &= ~(1UL<<14);				//Makes the PG7 pin input
		delay115us();
		re_bit |= ((GPIOG->IDR) & (1UL<<7))>>7;

		for(int i = 0; i < 6; i++)
		{
			delay115us();
		}

		return re_bit;
	}
	else if(sensorNum == 2)
	{
		uint8_t re_bit = 0;
		GPIOG->MODER |= 1UL<<12; 				//Makes the PG6 pin output
		GPIOG->ODR &= ~(1UL<<6); 				//Sends 0 to start sending process
		delay25us();

		GPIOG->MODER &= ~(1UL<<12);				//Makes the PG6 pin input
		delay115us();
		re_bit |= ((GPIOG->IDR) & (1UL<<6))>>6;

		for(int i = 0; i < 6; i++)
		{
			delay115us();
		}

		return re_bit;
	}
	else
	{
		uint8_t re_bit = 0;
		GPIOB->MODER |= 1UL<<8; 				//Makes the PB4 pin output
		GPIOB->ODR &= ~(1UL<<4); 				//Sends 0 to start sending process
		delay25us();

		GPIOB->MODER &= ~(1UL<<8);				//Makes the PB4 pin input
		delay115us();
		re_bit |= ((GPIOB->IDR) & (1UL<<4))>>4;

		for(int i = 0; i < 6; i++)
		{
			delay115us();
		}

		return re_bit;
	}
}

/*This function reads 1 byte from the thermometer*/
uint8_t One_Wire_Read_Byte(int sensorNum)
{
	uint8_t re_byte = 0;
	for(int i = 0; i <= 7; i++)
	{
		re_byte >>= 1;
		re_byte |= One_Wire_Read_Bit(sensorNum) << 7;
	}
	return re_byte;
}
