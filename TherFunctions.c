/*
 * Author: Atakan Atanur
 * Date: 02.09.2019
 * Last edited: 04.09.2019
 */

#include "TherFunctions.h"

uint32_t Ther_Read_ROM(uint32_t* point)
{
	uint32_t result=0;
	One_Wire_Init();
//	One_Wire_Write_Byte(SKIP_ROM);
	One_Wire_Write_Byte(READ_ROM);
	for(uint32_t i=0; i<4; i++)
	{
		result |= ((uint32_t)One_Wire_Read_Byte()) << (8*i);

	}
	for(uint32_t i=0; i<4; i++)
	{
		*point |= ((uint32_t)One_Wire_Read_Byte() << (8*i));

	}
	return result;
}

void Ther_Write_SP(uint8_t high, uint8_t low, uint8_t conf_reg)
{
	One_Wire_Init();
	One_Wire_Write_Byte(SKIP_ROM);
	One_Wire_Write_Byte(WRITE_SP);

	One_Wire_Write_Byte(high);

	One_Wire_Write_Byte(low);

	One_Wire_Write_Byte(conf_reg);

}

uint32_t Ther_Read_SP(uint32_t *pointer_1, uint32_t *pointer_2)
{
	One_Wire_Init();
	One_Wire_Write_Byte(SKIP_ROM);
	One_Wire_Write_Byte(READ_SP);

	for(int i = 0; i < 4; i++)
	{
		*pointer_1 |= One_Wire_Read_Byte();
		*pointer_1 >>= 8;
	}
	for(int i = 0; i < 4; i++)
	{
		*pointer_2 |= One_Wire_Read_Byte();
		*pointer_2 >>= 8;
	}
	return 1;
}

void Ther_Convert(void)											//This function converts the analog temperature data into digital
{
	One_Wire_Init();
	One_Wire_Write_Byte(SKIP_ROM);
	One_Wire_Write_Byte(CONVERT_T);
	HAL_Delay(750);
}

uint16_t Ther_Read_Temp_Value(void)								//This function reads the temperature value inside the registers
{
	uint16_t final_value = 0;
	uint8_t first_value, second_value;

	One_Wire_Init();
	One_Wire_Write_Byte(SKIP_ROM);
	One_Wire_Write_Byte(READ_SP);

	first_value = One_Wire_Read_Byte();						//Start reading the values
	second_value = One_Wire_Read_Byte();

	final_value |= first_value;
	final_value |= ((uint16_t)second_value) << 8;

	//	final_value = (final_value << 8) | (final_value >> 8);

	return final_value;
}
