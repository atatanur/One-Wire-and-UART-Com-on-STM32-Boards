/*
 * Author: Atakan Atanur
 * Date: 02.09.2019
 * Last edited: 05.09.2019
 */

#include "TherFunctions.h"

/*This function makes the sensor send its ID number*/
uint32_t Ther_Read_ROM(uint32_t* point, int sensorNum)
{
	uint32_t result=0;
	One_Wire_Init(sensorNum);
	One_Wire_Write_Byte(READ_ROM, sensorNum);
	for(uint32_t i=0; i<4; i++)
	{
		result |= ((uint32_t)One_Wire_Read_Byte(sensorNum)) << (8*i);

	}
	for(uint32_t i=0; i<4; i++)
	{
		*point |= ((uint32_t)One_Wire_Read_Byte(sensorNum) << (8*i));

	}
	return result;
}

/*This function writes the wanted bytes to the scratchpad*/
void Ther_Write_SP(uint8_t high, uint8_t low, uint8_t conf_reg, int sensorNum)
{
	One_Wire_Init(sensorNum);
	One_Wire_Write_Byte(SKIP_ROM, sensorNum);
	One_Wire_Write_Byte(WRITE_SP, sensorNum);

	One_Wire_Write_Byte(high, sensorNum);

	One_Wire_Write_Byte(low, sensorNum);

	One_Wire_Write_Byte(conf_reg, sensorNum);

}

/*This function reads the whole scratchpad*/
uint32_t Ther_Read_SP(uint32_t *pointer_1, uint32_t *pointer_2, int sensorNum)
{
	One_Wire_Init(sensorNum);
	One_Wire_Write_Byte(SKIP_ROM, sensorNum);
	One_Wire_Write_Byte(READ_SP,sensorNum);

	for(int i = 0; i < 4; i++)
	{
		*pointer_1 |= One_Wire_Read_Byte(sensorNum);
		*pointer_1 >>= 8;
	}
	for(int i = 0; i < 4; i++)
	{
		*pointer_2 |= One_Wire_Read_Byte(sensorNum);
		*pointer_2 >>= 8;
	}
	return 1;
}

/*This function converts the analog temperature data into digital*/
void Ther_Convert(void)
{
	One_Wire_Init(1);
	One_Wire_Init(2);
	One_Wire_Init(3);
	One_Wire_Write_Byte(SKIP_ROM, 1);
	One_Wire_Write_Byte(SKIP_ROM, 2);
	One_Wire_Write_Byte(SKIP_ROM, 3);
	One_Wire_Write_Byte(CONVERT_T, 1);
	One_Wire_Write_Byte(CONVERT_T, 2);
	One_Wire_Write_Byte(CONVERT_T, 3);
	HAL_Delay(750);
}

/*This function reads the temperature value inside the registers*/
uint16_t Ther_Read_Temp_Value(int sensorNum)
{
	uint16_t final_value = 0;
	uint8_t first_value, second_value;

	One_Wire_Init(sensorNum);
	One_Wire_Write_Byte(SKIP_ROM, sensorNum);
	One_Wire_Write_Byte(READ_SP, sensorNum);

	first_value = One_Wire_Read_Byte(sensorNum);						//The values from the sensor are being read here
	second_value = One_Wire_Read_Byte(sensorNum);

	final_value |= first_value;
	final_value |= ((uint16_t)second_value) << 8;

	return final_value;
}
