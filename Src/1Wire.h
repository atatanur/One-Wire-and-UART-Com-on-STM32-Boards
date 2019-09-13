/*
 * Author: Atakan Atanur
 * Date: 28.08.2019
 * Last edited: 05.09.2019
 */

#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include <stdint.h>

//Variables//


//Functions//
void delay25us(void);
void delay115us(void);
void One_Wire_Init(int);
void One_Wire_Write_Bit(uint8_t, int);
void One_Wire_Write_Byte(uint8_t, int);
uint8_t One_Wire_Read_Bit(int);
uint8_t One_Wire_Read_Byte(int);

