/*
 * Author: Atakan Atanur
 * Date: 02.09.2019
 * Last edited: 05.09.2019
 */

#include "1Wire.h"

//Variables//
#define READ_ROM							0x33
#define SKIP_ROM							0xCC
#define CONVERT_T							0x44
#define WRITE_SP							0x4E
#define READ_SP								0xBE

//Functions//
uint32_t Ther_Read_ROM(uint32_t*, int);
void Ther_Write_SP(uint8_t, uint8_t, uint8_t, int);
uint32_t Ther_Read_SP(uint32_t*, uint32_t*, int);
void Ther_Convert(void);
uint16_t Ther_Read_Temp_Value(int);
