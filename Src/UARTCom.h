/*
 * Author: Atakan Atanur
 * Date: 06.09.2019
 * Last edited: 09.09.2019
 */

#include "stm32f7xx_hal.h"
#include <stdint.h>

//Variables

//Functions
void uartInit(void);
void uartTransmitByte(uint8_t);
void uartTransmitBytes(uint8_t*, int);
uint8_t uartReceiveByte(void);
