  /**
  ******************************************************************************
  * @file    GUI_App.c
  * @author  MCD Application Team
  * @brief   Simple demo drawing "Hello world"  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2018 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "GUI_App.h"
#include "GUI.h"

void GRAPHICS_MainTask(void) {

/* USER CODE BEGIN GRAPHICS_MainTask */
	int color = 0;
	int color2 = 0;
	int color3 = 0;
	ki = 0;

	Ther_Convert();
	temperatureNew = Ther_Read_Temp_Value(1);
	numnum = byte_to_double(temperatureNew);
	temperatureNew2 = Ther_Read_Temp_Value(2);
	numnum2 = byte_to_double(temperatureNew2);
	temperatureNew3 = Ther_Read_Temp_Value(3);
	numnum3 = byte_to_double(temperatureNew3);

	GUI_SetBkColor(GUI_BLACK);
    GUI_Clear();
    GUI_SetColor(GUI_WHITE);

//    GUI_SetFont(&GUI_Font32_1);
//    GUI_DispStringAt("Temperatures: ", 30, 55);
    GUI_SetFont(&GUI_Font8_1);
    GUI_DispStringAt("Temp#1", 242, 2);
    GUI_DispStringAt("Temp#2", 2, 138);
    GUI_DispStringAt("Temp#3", 242, 138);

    GUI_SetFont(&GUI_FontD64);

//    GUI_DispStringAt("Temperature (#1): ", 260, 68);
    GUI_GotoY(45);
    GUI_GotoX(250);
    GUI_DispFloat(numnum, 5);
//    GUI_DispString("°C");
//    GUI_GotoX(0); GUI_GotoY(0);

//    GUI_DispStringAt("Temperature (#2): ", 20, 204);
    GUI_GotoY(175);
    GUI_GotoX(10);
    GUI_DispFloat(numnum2, 5);

//    GUI_DispStringAt("Temperature (#3): ", 260, 204);
    GUI_GotoY(175);
    GUI_GotoX(250);
    GUI_DispFloat(numnum3, 5);

    /*JPEG printing and filling the insides*/
//    GUI_SetColor(GUI_BLUE);
//	  #include "Thermometer.c"
//    GUI_JPEG_Draw(_acThermometer, sizeof(_acThermometer),350,8);
//	  GUI_ClearRect(362,18,376,250);
//    GUI_DrawGradientV(362,(229 - ((numnum + 50) * 179/170)),376,250,GUI_CYAN, GUI_BLUE);
//    GUI_FillCircle(369,245,17);

    GUI_DrawLine(240, 0, 240,272);
    GUI_DrawLine(0, 136, 480,136);


    while(1)
	{
		HAL_Delay(5);																			//A delay just in case

		if (ki >= 3)
		{
			ki = 0;
		}

		/*Temperature conversion for sensors and reading the data from them*/
		Ther_Convert();
		temperatureNew = Ther_Read_Temp_Value(1);
		temperatureNew2 = Ther_Read_Temp_Value(2);
		temperatureNew3 = Ther_Read_Temp_Value(3);

//		GUI_ClearRect(362,(229 - ((numnum + 50) * 179/170)),376,250);

		numnum = byte_to_double(temperatureNew);
		numnum2 = byte_to_double(temperatureNew2);
		numnum3 = byte_to_double(temperatureNew3);


		/*Displaying UART data*/
	    GUI_GotoY(1);
	    GUI_GotoX(1);
	    GUI_DispFloat(rec_byte, 2);

	    /*Displaying the temperature value from sensor #1*/
		if(numnum >= 31 && !color)
		{
			GUI_SetColor(GUI_BLUE);
			color++;
		}
		else
		{
			GUI_SetColor(GUI_WHITE);
			color = 0;
		}
//    	GUI_DispStringAt("Temperature (#1): ", 260, 68);
	    GUI_GotoY(45);
	    GUI_GotoX(250);
	    GUI_DispFloat(numnum, 5);
//	    GUI_DispString("°C");
//	    GUI_GotoX(0); GUI_GotoY(0);
		GUI_SetColor(GUI_WHITE);

		/*Displaying the temperature value from sensor #2*/
		if(numnum2 >= 31 && !color2)
		{
			GUI_SetColor(GUI_BLUE);
			color2++;
		}
		else
		{
			GUI_SetColor(GUI_WHITE);
			color2 = 0;
		}
//    	GUI_DispStringAt("Temperature (#2): ", 20, 204);
	    GUI_GotoY(175);
	    GUI_GotoX(10);
	    GUI_DispFloat(numnum2, 5);
		GUI_SetColor(GUI_WHITE);

		/*Displaying the temperature value from sensor #3*/
		if(numnum3 >= 31 && !color3)
		{
			GUI_SetColor(GUI_BLUE);
			color3++;
		}
		else
		{
			GUI_SetColor(GUI_WHITE);
			color3 = 0;
		}
//    	GUI_DispStringAt("Temperature (#3): ", 260, 204);
	    GUI_GotoY(175);
	    GUI_GotoX(250);
	    GUI_DispFloat(numnum3, 5);
		GUI_SetColor(GUI_WHITE);

		/*Drawing the line that separates the screen into 4 pieces*/
	    GUI_DrawLine(240, 0, 240,272);
	    GUI_DrawLine(0, 136, 480,136);
}
   
/* USER CODE END GRAPHICS_MainTask */
  while(1)
{
      GUI_Delay(100);
}
}

/*************************** End of file ****************************/
