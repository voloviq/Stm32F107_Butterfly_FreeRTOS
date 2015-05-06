/*
    Description:

    This C module is used to configure
    all needed peryferial on MCBSTM32
    on Kamami evaluation board witm
    Stm32F107Vbt6

	Compiler: Gnu Gcc

	Language: Ansi C

	Ide: Eclipse

	Author: Michal Wolowik

	Date: Rzeszow September 2009
*/







#ifndef _STM32L_LEVEL_H
#define _STM32L_LEVEL_H






typedef void(*Funkcja_1)(unsigned char);






#include "stm32f10x_lib.h"






/*
Function name	:	Low_Level_Initialize
Description		:	Function configure all peryferials
Input   		:	None
Output  		:	None
Function return :	None
*/
void Low_Level_Initialize(void);





/*
Function name	:	Rccu_Configure
Description		:	Function configure main clocks of STM32
Input   		:	None
Output  		:	None
Function return :	None
*/
void Rccu_Configure(void);






/*
Function name	:	Clean_All_Rcc_Register
Description		:	Function clean all needed Rcc registers
Input   		:	None
Output  		:	None
Function return :	None
*/
void Clean_All_Rcc_Register(void);






/*
Function name	:	Enable_External_Clock
Description		:	Function Enable High Speed External Colcking
Input   		:	None
Output  		:	None
Function return :	unsigned char: if function return 1 - HSE is active else 0 is disable
*/
unsigned char Enable_External_Clock(void);





/*
Function name	:	Gpio_Configure
Description		:	Function configure all used GPIO of STM32
Input   		:	None
Output  		:	None
Function return :	None
*/
void Gpio_Configure(void);






/*
Function name	:	Led_1
Description		:	Function drive PE14 Led 1
Input   		:	state: 1 light on, 0 light off
Output  		:	None
Function return :	None
*/
void Led_1(unsigned char state);







/*
Function name	:	Led_2
Description		:	Function drive PE15 Led 2
Input   		:	state: 1 light on, 0 light off
Output  		:	None
Function return :	None
*/
void Led_2(unsigned char state);







/*
Function name	:	Read_Tamp_Button
Description		:	Function return state of pin PC13
                    on MCBSTM32
Input   		:	None
Output  		:	None
Function return :	1 - switch on, 0 - switch off
*/
unsigned char Read_Tamp_Button(void);






/*
Function name	:	Read_Wkup_Button
Description		:	Function return state of pin PA0
                    on MCBSTM32
Input   		:	None
Output  		:	None
Function return :	1 - switch on, 0 - switch off
*/
unsigned char Read_Wkup_Button(void);




#endif
