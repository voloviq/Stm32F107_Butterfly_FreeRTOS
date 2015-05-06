/*
	Description:
	Module is use to convert value to string etc.

	Compiler: Gnu Gcc

	Language: Ansi C

	Ide: Eclipse

	Author: Michal Wolowik

	Date: Mielec december 2008r.
*/






#ifndef _STRINGSCONV_H
#define _STRINGSCONV_H






#define INFO_STRING_SIZE 0x30






// Imitation Structure of float variable
typedef struct
{
	unsigned int int_part;
	unsigned int rest_part;
}_float_s;






// Size of internal buffer in module
#define INTERNAL_BUFF_SIZE 20






/*
Function Name	:	Convert_2_String
Description		:	Function convert value to string and glue
					created string with constant string and
					directed string to output string


Input 1			:	*string_1: constant string for example "Temp=" start sting
Input 2			:	*string_2: constant string for example "C" - end string
Input 3			:	*string_out: - common output string
Input 4			:	value: - liczba zmiennoprzecinkowa 25.03
Input 5			:	type: - type of value to converted type 0 - dec, 1 -float, 2 - hex

Output			:	None
Return			:	None
*/
void Convert_2_String	(
							const char *string_1,
							const char *string_2,
							char *string_out,
							float value,
							unsigned char type
						);






/*
Function Name	:	Convert_Float_2_Two_Int_Value
Description		:	Function convert one float variable to two
					unsigned int variable part_int and rest_part

Input 			:	float: float variable

Output			:	None
Return 			:	_float_s: structure with converted value
*/
_float_s Convert_Float_2_Two_Int_Value(float value);







/*
Function Name	:	_Sprint_f
Description		:	Function similar to sprintf ANSI C function

Input 1			:	*buffer: storage buffer
Input 2			:	val_type: 0 - dec, 1 -float, 2 - hex
Input 3			:	_value: structure of oryginal value

Output			:	None
Return 			:	unsigned int: string length
*/
unsigned int _Sprint_f(char *buffer, unsigned char val_type, _float_s _value);







/*
Function Name	:	Convert_Int_2_String
Description		:	Function convert one int value to string

Input 1			:	value: integer value
Input 2			:	*out_str: storage array

Output			:	None
Return 			:	unsigned int: lenght of created string
*/
unsigned int Convert_Int_2_String(unsigned int value, char *out_str);







/*
Function Name	:	String_Width
Description		:	Function calculated how many characters have
					input string

Input 			:	*string_1: input string

Output			:	None
Return 			:	unsigned int: number of characters in string
*/
unsigned int String_Width(const char *string);






/*
Function Name	:	Dec_2_Hex
Description		:	Function convert value from dec to hex

Input 1			:	value: dec value
Input 2			:	*out_buff: storage array

Output			:	None
Return 			:	unsigned char: number of activ character in hex value
*/
unsigned char Dec_2_Hex(unsigned int value, char *out_buff);







char* Connect_2String_With_Value	(
									char *string_1,
									unsigned int value,
									char *string_2
								);



#endif
