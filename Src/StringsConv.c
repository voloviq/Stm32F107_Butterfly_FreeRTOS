/*
	Description:
	Module is use to convert value to string etc.

	Compiler: Gnu Gcc

	Language: Ansi C

	Ide: Eclipse

	Author: Michal Wolowik

	Date: Mielec december 2008r.
*/






#include "StringsConv.h"






char str_tab[INFO_STRING_SIZE];






void Convert_2_String	(
							const char *string_1,
							const char *string_2,
							char *string_out,
							float value,
							unsigned char type
						)
{

	unsigned int size_string = 0;
	unsigned int i = 0, j = 0;
	char buffer[INTERNAL_BUFF_SIZE];

	_float_s _float_val;

	_float_val.int_part = 0;
	_float_val.rest_part = 0;


	// Clear display output sting buffer
	for(i=0; i<INFO_STRING_SIZE; i++)
		string_out[i] = 0;


	// Clear internal usefull buffer
	for(i=0; i<INTERNAL_BUFF_SIZE; i++)
        buffer[i] = 0;


	// Calculate number of characters in constant string
	size_string = String_Width(string_1);


    // Copy constant string (start) to output string
	for(i=0; i<size_string; i++)
		if(i < INFO_STRING_SIZE)
			string_out[i] = string_1[i];


	// Calculate number of characters in converter value to string
    if(type == 0)
    {
    	_float_val = Convert_Float_2_Two_Int_Value(value);
    	size_string = _Sprint_f(buffer, 0, _float_val);
    }
    else if(type == 1)
    {
    	_float_val = Convert_Float_2_Two_Int_Value(value);
    	size_string = _Sprint_f(buffer, 1, _float_val);
    }
    else if(type == 2)
    {
    	_float_val = Convert_Float_2_Two_Int_Value(value);
    	size_string = _Sprint_f(buffer, 2, _float_val);
    }



	// Copy convertered string from value
	for(j=0; j<(size_string); j++)
		if( ( (j+i) < INFO_STRING_SIZE) && (j<INTERNAL_BUFF_SIZE) )
			string_out[j+i] = buffer[j];

	i += j;

	// Calculate number of characters in constant string
	size_string = String_Width(string_2);


    // Copy constant string (end) to output string
	for(j=0; j<size_string; j++)
		if(i < INFO_STRING_SIZE)
			string_out[i+j] = string_2[j];


}






_float_s Convert_Float_2_Two_Int_Value(float value)
{
	_float_s _flt_val;

	_flt_val.int_part = 0;
	_flt_val.rest_part = 0;

	_flt_val.int_part = (unsigned int)value;

	value -= _flt_val.int_part;

	// Setup precision for this moment is 3 places
	_flt_val.rest_part = (unsigned int)(value * 1000);

	return _flt_val;
}






unsigned int _Sprint_f(char *buffer, unsigned char val_type, _float_s _value)
{
	unsigned int str_len = 0;

	if(val_type == 0)
	{
		// Decimal
		str_len = Convert_Int_2_String(_value.int_part, buffer);

	}
	else if(val_type == 1)
	{
		// Float
		str_len = Convert_Int_2_String(_value.int_part, buffer);
		buffer[str_len] = ',';
		str_len++;
		str_len += Convert_Int_2_String(_value.rest_part, buffer+str_len);
	}
	else if(val_type == 2)
	{
		// Hex
		buffer[str_len++] = '0';
		buffer[str_len++] = 'x';

		str_len += Dec_2_Hex(_value.int_part, buffer);

	}

	return str_len;

}






unsigned int Convert_Int_2_String(unsigned int value, char *out_str)
{
	volatile unsigned int i = 0, j = 0;
	unsigned int temp = 0;
	unsigned int tab_value[7] = {1000000, 100000, 10000, 1000, 100, 10};
	unsigned int znaleziono = 0;

	unsigned int string_lenght = 0;

	// Clear string
	for(j=0; j<INTERNAL_BUFF_SIZE; j++)
		out_str[i] = 0;


	for(j=0; j<6; j++)
	{
		temp = value/(tab_value[j]);

		if(temp < 10)
		{
			if(znaleziono == 0 && temp > 0)
				znaleziono = 1;

			if(znaleziono == 1)
			{
				value -= (temp*tab_value[j]);

				if(i<INTERNAL_BUFF_SIZE)
					out_str[i++] += temp+0x30;
				else
					break;
			}
		}
	}

	temp = value%10;

	string_lenght = i+1;

	if(i<INTERNAL_BUFF_SIZE)
		out_str[i] += temp+0x30;

	return(string_lenght);
}






unsigned int String_Width(const char *string)
{
	unsigned int idx_of_string = 0x00;

	do
	{
		idx_of_string++;
	}while(string[idx_of_string]!='\0');

	return idx_of_string;
}






unsigned char Dec_2_Hex(unsigned int value, char *out_buff)
{
	unsigned char tmp = 0;
	unsigned char i = 0, j = 2;
	unsigned char sign_zero = 0;

	for(i=0; i<4; i++)
	{
		tmp = value >> (8*(3-i));

		if(j<INTERNAL_BUFF_SIZE-1)
		{
			if( (tmp/16) != 0 )
			{
				sign_zero = 1;
			}


			if((tmp/16) < 10)
			{
				if( sign_zero == 1)
					out_buff[j++] = (tmp/16) + 0x30;
			}
			else
				out_buff[j++] = (tmp/16) + 0x37;

			if( (tmp%16) != 0 )
			{
				sign_zero = 1;
			}

			if((tmp%16) < 10)
			{
				if( sign_zero == 1)
					out_buff[j++] = (tmp%16) + 0x30;
			}
			else
				out_buff[j++] = (tmp%16) + 0x37;


		}
	}

	return j;

}






char* Connect_2String_With_Value	(
									char *string_1,
									unsigned int value,
									char *string_2
								)
{
	unsigned int i = 0, j = 0;
	unsigned int temp = 0;
	unsigned long tab_value[7] = {1000000, 100000, 10000, 1000, 100, 10};
	unsigned int znaleziono = 0;


	// Clear string information
	for(i=0; i<INTERNAL_BUFF_SIZE-1; i++)
		str_tab[i] = 0;

	// Clear i variable
	i = 0;


	// Copy input string to string
	while(*string_1)
	{

		// Execute until reached maximum size of string
		if(i<INTERNAL_BUFF_SIZE-1)
			str_tab[i++] = *string_1;
		else
			break;
		string_1++;

	}

	for(j=0; j<6; j++)
	{
		temp = value/(tab_value[j]);

		if(temp < 10)
		{
			if(znaleziono == 0 && temp > 0)
				znaleziono = 1;

			if(znaleziono == 1)
			{
				value -= (temp*tab_value[j]);

				if(i<INTERNAL_BUFF_SIZE-1)
					str_tab[i++] += temp+0x30;
				else
					break;
			}
		}
	}

	temp = value%10;

	if(i<INTERNAL_BUFF_SIZE-1)
		str_tab[i++] += temp+0x30;
/*

	// Copy input string_2 to string
	while(*string_2)
	{

		// Execute until reached maximum size of string
		if(i<STRING_SIZE)
			string_comunicat[i++] = *string_2;
		else
			break;
		string_2++;
	}
*/
	// Execute until reached maximum size of string
	for(i=i; i<INTERNAL_BUFF_SIZE-1; i++)
		str_tab[i] = 0x20; // space char

	// This is necessary to end of actual string
	str_tab[i-1] = '\0'; // end of string


	return(str_tab);
}


