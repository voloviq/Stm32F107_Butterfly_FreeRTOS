/*
 * lcd3310.h
 *
 *  Created on: 2010-01-09
 *      Author: X
 */

#ifndef LCD3310_H_
#define LCD3310_H_





//definicja linii portu
#define CLK GPIO_Pin_4  //CLK PC4
#define DATA GPIO_Pin_5 //DATA PC5
#define DC GPIO_Pin_6   //DC PC6
#define RES GPIO_Pin_7  //RES PC7
#define SCE GPIO_Pin_11  //SCE PC11






void Delay(int miliSec);
void SetBit(int bits);
void ClrBit(int bits);
void WriteSPI(unsigned char data);
void LCD_WriteData(unsigned char data);
void LCD_WriteCmd(unsigned char cmd);
void LCD_Init(void);
void LCD_GotoXY ( unsigned char x, unsigned char y );
void LCD_Clear ( void );
void LCD_WriteChar (unsigned char ch, unsigned char col, unsigned char raw);



#endif /* LCD3310_H_ */
