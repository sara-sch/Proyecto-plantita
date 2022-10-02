/* 
 * File: LCD.h  
 * Se utilizó y se adaptaron las librerías de Ligo George 
 * de la página www.electrosome.com
 * Enlace: https://electrosome.com/lcd-pic-mplab-xc8/
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef LCD_H
#define	LCD_H

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 4000000
#endif

#ifndef RS
#define RS RD7
#endif

#ifndef EN
#define EN RD6
#endif

#ifndef B0
#define B0 RB0
#endif

#ifndef B1
#define B1 RB1
#endif

#ifndef B2
#define B2 RB2
#endif

#ifndef B3
#define B3 RB3
#endif

#ifndef B4
#define B4 RB4
#endif

#ifndef B5
#define B5 RB5
#endif

#ifndef B6
#define B6 RB6
#endif

#ifndef B7
#define B7 RB7
#endif

#include <xc.h> // include processor files - each processor file is guarded.  

//LCD Functions Developed by electroSome


// 8bits

void Lcd_Cmd(char a);

void Lcd_Port(char a);

void Lcd_Init(void);

void Lcd_Clear(void);

void Lcd_Set_Cursor(char a, char b);

void Lcd_Write_Char(char a);

void Lcd_Shift_Right(void);

void Lcd_Shift_Left(void);

void Lcd_Write_String(char *a);


#endif	/* LCD_H */

