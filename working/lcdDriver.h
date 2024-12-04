
#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include "TM4C123.h"

#define LCD GPIOB   /* Define "LCD" as a symbolic name for GPIOB */
#define RS 0x01 /* PORTB BIT5 mask */
#define RW 0x02 /* PORTB BIT6 mask */
#define EN 0x04 /* PORTB BIT7 mask */
#define HIGH 1
#define LOW 0

/*define useful symbolic names for LCD commands */
#define clear_display     0x01
#define returnHome        0x02
#define moveCursorRight   0x06
#define moveCursorLeft    0x08
#define shiftDisplayRight 0x1C
#define shiftDisplayLeft  0x18
#define cursorBlink       0x0F
#define cursorOff         0x0C
#define cursorOn          0x0E
#define Function_set_4bit 0x28
#define Function_set_8bit 0x38
#define Entry_mode        0x06
#define Function_8_bit    0x32
#define Set5x7FontSize    0x28
#define FirstRow          0x80
#define SecondRow         0xC0

/* prototypes of LCD functions */
void LCD_init(void);
void LCD_Cmd(unsigned char command);
void LCD_Write_Char(unsigned char data);
void LCD_Write_Nibble(unsigned char data, unsigned char control);
void LCD_String (char *str);
void LCD4bits_Data(unsigned char data);
void LCD4bits_Init(void);
void LCD4bits_Cmd(unsigned char command);
void LCD_WriteString (char *str);

#endif