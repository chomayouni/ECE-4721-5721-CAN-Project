#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include "TM4C123.h"

// LCD definitions
#define LCD GPIOB
#define RS 0x20
#define RW 0x40
#define EN 0x80

// LCD commands
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
#define Set5x7FontSize    0x20
#define FirstRow          0x80

// Function prototypes
void LCD_init(void);
void LCD_Cmd(unsigned char command);
void LCD_Write_Char(unsigned char data);
void LCD_Write_Nibble(unsigned char data, unsigned char control);
void LCD_String(char *str);

#endif // LCD_DRIVER_H