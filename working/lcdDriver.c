#include "lcdDriver.h"
#include "delay.h"

void LCD_init(void)
{
    SYSCTL->RCGCGPIO |=(1<<1); /* Enable Clock to GPIOB */
    LCD->DIR |=0xFF; /* Set GPIOB all pins a digital output pins */
    LCD->DEN |=0xFF; /* Declare GPIOB pins as digital pins */

    LCD_Cmd(Set5x7FontSize);  /* select 5x7 font size and 2 rows of LCD */
    LCD_Cmd(Function_set_4bit); /* Select 4-bit Mode of LCD */
    LCD_Cmd(moveCursorRight); /* shift cursor right */
    LCD_Cmd(clear_display); /* clear whatever is written on display */
    LCD_Cmd(cursorOff);  /* Enable Display and cursor blinking */
}

void LCD_Cmd(unsigned char command)
{
    LCD_Write_Nibble(command & 0xF0, 0);   /* Write upper nibble to LCD */
    LCD_Write_Nibble(command << 4, 0);     /* Write lower nibble to LCD */
    
    if (command < 4)
        delay_ms(2);         /* 2ms delay for commands 1 and 2 */
    else
        delay_us(40);        /* 40us delay for other commands */
}

void LCD4bits_Init(void)
{
    SYSCTL->RCGCGPIO |= 0X02;
    delay_ms(10);
    LCD->DIR |=0xFF; /* Set GPIOB all pins a digital output pins */
    LCD->DEN |=0xFF; /* Declare GPIOB pins as digital pins */
    LCD4bits_Cmd(Set5x7FontSize);  /* select 5x7 font size and 2 rows of LCD */
    LCD4bits_Cmd(moveCursorRight); /* shift cursor right */
    LCD4bits_Cmd(clear_display); /* clear whatever is written on display */
    LCD4bits_Cmd(cursorOff);  /* Enable Display and cursor blinking */
}

void LCD4bits_Cmd(unsigned char command)
{
    LCD_Write_Nibble(command & 0xF0, 0);   /* Write upper nibble to LCD */
    LCD_Write_Nibble(command << 4, 0);     /* Write lower nibble to LCD */
    
    if (command < 4)
        delay_ms(2);         /* 2ms delay for commands 1 and 2 */
    else
        delay_us(40);        /* 40us delay for other commands */
}

void LCD_Write_Nibble(unsigned char data, unsigned char control)
{
    data &= 0xF0;       /* Extract upper nibble for data */
    control &= 0x0F;    /* Extract lower nibble for control */
    LCD->DATA = data | control;       /* Set RS and R/W to zero for write operation */
    LCD->DATA = data | control | EN;  /* Provide Pulse to Enable pin to perform write operation */
    delay_us(0);
    LCD->DATA = data; /*Send data */
    LCD->DATA = 0; /* stop writing data to LCD */
}

void LCD_Write4bits(unsigned char data, unsigned char control)
{
    data &= 0xF0;       /* Extract upper nibble for data */
    control &= 0x0F;    /* Extract lower nibble for control */
    LCD->DATA = data | control;       /* Set RS and R/W to zero for write operation */
    LCD->DATA = data | control | EN;  /* Provide Pulse to Enable pin to perform write operation */
    delay_us(0);
    LCD->DATA = data; /*Send data */
    LCD->DATA = 0; /* stop writing data to LCD */
}

void LCD_Write_Char(unsigned char data)
{
    LCD_Write_Nibble(data & 0xF0, RS);    /* Write upper nibble to LCD and RS = 1 to write data */
    LCD_Write_Nibble(data << 4, RS);      /* Write lower nibble to LCD and RS = 1 to write data */
    delay_us(40);
}

void LCD_String (char *str)	/* Send string to LCD function */
{
    int i;
    for(i=0;str[i]!=0;i++)  /* Send each char of string till the NULL */
    {
        LCD_Write_Char(str[i]);  /* Call LCD data write */
    }
}

void LCD_WriteString (char *str)	/* Send string to LCD function */
{
    volatile int i = 0;
    while(*(str+i) != '\0'){
        LCD4bits_Data(*(str+i));
        i++;
    }
}

void LCD4bits_Data(unsigned char data)
{
    LCD_Write4bits(data & 0xF0, RS);
    LCD_Write4bits(data << 4, RS);
    delay_us(40);
}