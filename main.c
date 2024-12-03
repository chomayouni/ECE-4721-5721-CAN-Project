#include "TM4C123.h" /* include register defintion file of TM4C123GH6PM */
#include "lcdDriver.h" /* include LCD driver header file */
#include "delay.h" /* include delay functions header file */

int main()
{
    char* str1 = "Hello World";
    char* str2 = "The updated LCD code is working :)";
    
    LCD4bits_Init();
    
    while (1) {
        LCD4bits_Cmd(clear_display);
        LCD4bits_Cmd(FirstRow);
        delay_ms(500);
        LCD_WriteString(str1);
        delay_ms(4000);
        
        LCD4bits_Cmd(clear_display);
        LCD4bits_Cmd(FirstRow);
        delay_ms(500);
        LCD_WriteString(str2);
        delay_ms(4000);
    }
}