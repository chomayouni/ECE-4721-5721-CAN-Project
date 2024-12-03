#include "TM4C123.h" /* include register defintion file of TM4C123GH6PM */
#include "lcdDriver.h" /* include LCD driver header file */
#include "delay.h" /* include delay functions header file */
#include "canDriver.h" /* include CAN driver header file */

int main()
{
    char* str1 = "Hello World";
    char* str2 = "The updated LCD code is working :)";
    
    // Initialize LCD
    LCD4bits_Init();
    
    // Initialize CAN
    CAN0_Init();

	CAN_RX_init();
	uint8_t Current = 0;
	uint8_t temp = 'A';
    
    while (1) {
        // Display first message on LCD
        LCD4bits_Cmd(clear_display);
        LCD4bits_Cmd(FirstRow);
        delay_ms(500);
        LCD_WriteString(str1);
        delay_ms(4000);
        
        // Display second message on LCD
        LCD4bits_Cmd(clear_display);
        LCD4bits_Cmd(FirstRow);
        delay_ms(500);
        LCD_WriteString(str2);
        delay_ms(4000);
        
        // Add the CAN communication logic here
        // Example: CAN_SendMessage("CAN message");
		CAN_TX(temp);
		if(CAN0_IF2MCTL_R&CAN_IF2MCTL_NEWDAT){
			Current = CAN_RX();
		}
    }
}