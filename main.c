#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "TM4C123.h"
#include "C:\Keil_v5\Lab 2\inc/tm4c123gh6pm.h"
#include "main.h"
#include "lcdDriver.h"
#include "canDriver.h"
#include "dht11Driver.h"

#define DHT (0)  //PB0 is connected to DHT11 sensor
#define MASK(x) (1UL<<(x))

//IO direction
#define DHT11_IO_IN() (GPIO_PORTB_DIR_R  &= ~MASK(DHT)) //PB0 input
#define DHT11_IO_OUT() (GPIO_PORTB_DIR_R |= MASK(DHT)) //PB0 output

//IO operation
#define DHT11_DQ_OUT(x) (x ?(GPIO_PORTB_DATA_R |= MASK(DHT)):(GPIO_PORTB_DATA_R &= ~MASK(DHT)))
#define DHT11_DQ_IN()   ((GPIO_PORTB_DATA_R & MASK(DHT))? 1:0)

static uint8_t temp, humi;

static unsigned long counter = 0;
static unsigned int UpDown=0;  //PWM counter: 0 upward count; 1 downward count

/*PWM and Timer configuration*/
void Pwm_Init(void);
void Timer_Init(void);
void TIMER1A_Handler(void);

/*UART configuration (optional)*/
void Uart_Init(void);
void uart_send_data(uint16_t Data);

/* CAN configuration */
void CAN0_Init(void);
void CAN0_Transmit_Set(int objNo);
void CAN0_Receive_Set(int objNo);
void CAN0_Transmit_Data(int objNo, char data[8]);
char* CAN0_Receive_Data(int objNo);

bool isMaster = true; // Set this to true for master, false for slave

int main(void)
{
	int read_number = 0;
	//Uart_Init();
	Timer_Init();
	Pwm_Init();
	//CAN0_Init();

	/*
	if (isMaster) {
		if (DHT11_Init()) {
			printf("DHT11 initialization failed!\n");
			return 1;
		}
	}
	*/

	LCD_init();
	LCD_Cmd(clear_display);
	LCD_Cmd(FirstRow); /* Force cursor to beginning of first row */
	delay_ms(500);

	while (1)
	{
		LCD_String("LCD is working");
		delay_ms(3000); // Wait for 3 seconds

		LCD_Cmd(clear_display);
		LCD_Cmd(FirstRow); /* Force cursor to beginning of first row */
		LCD_String("Hello World");

		delay_ms(3000); // Wait for 3 seconds

		/*
		if (isMaster) {
			while (DHT11_Read_Data())
			{
				delay_ms(5);
				if (read_number > 250)
				{
					printf("DHT11 does not work, please check and restart the system!\n");
					PWM1->ENABLE &= (unsigned)(~(1 << 6)); //disable pwm1 channel
				}
				read_number++;
			}
			printf("Temp. is %d\n", temp);
			printf("Humidity is %d\n", humi);

			char data[8];
			data[0] = temp;
			data[1] = humi;
			CAN0_Transmit_Data(1, data);

			read_number = 0;
		} else {
			char* receivedData = CAN0_Receive_Data(1);
			temp = receivedData[0];
			humi = receivedData[1];

			LCD_Cmd(clear_display);
			LCD_Cmd(FirstRow);
			char buffer[16];
			sprintf(buffer, "Temp: %d", temp);
			LCD_String(buffer);
			LCD_Cmd(FirstRow + 0x40); // Move to second line
			sprintf(buffer, "Humi: %d", humi);
			LCD_String(buffer);

			delay_ms(1000);
		}
		*/
	}
}

void Pwm_Init(void)
{
	SYSCTL->RCGCPWM |= (1<<1); //clock setting for pwm1 clock
	SYSCTL->RCGCGPIO |= (1<<5); //enable GPIOF clock
	
	GPIOF->AFSEL |= (1<<2); //select PF2 as alternative function pwm1
	GPIOF->PCTL |= (5<<8);  //set PF2 as pwm1 output
	GPIOF->DEN |= (1<<2);		//enable PF2 digital function
	
	PWM1->_3_CTL &= (unsigned)(~(1<<0)); // first disable pwm1 counter
	PWM1->_3_CTL &= (unsigned)(~(1<<1)); // select the downward counter
	PWM1->_3_LOAD = (1600); //set the pwm1 period 10KHz
	PWM1->_3_CMPA = (800); //set duty cycle 50%
	PWM1->_3_GENA |= (3<<2); //set pwm1 high or low voltage, when matches PWMCMPA
	PWM1->_3_GENA	|= (2<<6); //set PWM1 high or low voltage depending on the direction of counter
	PWM1->_3_CTL |= (1<<0); //enable pwm1 counter
	PWM1->ENABLE |=	(1<<6); //enable pwm1 channel
}

void Timer_Init(void)
{
	//enable timer clock (timer 1A)
	SYSCTL->RCGCTIMER |= (1<<1);
	
	//disable the timer before starting
	TIMER1->CTL &= (unsigned)(~(1<<0));
	
	//select timer resolution (16 or 32 bits)
	TIMER1->CFG = 0x04; //16 bits
	
	//select timer mode (one shot or periodic or up or down counter)
	TIMER1->TAMR = 0x02; //periodic mode
	TIMER1->TAMR &= (unsigned)(~(1<<4)); //downward counter
	
	//set interval load register value
	TIMER1->TAILR = 16000;  //period 1ms
	
	//clear timeout flag
	TIMER1->ICR |= (1<<0); 
	
	//enable the timer 1A
	TIMER1->CTL |= (1<<0);
	
	//enable timer 1A interrupt
	TIMER1->IMR |= (1<<0); 
	
	//enable nvic about timer 1A
	NVIC->ISER[0] |= (1<<21); //timer 1A 
}

void TIMER1A_Handler(void)
{ 
	if (TIMER1->MIS & 0x01)
	{
		PWM1->_3_CMPA = counter;

		switch (UpDown)
		{
			case 0:
				counter++;
				if (counter >= 1600)
					UpDown = 1;
				break;
			
			case 1:
				counter--;
				if (counter <= 0)
					UpDown = 0;
		}
	}
	
	TIMER1->ICR |= (1<<0);  //clear timeout flag
}

void Uart_Init(void) //UART0 -> PA0(RX) PA1(TX) 9600
{
	SYSCTL->RCGCUART |= (1<<0); //enable the uart0 clock
	SYSCTL->RCGCGPIO |= (1<<0); //enable the GPIOA clock
	
	GPIOA->AFSEL |= (1<<0) | (1<<1); //set alternative function of PA[0, 1] as uart0 pins
	GPIOA->PCTL |= (1<<0) | (1<<4);  //set uart mode
	GPIOA->DEN |= (1<<0) | (1<<1); //digitize the GPIO pins of uart0
	
	UART0->CTL &= ((unsigned)(~(1<<0))) & ((unsigned)(~(1<<8))) & ((unsigned)(~(1<<9))); //disable uart0, TX and RX
	UART0->IBRD = 104;  //set baud rate in IBRD 
	UART0->FBRD = 11; 	//set baud rate in FBRD 
	UART0->LCRH |= (3<<5); //set data length in the transmission
	UART0->CC = 0x05; //select clock source
	UART0->CTL |= (1<<0) | (1<<8) | (1<<9); //enable uart0, TX and RX
}

void uart_send_data(uint16_t Data)
{
	/* Transmit Data */
	UART0->DR = (Data & (uint16_t)0x00FF);
}

//rewrite function for using printf function
int fputc(int ch, FILE *f)
{
	uart_send_data((uint8_t)ch);
	while ((UART0->FR & (1<<5)) != 0) {}		
	return (ch);
}

void delay_us(int delay)  
{ 
	while (delay--) {
		__asm("nop"); // almost 1us
	}
}

void delay_ms(int ms) 
{
	for (int i = 0; i < ms; i++) {
		delay_us(1000); 
	}
}