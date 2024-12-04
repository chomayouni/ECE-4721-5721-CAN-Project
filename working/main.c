#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "TM4C123.h"
#include "C:\Keil_v5\Lab 2\inc/tm4c123gh6pm.h"
#include "lcdDriver.h" /* include LCD driver header file */
#include "delay.h" /* include delay functions header file */

#define DHT (3)  //PB3
#define MASK(x) (1UL<<(x))
//IO direction
#define DHT11_IO_IN() (GPIO_PORTB_DIR_R  &= ~MASK(DHT)) //PB3 input
#define DHT11_IO_OUT() (GPIO_PORTB_DIR_R |= MASK(DHT))	//PB3 output
//IO operation
#define DHT11_DQ_OUT(x) (x ?(GPIO_PORTB_DATA_R |= MASK(DHT)):(GPIO_PORTB_DATA_R &= ~MASK(DHT)))
#define DHT11_DQ_IN()   ((GPIO_PORTB_DATA_R & MASK(DHT))? 1:0)

// DHT11 function prototypes
uint8_t DHT11_Init(void);				//Init DHT11
uint8_t DHT11_Read_Data(void);	//read temp and humidity.
uint8_t DHT11_Read_Byte(void);	//read one byte.
uint8_t DHT11_Read_Bit(void);		//read one bit.
uint8_t DHT11_Check(void);			//check DHT11
void DHT11_Rst(void);						//reset DHT11.

// Delays that originally came with the
void delay_us_dht(int us);
void delay_ms_dht(int ms);

static unsigned long counter = 0;
static uint8_t temp, humi;
static unsigned int UpDown=0;  //PWM counter: 0 upward count; 1 downward count

/*PWM and Timer configuration*/
void Pwm_Init(void);
void Timer_Init(void);
void TIMER1A_Handler(void);




//Init DHT11 of IO ports and then DQ check if DHT11 exist
//return 1:no dht11
//return 0:exist         
uint8_t DHT11_Init(void)
{
    
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB;// active clock for PB.
    
    GPIO_PORTB_DIR_R |= (MASK(DHT)|MASK(DHT)); // enable as digital output
    GPIO_PORTB_DEN_R |=(MASK(DHT)|MASK(DHT));   
    
    
    GPIO_PORTB_DIR_R |= MASK(DHT);

    DHT11_Rst();
    
    return DHT11_Check();
}

//DHT11 start
void DHT11_Rst(void)       
{                 
    DHT11_IO_OUT();     //set as output
    DHT11_DQ_OUT(0);    //pull down DQ
    delay_ms_dht(20);       //pull down 18ms
    DHT11_DQ_OUT(1);    //DQ=1 
    delay_us_dht(25);       //20~40us
}

//dht11 response
//0--normal
//1--failed
uint8_t DHT11_Check(void)      
{   
    uint8_t retry=0;
    DHT11_IO_IN();//SET INPUT    
  while (DHT11_DQ_IN()&&retry<100)//DHT11 will pull down 40~80us
    {
        // check to see if the data line is high and if it is, the device hasnt pulled down properly yet
        retry++;
        delay_us_dht(1);
    }
    
    if(retry>=100)return 1; // device was outside of the timing specs, there was an error
    else retry=0;
   while (!DHT11_DQ_IN()&&retry<100)//DHT11 pull down and then pull up 40~80us
    {
        retry++;
        delay_us_dht(1);
    }
    if(retry>=100)return 1;     // device was outside of the timing specs, there was an error
    return 0; // start of data was good
}

//read one bit
//return 0/1
uint8_t DHT11_Read_Bit(void)             
{
    uint8_t retry=0;
    while(DHT11_DQ_IN()&&retry<100)// wait low voltage
    {
        retry++;
        delay_us_dht(1);
    }
    retry=0;
    while(!DHT11_DQ_IN()&&retry<100)// wait high voltage
    {
        retry++;
        delay_us_dht(1);
    }
    delay_us_dht(40);// wait 40us which is above the upper limit of a 0 bit
    if(DHT11_DQ_IN())return 1; 
    else return 0;         
}

//read one byte
//return byte from dht11
uint8_t DHT11_Read_Byte(void)    
{        
    uint8_t i,dat;
    dat=0;
    for (i=0;i<8;i++) 
    {
        dat<<=1; // shift and append the bits of data
        dat|=DHT11_Read_Bit();
    }                           
    return dat;
}

//read data from DHT11
//temp:(0-50)
//humi:(20%~90%)
uint8_t DHT11_Read_Data(void)    
{        
    uint8_t buf[5]; // buffer for RH1, RH2, T1, T2, Checksum
    uint8_t i; // counter
    DHT11_Rst(); // set signal to ask for data
    if(DHT11_Check()==0) // check if start of data was good
    {
        for(i=0;i<5;i++)//read 40 bits
        {
            buf[i]=DHT11_Read_Byte();
        }
        if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
        {
            humi=buf[0]; // pull data from the buffers if the checksum is passed
            temp=buf[2];
        }
    }else return 1;
    return 0;       
}


void Pwm_Init(void)
{
    SYSCTL->RCGCPWM |= (1<<1); //clock setting for pwm1 clock
    SYSCTL->RCGCGPIO |= (1<<5); //enable GPIOF clock
    
    GPIOF->AFSEL |= (1<<2); //select PF2 as alternative function pwm1
    GPIOF->PCTL |= (5<<8);  //set PF2 as pwm1 output
    GPIOF->DEN |= (1<<2);       //enable PF2 digital function
    
    PWM1->_3_CTL &= (unsigned)(~(1<<0)); // first disable pwm1 counter
    PWM1->_3_CTL &= (unsigned)(~(1<<1)); // select the downward counter
    PWM1->_3_LOAD = (1600); //set the pwm1 period 10KHz
    PWM1->_3_CMPA = (800); //set duty cycle 50%
    PWM1->_3_GENA |= (3<<2); //set pwm1 high or low voltage, when matches PWMCMPA
    PWM1->_3_GENA   |= (2<<6); //set PWM1 high or low voltage depending on the direction of counter
    PWM1->_3_CTL |= (1<<0); //enable pwm1 counter
    PWM1->ENABLE |= (1<<6); //enable pwm1 channel
    
    
}

void Timer_Init(void)
{
    //enbale timer clock (timer 1A)
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
    
    //clear timeort flag
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
    
    if(TIMER1->MIS & 0x01)
    {
        
        PWM1->_3_CMPA = counter;

        switch (UpDown)
        {
            case 0:
                counter++;
                if(counter>=1600)
                    UpDown = 1;
            break;
            
            case 1:
                counter--;
                if(counter<=0)
                    UpDown = 0;
        }

    }
    
    TIMER1->ICR |= (1<<0);  //clear timeout flag

}

void Uart_Init(void) //UART0 -> PA0(RX) PA1(TX) 9600
{
    SYSCTL->RCGCUART |= (1<<0); //enable the uart0 clock
    SYSCTL->RCGCGPIO |= (1<<0); //enable the GPIOA clock
    
    GPIOA->AFSEL |= (1<<0)|(1<<1); //set alternative function of PA[0, 1] as uart0 pins
    GPIOA->PCTL |= (1<<0)|(1<<4);  //set uart mode
    GPIOA->DEN |= (1<<0)|(1<<1); //digitize the GPIO pins of uart0
    
    UART0->CTL &= ((unsigned)(~(1<<0))) & ((unsigned)(~(1<<8))) & ((unsigned)(~(1<<9))); //disable uart0, TX and RX
    UART0->IBRD = 104;  //set baud rate in IBRD 
    UART0->FBRD = 11;   //set baud rate in FBRD 
    UART0->LCRH |= (3<<5); //set data length in the transmission
    UART0->CC = 0x05; //select clock source
    UART0->CTL |= (1<<0)|(1<<8)|(1<<9); //enable uart0, TX and RX
    
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
    while((UART0->FR    & (1<<5))!=0){}     
    return (ch);
}

void delay_us_dht( int delay)  
{ 
		delay = delay*1.6; //edited to match with oscilloscope results
    while(delay--){
        __asm("nop"); // almost 1us
    }
    
}


void delay_ms_dht(int ms) 
{
    for(int i=0; i<ms; i++)
    {
        delay_us_dht(1000); 
    }
}

int main()
{
    char temp_str[16];    // String buffer for temperature
    char humi_str[16];    // String buffer for humidity
    int read_number = 0;
    
    // Initialize LCD and DHT11
    LCD4bits_Init();
		Timer_Init();
    Pwm_Init();
    while(DHT11_Init()){}
    delay_ms_dht(400); // Wait for DHT11 stability
	
		LCD4bits_Cmd(clear_display);
		LCD4bits_Cmd(FirstRow);
		LCD_WriteString("Initializing..");
    delay_ms(2000);
    
    while(1) {
        LCD4bits_Cmd(clear_display);
        
        // Read DHT11 sensor
        while(DHT11_Read_Data()) { 
            delay_ms_dht(5);
            if(read_number > 250) {
								PWM1->ENABLE &= (unsigned)(~(1<<6)); //disable pwm1 channel
								
                LCD4bits_Cmd(clear_display);
                LCD_WriteString("Sensor Error!");
                delay_ms_dht(2000);
                read_number = 0;
                //continue;
            }
            read_number++;
        }
        
        // Format temperature and humidity strings
        sprintf(temp_str, "Temp:%c%d", 0xDF, temp);
				
        sprintf(humi_str, "Humi:%d%%",humi);
        
        // Display temperature and humidity
        LCD4bits_Cmd(clear_display);
        LCD4bits_Cmd(FirstRow);
        LCD_WriteString(temp_str);
				LCD4bits_Cmd(SecondRow); // the second row function is not written
				LCD_WriteString(humi_str);
        
        delay_ms_dht(2000);  // Display for 2 seconds
        read_number = 0;
    }
}