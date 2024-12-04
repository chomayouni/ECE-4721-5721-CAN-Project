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
#define DHT11_IO_IN() (GPIO_PORTB_DIR_R  &= ~MASK(DHT))
#define DHT11_IO_OUT() (GPIO_PORTB_DIR_R |= MASK(DHT))

//IO operation
#define DHT11_DQ_OUT(x) (x ?(GPIO_PORTB_DATA_R |= MASK(DHT)):(GPIO_PORTB_DATA_R &= ~MASK(DHT)))
#define DHT11_DQ_IN()   ((GPIO_PORTB_DATA_R & MASK(DHT))? 1:0)

// DHT11 function prototypes
uint8_t DHT11_Init(void);
uint8_t DHT11_Read_Data(void);
uint8_t DHT11_Read_Byte(void);
uint8_t DHT11_Read_Bit(void);
uint8_t DHT11_Check(void);
void DHT11_Rst(void);

static unsigned long counter = 0;
static unsigned int UpDown=0;  //PWM counter: 0 upward count; 1 downward count

static uint8_t temp, humi;

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
    delay_ms(20);       //pull down 18ms
    DHT11_DQ_OUT(1);    //DQ=1 
    delay_us(25);       //20~40us
}

//dht11 response
//0--normal
//1--failed
uint8_t DHT11_Check(void)      
{   
    uint8_t retry=0;
    DHT11_IO_IN();//SET INPUT    
    while (DHT11_DQ_IN()&&retry<100)
    {
        retry++;
        delay_us(1);
    }
    
    if(retry>=100) {
        LCD4bits_Cmd(clear_display);
        LCD_WriteString("Init Err: Low");  // Failed to go low
        return 1;
    }
    retry=0;
    while (!DHT11_DQ_IN()&&retry<100)
    {
        retry++;
        delay_us(1);
    }
    if(retry>=100) {
        LCD4bits_Cmd(clear_display);
        LCD_WriteString("Init Err: High"); // Failed to go high
        return 1;
    }
    return 0;
}

//read one bit
//return 0/1
uint8_t DHT11_Read_Bit(void)             
{
    uint8_t retry=0;
    while(DHT11_DQ_IN()&&retry<100)// wait low voltage
    {
        retry++;
        delay_us(1);
    }
    retry=0;
    while(!DHT11_DQ_IN()&&retry<100)// wait high voltage
    {
        retry++;
        delay_us(1);
    }
    delay_us(40);// wait 40us which is above the upper limit of a 0 bit
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

// Add this function before main()
void intToStr(uint8_t num, char *str) {
    str[0] = (num/10) + '0';  // Convert tens digit to ASCII
    str[1] = (num%10) + '0';  // Convert ones digit to ASCII
    str[2] = '\0';            // Null terminate
}

int main()
{
    char temp_str[16];    
    char humi_str[16];
    char num_buffer[3];    
    int read_number = 0;
    bool valid_reading = false;
    int init_failures = 0;  // Track initialization failures
    
    // Initialize LCD and DHT11
    LCD4bits_Init();
   
    
    while(1) {

         if(DHT11_Init()) {  // Check if sensor initialized properly
            LCD4bits_Cmd(clear_display);
            LCD_WriteString("Init Failed:");
            init_failures++;
            intToStr(init_failures, num_buffer);
            LCD4bits_Cmd(SecondRow);
            LCD_WriteString(num_buffer);
            delay_ms(2000);
        }
        delay_ms(400);


        LCD4bits_Cmd(clear_display);
        LCD4bits_Cmd(FirstRow);
        LCD_WriteString("Reading Sensor..");
        
        valid_reading = false;
        read_number = 0;
        
        // Try to get valid reading
        while(!valid_reading && read_number < 3) {
            if(DHT11_Read_Data() == 0) {  // 0 means successful read
                if(temp != 0 && humi != 0) {  // Check for non-zero values
                    valid_reading = true;
                }
            }
            read_number++;
            delay_ms(1000);
        }
        
        if(!valid_reading) {
            LCD4bits_Cmd(clear_display);
            LCD_WriteString("Sensor Error!");
            delay_ms(2000);
            continue;
        }
        
        // Display valid readings
        LCD4bits_Cmd(clear_display);
        LCD4bits_Cmd(FirstRow);
        LCD_WriteString("Temp: ");
        intToStr(temp, num_buffer);
        LCD_WriteString(num_buffer);
        LCD_WriteString(" C");
        
        delay_ms(2000);
        
        LCD4bits_Cmd(clear_display);
        LCD4bits_Cmd(FirstRow);
        LCD_WriteString("Humi: ");
        intToStr(humi, num_buffer);
        LCD_WriteString(num_buffer);
        LCD_WriteString("%");
        
        delay_ms(2000);
    }
}