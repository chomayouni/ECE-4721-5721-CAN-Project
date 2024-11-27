/*
	
	
*/

#include <stdio.h>
#include <stdint.h>

#include "TM4C123.h"
#include "C:\Keil_v5\Lab 2\inc/tm4c123gh6pm.h"

#define LCD GPIOB   /* Define "LCD" as a symbolic name for GPIOB */
#define RS 0x20 /* PORTB BIT5 mask */
#define RW 0x40 /* PORTB BIT6 mask */
#define EN 0x80 /* PORTB BIT7 mask */
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
#define Set5x7FontSize    0x20
#define FirstRow          0x80

/* prototypes of LCD functions */
void delay_ms(int n); /* mili second delay function */
void delay_us(int n); /* micro second delay function */ 
void LCD_init(void);  /* LCD initialization function */
void LCD_Cmd(unsigned char command); /*Used to send commands to LCD */
void LCD_Write_Char(unsigned char data); /* Writes ASCII character */
void LCD_Write_Nibble(unsigned char data, unsigned char control); /* Writes 4-bits */
void LCD_String (char *str);	/* Send string to LCD function */

#define DHT (0)  //PB0 is connected to DHT11 sensor
#define MASK(x) (1UL<<(x))

//IO direction
#define DHT11_IO_IN() (GPIO_PORTB_DIR_R  &= ~MASK(DHT)) //PB0 input
#define DHT11_IO_OUT() (GPIO_PORTB_DIR_R |= MASK(DHT)) //PB0 output

//IO operation
#define DHT11_DQ_OUT(x) (x ?(GPIO_PORTB_DATA_R |= MASK(DHT)):(GPIO_PORTB_DATA_R &= ~MASK(DHT)))
#define DHT11_DQ_IN()   ((GPIO_PORTB_DATA_R & MASK(DHT))? 1:0)

uint8_t DHT11_Init(void);	//Init DHT11
uint8_t DHT11_Read_Data(void);//read temp and humidity.
uint8_t DHT11_Read_Byte(void);//read one byte.
uint8_t DHT11_Read_Bit(void);//read one bit.
uint8_t DHT11_Check(void);	//check DHT11
void DHT11_Rst(void);	//reset DHT11.

void delay_us(int us);
void delay_ms(int ms);

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

int main(void)
{
	int read_number = 0;
	//Uart_Init();
	Timer_Init();
	Pwm_Init();
	DHT11_Init();

    LCD_init();
    LCD_Cmd(clear_display);
    LCD_Cmd(FirstRow); /* Force cusor to begining of first row */
    delay_ms(500);
 
    LCD_Write_Char('A');
    delay_ms(500);
    
	delay_ms(400); // wait DHT11 for stability
	
	while(1)
	{
		while(DHT11_Read_Data())
		{ 
			delay_ms(5);
			if(read_number>250)
			{
				//printf("DHT11 does not work, please check and restart the system!\r\n");
				PWM1->ENABLE &=	(unsigned)(~(1<<6)); //disable pwm1 channel
			}
			read_number++;
			
		}
		//printf("Temp. is %d\r\n",temp);
		//printf("Humidity is %d\r\n", humi);
			
		read_number = 0;	
		
	}
}

//Init DHT11 of IO ports and then DQ check if DHT11 exist
//return 1:no dht11
//return 0:exist    	 
uint8_t DHT11_Init(void)
{
	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB;// active clock for PB.
	
	GPIO_PORTB_DIR_R |= (MASK(DHT)|MASK(DHT));
	GPIO_PORTB_DEN_R |=(MASK(DHT)|MASK(DHT));
	
	
	GPIO_PORTB_DIR_R |= MASK(DHT);

	DHT11_Rst();
	
	return DHT11_Check();
}

//DHT11 start
void DHT11_Rst(void)	   
{                 
	DHT11_IO_OUT(); 	//set output
	DHT11_DQ_OUT(0); 	//pull down DQ
	delay_ms(20);    	//pull down 18ms
	DHT11_DQ_OUT(1); 	//DQ=1 
	delay_us(25);     	//20~40us
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
		retry++;
		delay_us(1);
	}
	
	if(retry>=100)return 1;
	else retry=0;
   while (!DHT11_DQ_IN()&&retry<100)//DHT11 pull down and then pull up 40~80us
	{
		retry++;
		delay_us(1);
	}
	if(retry>=100)return 1;	    
	return 0;
}

//read one bit
//return 0/1
uint8_t DHT11_Read_Bit(void) 			 
{
 	uint8_t retry=0;
	while(DHT11_DQ_IN()&&retry<100)//wait low voltage
	{
		retry++;
		delay_us(1);
	}
	retry=0;
	while(!DHT11_DQ_IN()&&retry<100)//wait high voltage
	{
		retry++;
		delay_us(1);
	}
	delay_us(40);//wait 40us
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
   		dat<<=1; 
	    dat|=DHT11_Read_Bit();
    }						    
    return dat;
}

//read data from DHT11
//temp:(0-50)
//humi:(20%~90%)
uint8_t DHT11_Read_Data(void)    
{        
 	uint8_t buf[5];
	uint8_t i;
	DHT11_Rst();
	if(DHT11_Check()==0)
	{
		for(i=0;i<5;i++)//read 40 bits
		{
			buf[i]=DHT11_Read_Byte();
		}
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
		{
			humi=buf[0];
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
	UART0->FBRD = 11; 	//set baud rate in FBRD 
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
	while((UART0->FR	& (1<<5))!=0){}		
	return (ch);
}

void delay_us(int delay)  
{ 
	while(delay--){
		__asm("nop"); // almost 1us
	}
	
}


void delay_ms(int ms) 
{
	for(int i=0; i<ms; i++)
	{
		delay_us(1000); 
	}
}


//This function intialises the CAN Bus on Port B
void CAN0_Init()
{
    int temp;
    {
        //Port B CAN // PB5 = TX and PB4 = RX
        //Here the Port B is prepared for CAN operation
        volatile unsigned long delay;
        SYSCTL_RCGC0_R |= 0x01000000;     // Enable Clock for CAN0 Module
        SYSCTL_RCGC2_R |= 0x00000002;     // Port B clock gating control
        delay = SYSCTL_RCGC2_R;           // delay
        GPIO_PORTB_CR_R |= 0x30;          // allow changes to PB4-5 //Commit Register
        GPIO_PORTB_AMSEL_R &= 0x00;       // disable analog function
        GPIO_PORTB_DEN_R |= 0x30;         // Enable Digital Pins
        GPIO_PORTB_DIR_R |= 0x20;             // Direction Output for PB5 and Input for PB4
        GPIO_PORTB_AFSEL_R |= 0x30;       // CAN0 alternate function on PB4-5
        GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFF00FFFF)+0x00880000;
        // Set PCTL Register PMC5 and PMC4 to 0x08 for CAN functionality
    }
 
    //Initialise CAN0 Module
    {
        //Place CAN0 in Init Mode
        CAN0_CTL_R |= 0x01;                 //Set Init Bit in the Register
        while(CAN0_IF1CRQ_R & 0x00008000); //wait while CAN0 Module is BUSY
    }
 
    //Enabling Loopback test mode
    CAN0_CTL_R |= 0x81;
    CAN0_TST_R |= 0x10;
 
    //Bit rate setting
    {
        CAN0_CTL_R |= 0x40;                 //Setting the CCE bit to edit CANBIT Register
        CAN0_BIT_R |= 0x0201;               //Setting calculated according to example 1 in datasheet, 16Mhz and 1Mbps = 0x0203
                                            //But it gave a 500Kbps bit rate soo by trial and error 1Mbps = 0x0201
    }
 
    //No interrupts are used
    //CAN is enabled (Init = 0)
    CAN0_CTL_R &= ~0x41;
}

//Initialise the transmit message object
void CAN0_Transmit_Set(int objNo)       //objNo is the Object Number
{
    int CmdMaskReg = 0;
    int ArbReg0 = 0;
    int ArbReg1 = 0;
    int MsgCtrl = 0;
    int MaskReg0 = 0;
    int MaskReg1 = 0;
    char temp[10];
 
    while(CAN0_IF1CRQ_R & 0x00008000);  //wait while CAN0 Module is BUSY
 
    CmdMaskReg |= 0x93;                 //WRNRD, CONTROL, DATAA, DATAB bits set
 
    MsgCtrl |= 0x100;                   //TXRQST bit set (Not Necessary to do this)
    ArbReg1 = 0x2000;                   // DIR bit setting in ARB2 reg
 
    CmdMaskReg |= 0x20;                 //ARB bit set in CMSK reg
    ArbReg1 |= 0x04;                    // MSG ID set to 0x01 in ARB2 reg
 
    ArbReg1 |= 0x8000;                  //MSG Valid bit set in ARB2 reg
    MsgCtrl |= 0x88;                    // Set no. of data bytes in MCTL Reg 
 
    CAN0_IF1DA1_R = 0x5453;             //Actual Data to be sent &quot;START&quot; (Not required here, can be ignored)
    CAN0_IF1DA2_R = 0x5241;             //This is the first message sent.
    CAN0_IF1DB1_R = 0x0054;
    CAN0_IF1DB2_R = 0x0000;
 
    CAN0_IF1CMSK_R = CmdMaskReg;        //Registers updation
    CAN0_IF1MSK1_R = MaskReg0;
    CAN0_IF1MSK2_R = MaskReg1;
    CAN0_IF1ARB1_R = ArbReg0;
    CAN0_IF1ARB2_R = ArbReg1;
    CAN0_IF1MCTL_R = MsgCtrl;
 
    CAN0_IF1CRQ_R = objNo;              //Message object number
}


//Initialise the Receive message object
void CAN0_Receive_Set(int objNo)
{
  int CmdMaskReg = 0;
    int ArbReg0 = 0;
    int ArbReg1 = 0;
    int MsgCtrl = 0;
    int MaskReg0 = 0;
    int MaskReg1 = 0;
 
    while(CAN0_IF2CRQ_R & 0x00008000);  //wait while CAN0 Module is BUSY
 
    CmdMaskReg |= 0xD3;                 //WRNRD, CONTROL, MASK, DATAA, DATAB bits set
 
    ArbReg1 = 0x00;                     // DIR bit setting in ARB2 reg
 
    CmdMaskReg |= 0x20;                 //ARB bit set in CMSK reg
    ArbReg1 |= 0x00; 
 
    ArbReg1 |= 0x8000;                  //MSG Valid bit set in ARB2 reg
    MsgCtrl |= 0x1088;                  // Set no. of data bytes in MCTL Reg, EOB Set and UMASK set. 
 
    CAN0_IF2CMSK_R = CmdMaskReg;        //Registers updation
    CAN0_IF2MSK1_R = MaskReg0;
    CAN0_IF2MSK2_R = MaskReg1;
    CAN0_IF2ARB1_R = ArbReg0;
    CAN0_IF2ARB2_R = ArbReg1;
    CAN0_IF2MCTL_R = MsgCtrl;
 
    CAN0_IF2CRQ_R = objNo;              //Message object number
 
    while(CAN0_IF2CRQ_R & 0x00008000);  //wait while CAN0 Module is BUSY
}


//This function actually sends the data on the TX pin.
//Since registers have already been initialised in the CAN0_Transmit_Set function
void CAN0_Transmit_Data(int objNo,char data[8])
{
    int i;
    char temp[10];
    CAN0_IF1CMSK_R &= ~0x30; //Clear ARB and CONTROL bits in CMSK reg
    CAN0_IF1CMSK_R |= 0x83; //Set WRNRD, DATAA, DATAB bits in CMSK
 
    CAN0_IF1DA1_R = ((data[1]<<8)+data[0]); //Actual Data
    CAN0_IF1DA2_R = ((data[3]<<8)+data[2]);
    CAN0_IF1DB1_R = ((data[5]<<8)+data[4]);
    CAN0_IF1DB2_R = ((data[7]<<8)+data[6]);
 
    CAN0_IF1CMSK_R |= 0x87; //Set the NEWDAT and TXRQST bit in CMSK reg
 
    CAN0_IF1CRQ_R = objNo; //Message object number
    while(CAN0_IF1CRQ_R & 0x00008000); //Wait for BUSY bit to clear
 
    //Send debug messages to UART
    UART_OutString("Sending Data: ");
    for(i=0; i<8; i++)
    {
        sprintf(temp,"%c",data[i]);
        UART_OutString(temp);
        UART_OutChar(' ');
    }
    UART_OutChar('\n');
} 
 
//This function actually receives the data put on the RX pin.
//Since registers have already been initialised in the CAN0_Receive_Set function
char* CAN0_Receive_Data(int objNo)
{
    char temp[10];
    int ArbReg0 = 0;
    int ArbReg1 = 0;
    int MsgCtrl = 0;
    int MaskReg0 = 0;
    int MaskReg1 = 0;
    char data[8];
        int i;  
 
    CAN0_IF2CMSK_R = 0x73; //ARB, CONTROL, MASK, DATAA, DATAB bits set
 
    CAN0_IF2CRQ_R = objNo; //Message object number
 
    while(CAN0_IF2CRQ_R & 0x00008000);//wait while CAN0 Module is BUSY
 
    MaskReg0 = CAN0_IF2MSK1_R;//Get data from registers
        MaskReg1 = CAN0_IF2MSK2_R;
        ArbReg0 = CAN0_IF2ARB1_R;
        ArbReg1 = CAN0_IF2ARB2_R;
        MsgCtrl = CAN0_IF2MCTL_R;
 
    if(MsgCtrl & 0x8000) //If NEWDAT bit is set i.e. there is new data in the data registers
    {
        data[0] = (0x00FF & CAN0_IF2DA1_R); //Get this new data.
        data[1] = ((0xFF00 & CAN0_IF2DA1_R)>>8);
        data[2] = (0x00FF & CAN0_IF2DA2_R);
        data[3] = ((0xFF00 & CAN0_IF2DA2_R)>>8);
        data[4] = (0x00FF & CAN0_IF2DB1_R);
        data[5] = ((0xFF00 & CAN0_IF2DB1_R)>>8);
        data[6] = (0x00FF & CAN0_IF2DB2_R);
        data[7] = ((0xFF00 & CAN0_IF2DB2_R)>>8);
 
        CAN0_IF2CMSK_R |= 0x04;//Set NEWDAT in CMSK to Clear the NEWDAT bit in MCTL
    }
 
    if(MsgCtrl & 0x4000) //If MSGLST bit is set i.e. there was a message lost
    {
        MsgCtrl &= ~0x4000; //Clear the MSGLST bit
        CAN0_IF2MCTL_R = MsgCtrl; //Clear the MSGLST bit
        UART_OutString("Message lost\n");
    }
 
    CAN0_IF2CRQ_R = objNo; //Very important, when the message object no. is written the values in register (MCTL, ARB etc.) are updated or transfered.
 
    UART_OutString("Received Message:");
    UART_OutString(" ID ");
    sprintf(temp,"%d", ((ArbReg1 & 0x1FFF)>>3));
    UART_OutString(temp);
    UART_OutString(" RTR 0");
    UART_OutString(" DLC ");
    sprintf(temp,"%d", (MsgCtrl & 0xF));
    UART_OutString(temp);   
 
    //Send debug messages to UART
    UART_OutString(" Data: ");
    for(i=0; i<8; i++)
    {
        sprintf(temp,"%d",data[i]);
        UART_OutString(temp);
        UART_OutChar(' ');
    }
    UART_OutChar('\n');
    return data;
}


/* LCD and GPIOB initialization Function */ 
void LCD_init(void)
{
 SYSCTL->RCGCGPIO |=(1<<1);     /* Enable Clock to GPIOB */
 LCD->DIR |=0xFF;               /* Set GPIOB all pins a digital output pins */
 LCD->DEN |=0xFF;               /* Declare GPIOB pins as digital pins */

 LCD_Cmd(Set5x7FontSize);       /* select 5x7 font size and 2 rows of LCD */
 LCD_Cmd(Function_set_4bit);    /* Select 4-bit Mode of LCD */
 LCD_Cmd(moveCursorRight);      /* shift cursor right */
 LCD_Cmd(clear_display);        /* clear whatever is written on display */
 LCD_Cmd(cursorBlink);          /* Enable Display and cursor blinking */
 
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


void LCD_Write_Nibble(unsigned char data, unsigned char control)
{

    data &= 0xF0;                       /* Extract upper nibble for data */
    control &= 0x0F;                    /* Extract lower nibble for control */
    LCD->DATA = data | control;         /* Set RS and R/W to zero for write operation */
    LCD->DATA = data | control | EN;    /* Provide Pulse to Enable pin to perform wite operation */
    delay_us(0);
    LCD->DATA = data;                   /*Send data */
    LCD->DATA = 0;                      /* stop writing data to LCD */
}
void LCD_Write_Char(unsigned char data)
{
    LCD_Write_Nibble(data & 0xF0, RS);    /* Write upper nibble to LCD and RS = 1 to write data */
    LCD_Write_Nibble(data << 4, RS);      /* Write lower nibble to LCD and RS = 1 to write data */
    delay_us(40);
}


void LCD_String (char *str)	            /* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)              /* Send each char of string till the NULL */
	{
		LCD_Write_Char(str[i]);         /* Call LCD data write */
	}
}

/* Mili seconds delay function */
void delay_ms(int n)
{
 int i,j;
 for(i=0;i<n;i++)
 for(j=0;j<3180;j++)
 {}
}

/* Micro seconds delay function */
void delay_us(int n)
{
 int i,j;
 for(i=0;i<n;i++)
 for(j=0;j<3;j++)
 {}
 
}