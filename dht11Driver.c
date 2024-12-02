
#include "TM4C123.h"
#include "dht11Driver.h"

//IO direction
#define DHT11_IO_IN() (GPIO_PORTB_DIR_R  &= ~MASK(DHT)) //PB0 input
#define DHT11_IO_OUT() (GPIO_PORTB_DIR_R |= MASK(DHT)) //PB0 output

//IO operation
#define DHT11_DQ_OUT(x) (x ?(GPIO_PORTB_DATA_R |= MASK(DHT)):(GPIO_PORTB_DATA_R &= ~MASK(DHT)))
#define DHT11_DQ_IN()   ((GPIO_PORTB_DATA_R & MASK(DHT))? 1:0)

//Init DHT11 of IO ports and then DQ check if DHT11 exist
//return 1:no dht11
//return 0:exist    	 
uint8_t DHT11_Init(void)
{
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB; // activate clock for PB
	
	GPIO_PORTB_DIR_R |= (MASK(DHT) | MASK(DHT));
	GPIO_PORTB_DEN_R |= (MASK(DHT) | MASK(DHT));
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
	uint8_t retry = 0;
	DHT11_IO_IN(); //SET INPUT	 
	while (DHT11_DQ_IN() && retry < 100) //DHT11 will pull down 40~80us
	{
		retry++;
		delay_us(1);
	}
	
	if (retry >= 100) return 1;
	else retry = 0;
	while (!DHT11_DQ_IN() && retry < 100) //DHT11 pull down and then pull up 40~80us
	{
		retry++;
		delay_us(1);
	}
	if (retry >= 100) return 1;	    
	return 0;
}

//read one bit
//return 0/1
uint8_t DHT11_Read_Bit(void) 			 
{
	uint8_t retry = 0;
	while (DHT11_DQ_IN() && retry < 100) //wait low voltage
	{
		retry++;
		delay_us(1);
	}
	retry = 0;
	while (!DHT11_DQ_IN() && retry < 100) //wait high voltage
	{
		retry++;
		delay_us(1);
	}
	delay_us(40); //wait 40us
	if (DHT11_DQ_IN()) return 1;
	else return 0;		   
}

//read one byte
//return byte from dht11
uint8_t DHT11_Read_Byte(void)    
{        
	uint8_t i, dat;
	dat = 0;
	for (i = 0; i < 8; i++) 
	{
		dat <<= 1; 
		dat |= DHT11_Read_Bit();
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
	if (DHT11_Check() == 0)
	{
		for (i = 0; i < 5; i++) //read 40 bits
		{
			buf[i] = DHT11_Read_Byte();
		}
		if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
		{
			humi = buf[0];
			temp = buf[2];
		}
		else return 1;
	}
	else return 1;
	return 0;	    
}