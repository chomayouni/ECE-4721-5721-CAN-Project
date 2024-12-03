#include "main.h"


void CAN0_Init(void);

void CAN_TX(unsigned char data);

void CAN_RX_init(void);
uint8_t CAN_RX(void);

void delay(uint32_t count);
void delayms(uint32_t count);

int main(void) {
	CAN0_Init();

	CAN_RX_init();
	uint8_t Current = 0;
	uint8_t temp = 'A';
	
	while(1){
		CAN_TX(temp);
		if(CAN0_IF2MCTL_R&CAN_IF2MCTL_NEWDAT){
			Current = CAN_RX();
		}
	}
}

void CAN0_Init(void) {
		// designed around 16 MHz default clock to enable can communication for 100 Kbps
	
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB; // Activate clock for port B
		// wait for clock to stabilize
		while((SYSCTL_PRGPIO_R & SYSCTL_RCGC2_GPIOB) == 0){};
			
    // Enable CAN0 module
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_CAN0;
    // Wait for clock to stabilize
    while((SYSCTL_RCGC0_R & SYSCTL_RCGC0_CAN0) == 0){};
    
    // Wait for clock to stabilize
    while((SYSCTL_RCC_R & (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32)) == 0){};			
			
    // Configure CAN0 pins
    GPIO_PORTB_AFSEL_R |= (MASK(RX)|MASK(TX));  // Enable alternate function for CAN0
    GPIO_PORTB_PCTL_R &= ~(0xF << (RX*4));       // Clear PCTL bits
    GPIO_PORTB_PCTL_R |= (8 << (RX*4));           // Set PCTL for PWM
    GPIO_PORTB_PCTL_R &= ~(0xF << (TX*4));       // Clear PCTL bits
    GPIO_PORTB_PCTL_R |= (8 << (TX*4));           // Set PCTL for PWM
			
    GPIO_PORTB_AMSEL_R &= ~(MASK(RX)|MASK(TX)) ; // disable analog function for pins
		GPIO_PORTB_DEN_R |= (MASK(RX)|MASK(TX)); // Enable digital output
			
    // Configure CAN0 
    CAN0_CTL_R = CAN_CTL_CCE | CAN_CTL_INIT;                    // set CAN0 to init mode and enable CCE
			
		// configure time quantum for 100 kbps, 100 Kbps corresponds to 160 SYSCLK clock cycles 
		// Baud Rate = SYSCLK/(BRP * (TSEG1 + TSEG2 + 3)) = 100 kHz, SYSCLK = 16 MHz, 
		// resets to TSEG1 = 3, TSEG2 = 2, SJW synch jump width is set to 0 which corresponds to 1 bit
		// Baud Rate = SYSCLK/(BRP * (3 + 2 + 3))= 100 kHz, therefore BRP = 20
		CAN0_BIT_R &=	~0x1f; // clear BRP field
		CAN0_BIT_R |= 20-1;	 // set BRP to 19, which acts as 20. CANBRPE is not used
		CAN0_CTL_R &= ~CAN_CTL_CCE; // disable CCE
		CAN0_CTL_R &= ~CAN_CTL_INIT;  // exit initialization
 
}

void CAN_TX(unsigned char data){
	// set the CAN to write mode, transfer the mask, arbitration, control, and all data bytes to the interface registers
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_WRNRD|CAN_IF1CMSK_MASK|CAN_IF1CMSK_ARB|CAN_IF1CMSK_CONTROL|CAN_IF1CMSK_DATAA|CAN_IF1CMSK_DATAB;
	// allow all messages to pass through acceptance filtering
	CAN0_IF1MSK1_R = 0x00;
	CAN0_IF1MSK2_R = 0x00;
	
	CAN0_IF1ARB2_R &= ~CAN_IF1ARB2_XTD; // 11 bit ID
	CAN0_IF1ARB2_R |= 1<<2 | CAN_IF1ARB2_DIR; // set message ID to 1 for 11 bit ID and set as transmit direction
	CAN0_IF1ARB2_R |= CAN_IF1ARB2_MSGVAL; // the message is valid and ready to be considered
	
	CAN0_IF1MCTL_R |= CAN_IF1MCTL_EOB|1; // set for Single message object with EOB and 1 byte data frame
	
	CAN0_IF1DA1_R = 0x0000 | data; // load the data to be transmitted
	
	CAN0_IF1CRQ_R |= 1; // use message object 1
	
	CAN0_IF1MCTL_R |= CAN_IF1MCTL_TXRQST; // request transmission
}


void CAN_RX_init(void){
	// set the CAN to write mode, transfer the mask, arbitration, control, and all data bytes to the interface registers
	CAN0_IF2CMSK_R |= CAN_IF2CMSK_WRNRD|CAN_IF2CMSK_MASK|CAN_IF2CMSK_ARB|CAN_IF2CMSK_CONTROL|CAN_IF2CMSK_DATAA|CAN_IF2CMSK_DATAB;
	// allow all messages to pass through acceptance filtering
	CAN0_IF2MSK1_R = 0x00;
	CAN0_IF2MSK2_R = 0x00;
	
	CAN0_IF2ARB2_R &= ~(CAN_IF2ARB2_XTD)| CAN_IF2ARB2_DIR; // 11 bit ID, recieve mode
	CAN0_IF2ARB2_R |= 1<<2 ; // set message ID to 1 for 11 bit ID to be recieved
	CAN0_IF2ARB2_R |= CAN_IF2ARB2_MSGVAL; // the message is valid and ready to be considered
	
	CAN0_IF2MCTL_R &= ~CAN_IF2MCTL_RMTEN; // disable remote frame respone
	CAN0_IF2MCTL_R |= CAN_IF2MCTL_EOB|1; // set for Single message object with EOB and 1 byte data frame
	
	CAN0_IF2CMSK_R |= 0x007F; // stop writing to the interface registers, pull all data from the messages recieved
	CAN0_IF2CRQ_R |= 2; // use message object 2
}

uint8_t CAN_RX(void){
	CAN0_IF2CMSK_R |= 0x007F; // stop writing to the interface registers, pull all data from the messages recieved
	CAN0_IF2CRQ_R |= 2; // use message object 2

	return (uint8_t)CAN0_IF2DA1_R; // pull the lower byte from the data register
}

void delayms(uint32_t count){
    volatile uint32_t i;
    for(i = 0; i < count; i++) {
			delay(1058); // empirical constant for 1ms delay
		}
}
void delay(uint32_t count) {
    volatile uint32_t i;
    for(i = 0; i < count; i++) {}
}



