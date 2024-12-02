
#include "TM4C123.h"
#include "canDriver.h"

//This function initializes the CAN Bus on Port B
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
		GPIO_PORTB_DIR_R |= 0x20;         // Direction Output for PB5 and Input for PB4
		GPIO_PORTB_AFSEL_R |= 0x30;       // CAN0 alternate function on PB4-5
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFF00FFFF) + 0x00880000;
		// Set PCTL Register PMC5 and PMC4 to 0x08 for CAN functionality
	}
 
	//Initialize CAN0 Module
	{
		//Place CAN0 in Init Mode
		CAN0_CTL_R |= 0x01;                 //Set Init Bit in the Register
		while (CAN0_IF1CRQ_R & 0x00008000); //wait while CAN0 Module is BUSY
	}
 
	//Enabling Loopback test mode
	CAN0_CTL_R |= 0x81;
	CAN0_TST_R |= 0x10;
 
	//Bit rate setting
	{
		CAN0_CTL_R |= 0x40;                 //Setting the CCE bit to edit CANBIT Register
		CAN0_BIT_R |= 0x0201;               //Setting calculated according to example 1 in datasheet, 16Mhz and 1Mbps = 0x0203
											//But it gave a 500Kbps bit rate so by trial and error 1Mbps = 0x0201
	}
 
	//No interrupts are used
	//CAN is enabled (Init = 0)
	CAN0_CTL_R &= ~0x41;
}

//Initialize the transmit message object
void CAN0_Transmit_Set(int objNo)       //objNo is the Object Number
{
	int CmdMaskReg = 0;
	int ArbReg0 = 0;
	int ArbReg1 = 0;
	int MsgCtrl = 0;
	int MaskReg0 = 0;
	int MaskReg1 = 0;
	char temp[10];
 
	while (CAN0_IF1CRQ_R & 0x00008000);  //wait while CAN0 Module is BUSY
 
	CmdMaskReg |= 0x93;                 //WRNRD, CONTROL, DATAA, DATAB bits set
 
	MsgCtrl |= 0x100;                   //TXRQST bit set (Not Necessary to do this)
	ArbReg1 = 0x2000;                   // DIR bit setting in ARB2 reg
 
	CmdMaskReg |= 0x20;                 //ARB bit set in CMSK reg
	ArbReg1 |= 0x04;                    // MSG ID set to 0x01 in ARB2 reg
 
	ArbReg1 |= 0x8000;                  //MSG Valid bit set in ARB2 reg
	MsgCtrl |= 0x88;                    // Set no. of data bytes in MCTL Reg 
 
	CAN0_IF1DA1_R = 0x5453;             //Actual Data to be sent "START" (Not required here, can be ignored)
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

//Initialize the Receive message object
void CAN0_Receive_Set(int objNo)
{
	int CmdMaskReg = 0;
	int ArbReg0 = 0;
	int ArbReg1 = 0;
	int MsgCtrl = 0;
	int MaskReg0 = 0;
	int MaskReg1 = 0;
 
	while (CAN0_IF2CRQ_R & 0x00008000);  //wait while CAN0 Module is BUSY
 
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
 
	while (CAN0_IF2CRQ_R & 0x00008000);  //wait while CAN0 Module is BUSY
}

//This function actually sends the data on the TX pin.
//Since registers have already been initialized in the CAN0_Transmit_Set function
void CAN0_Transmit_Data(int objNo, char data[8])
{
	int i;
	char temp[10];
	CAN0_IF1CMSK_R &= ~0x30; //Clear ARB and CONTROL bits in CMSK reg
	CAN0_IF1CMSK_R |= 0x83; //Set WRNRD, DATAA, DATAB bits in CMSK
 
	CAN0_IF1DA1_R = ((data[1]<<8) + data[0]); //Actual Data
	CAN0_IF1DA2_R = ((data[3]<<8) + data[2]);
	CAN0_IF1DB1_R = ((data[5]<<8) + data[4]);
	CAN0_IF1DB2_R = ((data[7]<<8) + data[6]);
 
	CAN0_IF1CMSK_R |= 0x87; //Set the NEWDAT and TXRQST bit in CMSK reg
 
	CAN0_IF1CRQ_R = objNo; //Message object number
	while (CAN0_IF1CRQ_R & 0x00008000); //Wait for BUSY bit to clear
 
	//Send debug messages to UART
	UART_OutString("Sending Data: ");
	for (i = 0; i < 8; i++)
	{
		sprintf(temp, "%c", data[i]);
		UART_OutString(temp);
		UART_OutChar(' ');
	}
	UART_OutChar('\n');
}

//This function actually receives the data put on the RX pin.
//Since registers have already been initialized in the CAN0_Receive_Set function
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
 
	while (CAN0_IF2CRQ_R & 0x00008000); //wait while CAN0 Module is BUSY
 
	MaskReg0 = CAN0_IF2MSK1_R; //Get data from registers
	MaskReg1 = CAN0_IF2MSK2_R;
	ArbReg0 = CAN0_IF2ARB1_R;
	ArbReg1 = CAN0_IF2ARB2_R;
	MsgCtrl = CAN0_IF2MCTL_R;
 
	if (MsgCtrl & 0x8000) //If NEWDAT bit is set i.e. there is new data in the data registers
	{
		data[0] = (0x00FF & CAN0_IF2DA1_R); //Get this new data.
		data[1] = ((0xFF00 & CAN0_IF2DA1_R) >> 8);
		data[2] = (0x00FF & CAN0_IF2DA2_R);
		data[3] = ((0xFF00 & CAN0_IF2DA2_R) >> 8);
		data[4] = (0x00FF & CAN0_IF2DB1_R);
		data[5] = ((0xFF00 & CAN0_IF2DB1_R) >> 8);
		data[6] = (0x00FF & CAN0_IF2DB2_R);
		data[7] = ((0xFF00 & CAN0_IF2DB2_R) >> 8);
 
		CAN0_IF2CMSK_R |= 0x04; //Set NEWDAT in CMSK to Clear the NEWDAT bit in MCTL
	}
 
	if (MsgCtrl & 0x4000) //If MSGLST bit is set i.e. there was a message lost
	{
		MsgCtrl &= ~0x4000; //Clear the MSGLST bit
		CAN0_IF2MCTL_R = MsgCtrl; //Clear the MSGLST bit
		UART_OutString("Message lost\n");
	}
 
	CAN0_IF2CRQ_R = objNo; //Very important, when the message object no. is written the values in register (MCTL, ARB etc.) are updated or transferred.
 
	UART_OutString("Received Message:");
	UART_OutString(" ID ");
	sprintf(temp, "%d", ((ArbReg1 & 0x1FFF) >> 3));
	UART_OutString(temp);
	UART_OutString(" RTR 0");
	UART_OutString(" DLC ");
	sprintf(temp, "%d", (MsgCtrl & 0xF));
	UART_OutString(temp);   
 
	//Send debug messages to UART
	UART_OutString(" Data: ");
	for (i = 0; i < 8; i++)
	{
		sprintf(temp, "%d", data[i]);
		UART_OutString(temp);
		UART_OutChar(' ');
	}
	UART_OutChar('\n');
	return data;
}