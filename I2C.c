#include <pic.h>
#include "I2C.h"


static volatile unsigned char Runt;

union
{ 	
	unsigned int Full_Val ;
	unsigned char Temp[2] ;
}PAC1710;

void I2C_Init(void)
{
	SCL=1;				// Initial PortC,bit 3 & 4 as Input with Open Drain
	SDA=1;				// RC3 = SCL  ,  RC4 = SDA
	SCL_DIR =1;			 
	SDA_DIR =1;
	
	SSPSTAT = 0;
	SSPCON1 = 0x28;
	SSPCON2 = 0;
	SSPCON3 = 0;
	SSPADD = 4;
}

//********************************************************************
// As below are I2C function
// These functions were modified from C18 I2C Libyaries' Source Code
//********************************************************************
//**** Test and wait until I2C module is idle.
void IdleI2C( void ) 				
{
  while ( ( SSPCON2 & 0x1F ) || ( R_nW ) )
     continue;
}

//Send I2C bus start condition.
void StartI2C( void )
{
  SEN = 1;            // initiate bus start condition
}

//This routine writes a single byte to the I2C bus.
unsigned char WriteI2C( unsigned char data_out )	
{
  SSPBUF = data_out;           	// write single byte to SSPBUF
  if (WCOL)      				// test if write collision occurred
   return ( -1 );              	// if WCOL bit is set return negative #
  else
  {
    while(BF);   				// wait until write cycle is complete         
    IdleI2C();                 	// ensure module is idle
    if ( SSPCON2bits.ACKSTAT ) 	// test for ACK condition received
    	 return ( -2 );
	else return ( 0 );          // if WCOL bit is not set return non-negative #
  }
}

unsigned char ReadI2C( void )
{
  RCEN = 1;           // enable master for 1 byte reception
  while ( !BF );      // wait until byte received  
  return ( SSPBUF );              // return with read byte 
}

void AckI2C( void )
{
  ACKDT = 0;           // set acknowledge bit state for ACK
  ACKEN = 1;           // initiate bus acknowledge sequence
}

void NotAckI2C( void )
{
  ACKDT = 1;          // set acknowledge bit for not ACK
  ACKEN = 1;          // initiate bus acknowledge sequence
}

void RestartI2C( void )
{
  RSEN = 1;           // initiate bus restart condition
}
void StopI2C( void )
{
  PEN = 1;            // initiate bus stop condition
}

void I2C_Done(void)
{
	while (!SSP1IF);			// Completed the action when the SSPIF is Hi.
	SSP1IF=0;   				// Clear SSPIF
}

void Read_PAC1710_2_Byte(unsigned char Addr)
{
	IdleI2C();             			// ensure module is idle
  	StartI2C();						// Start condition
	I2C_Done();						// Wait Start condition completed

	Runt=WriteI2C(PAC1710_Addr_Write);	// 寫入 0x9A 命令
	I2C_Done();						// Clear SSPIF flag

	Runt=WriteI2C(Addr);	// 寫入讀取溫度的命令          
	I2C_Done();						// Clear SSPIF flag

   	RestartI2C();              		// initiate Restart condition
	I2C_Done();

	Runt=WriteI2C(PAC1710_Addr_Read);	  
	I2C_Done();						// Clear SSPIF flag

	PAC1710.Temp[1]=ReadI2C();		// Enable I2C Receiver & wait BF=1 until received data
	I2C_Done();						// Clear SSPIF flag

	AckI2C();						// Genarate Non_Acknowledge to MCP9800
	I2C_Done();	   
 
	PAC1710.Temp[0]=ReadI2C();		// Enable I2C Receiver & wait BF=1 until received data
	I2C_Done();						// Clear SSPIF flag

	NotAckI2C();					// Genarate Non_Acknowledge to MCP9800
	I2C_Done();	  
                     
    StopI2C();                  	// send STOP condition
	I2C_Done();						// wait until stop condition is over 
}

void Read_PAC1710_1_Byte(unsigned char Addr)
{
	IdleI2C();             			// ensure module is idle
  	StartI2C();						// Start condition
	I2C_Done();						// Wait Start condition completed

	Runt=WriteI2C(PAC1710_Addr_Write);	// 寫入 0x9A 命令
	I2C_Done();						// Clear SSPIF flag

	Runt=WriteI2C(Addr);	// 寫入讀取溫度的命令          
	I2C_Done();						// Clear SSPIF flag

   	RestartI2C();              		// initiate Restart condition
	I2C_Done();

	Runt=WriteI2C(PAC1710_Addr_Read);	  
	I2C_Done();						// Clear SSPIF flag
   
	PAC1710.Temp[0]=ReadI2C();		// Enable I2C Receiver & wait BF=1 until received data
	I2C_Done();						// Clear SSPIF flag

	NotAckI2C();					// Genarate Non_Acknowledge to MCP9800
	I2C_Done();	  
                     
    StopI2C();                  	// send STOP condition
	I2C_Done();						// wait until stop condition is over 
	
	 
}


void Write_PAC1710(unsigned char Addr,unsigned char data)
{
	IdleI2C();             			// ensure module is idle
  	StartI2C();						// Start condition
	I2C_Done();						// Wait Start condition completed

	Runt=WriteI2C(PAC1710_Addr_Write);	  
	I2C_Done();					

	Runt=WriteI2C(Addr);	          
	I2C_Done();						

   	Runt=WriteI2C(data);			
	I2C_Done();						// Clear SSPIF flag 
                     
    StopI2C();                  	// send STOP condition
	I2C_Done();						// wait until stop condition is over 	 
}