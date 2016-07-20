#define 	SCL 	RC3			// 	I2C ±µ¸}
#define		SDA		RC4
#define		SCL_DIR	TRISC3
#define		SDA_DIR	TRISC4

#define		PAC1710_Addr_Read		0b00110001  
#define		PAC1710_Addr_Write	0b00110000	

void I2C_Init(void);
void Write_PAC1710(unsigned char Addr,unsigned char data);
void Read_PAC1710_1_Byte(unsigned char Addr);
void Read_PAC1710_2_Byte(unsigned char Addr);

