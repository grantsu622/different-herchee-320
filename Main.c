/******************************************************************************
 * 合騏工業 320 機種
 * base on TGB source code
 * ***************************************************************************/

#include <pic.h>
#include "I2C.h"
#include "ADC.h"
#include "main.h"

	__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_ON & CP_OFF & BOREN_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF); 
	__CONFIG(WRT_ALL & PLLEN_OFF & STVREN_ON & LVP_OFF & BORV_HI);
//================================================================================================
//  寫入燒入程式的版本  
//  EEPROM data  [NOP]  [年]  [年]  [月]  [日] [版次] [NOP]  [NOP]
//================================================================================================

//__EEPROM_DATA(0xAB, 0x20, 0x16, 0x06, 0x15, 0x01, 0xFF, 0xFF);
    
    
    
#define	Comeback_2WD_EN     			0
#define	Self_Test_EN							0
//#define	WBY_EN		 								0
#define Front_Position_EN					0										//前差馬達定位功能
#define Back_Position_EN					0												//後差馬達定位功能
#define _2WD_Position_EN					0
#define _4WDL_Position_EN					0
#define LED_EN										1
#define _4WD_Test_EN	 						0
#define No_Feedback_EN       			1
#define  AUTORUN    0 

#define	OUTPUT										0
#define	INPUT											1
#define HIGH											1
#define LOW												0
//#define _5S_Val										12											//1.5秒
#define _5S_Val										39		//5秒
//#define _5S_Val										8
//#define _3S_Val										24												//3秒
//#define PAC1710_Error_val					10
#define	RPM_Speed									0x927C									//rpm 1600


////////////////////////////////// 錯誤模式下拉纜繩參數//////////////////////////////////////
//#define Pull_Value								3
#define Pull_Value	20   //20160629
//#define Pull_Count_Val						39										//5秒
#define Pull_Count_Val						16										//2秒 20160629
unsigned char Pull_Count = 0;//					= Pull_Value;
unsigned char Pull = 0;
unsigned char Pull_Timer_Star = 0;											//錯誤模式下啟動拉纜繩計時旗標
unsigned char Pull_Error      = 0;
unsigned char Pull_5S_CNT = Pull_Count_Val;

////////////////////////////////// 把手訊號//////////////////////////////////////
#if(_4WD_Test_EN)
#define _2WD											0b00100011
#define _2WD_1										0b00100111
#define _2WDLOCK									0b00000011
#define _2WDLOCK_1								0b00000111
#define _4WD_1										0b00010001
#define _4WD_2										0b00110001
#define _4WD_3										0b00010101
#define _4WD_4										0b00110101
#define _4WDLOCK_1								0b00010010
#define _4WDLOCK_2								0b00110010								
#define _4WD_Test									0b00010110									
#define _4WD_Test_1								0b00110110									

#else
#define _LOCK   	0b00000000  //0x00                    
#define _UNLOCK		0b00100000  //0x20 			                      

#endif
////////////////////////////////////變速箱///////////////////////////////////////
#define L1_Out										LATE2										
#define L2_Out										LATE1
#define L3_Out										LATE0

//#define _4WDLOCK_Gear							0b100
//#define _4WD_Gear									0b110
//#define _2WDLOCK_Gear							0b010
//#define _2WD_Gear									0b011
//#if(_4WD_Test_EN)
//#define _4WD_Gear_Test						0b111
//#endif

//unsigned char TD_Status;
unsigned char TD_OLD_Status;
unsigned char	TD_Temp;
//unsigned char	OLD_2WD_Status 			= 0;    
//unsigned char OLD_2WDLOCK_Status  = 0;  
//unsigned char OLD_4WD_Status 			= 0;   
//unsigned char OLD_4WDLOCK_Status  = 0;
#if(_4WD_Test_EN)
unsigned char OLD_4WD_Test_Status = 0;
#endif

unsigned char Now_Status 					= 0;
unsigned char TD_CNT;


//#define L1												RE2										
//#define L2												RE1
//#define L3												RE0




////////////////////////////////////馬達///////////////////////////////////////


#define Motor1_B_in								RA1
#define Motor1_Y_in								RA3
#define Motor1_W_in								RA0
#define Motor1_Y_out							LATA4
#define Motor1_W_out							LATA2
#define Motor2_B_in								RC1
#define Motor2_Y_in								RC2
#define Motor2_W_in								RC0
#define Motor2_Y_out							LATA6
#define Motor2_W_out							LATA7
//unsigned char Motor2_Origin 			= 0;
//unsigned char Motor1_Origin 			= 0;
//unsigned char	Motor1_2WL 					= 0; 		           	
//unsigned char	Motor1_4WL 					= 0;		                  
//unsigned char	Motor2_Lock 				= 0;
unsigned char	OLD_Motor_2WDLOCK_Gear 	= 0;
unsigned char	OLD_Motor_4WD_Gear 			= 0;		 
unsigned char	OLD_Motor_2WD_Gear 			= 0;    
unsigned char	OLD_Motor_4WDLOCK_Gear 	= 0;
unsigned char	Motor_Remove            = 0;
#if(_4WD_Test_EN)
unsigned char Motor_4WD_Gear_Test = 0;		
#endif

unsigned char Motor_Front_Status	= 0;
unsigned char Motor_Back_Status		= 0;
unsigned char Motor_Status_Now		= 0;
unsigned char	Motor_OLD_Status		= 0;
unsigned char	Motor_Temp	      	= 0;

#define	Motor1_Status_4WD 					0b00000010					//前差RA0/W,RA1/B,RA3/Y
#define	Motor1_Status_2WL 	 				0b00001000					//前差RA0/W,RA1/B,RA3/Y
#define	Motor1_Status_4WL 	 				0b00001010					//前差RA0/W,RA1/B,RA3/Y
#define	Motor2_Status_UnLock 				0b00000010					//後差RC0/W,RC1/B,RC2/Y
#define	Motor2_Status_Lock 	 				0b00000100					//後差RC0/W,RC1/B,RC2/Y

//以下定義值後差馬達狀態
//Motor_Status_Now 為前後馬達合成後的狀態
#define	Motor_UNLOCK_Status	 			  0b00000010			//0x02
#define	Motor_LOCK_Status	 			  0b00000100			//0x04
////////////////////////////////////把手///////////////////////////////////////

unsigned char Gear_Status_NEW;
unsigned char	Gear_Status_OLD 		= 0;
unsigned int  Init_Flag 					= 0;
unsigned int	Init_Final 					= 0;

////////////////////////////////////RPM///////////////////////////////////////

unsigned char RPM_Flag 						= 0;
unsigned int 	PRM_NEW;
unsigned int	RPM_OLD 						= 0;
unsigned int	RPM_VAL;
unsigned char	RPM_Zero 						= 0 ;

////////////////////////////////////錯誤旗標///////////////////////////////////////

unsigned int	DelayTime_Count 		= 0; 
unsigned char Error_Flag 					= 0;										//5秒Time out旗標
//unsigned char PAC1710_Error 			= 0;								//過壓、欠壓、過流旗標
unsigned char	RPM_Speed_Error 		= 0 ;
//unsigned char	Overcurrent_Error 	= 0;
unsigned char	Over_Speed_Error 		= 0;
//unsigned char	_5S_Flage_Error = 0;
unsigned char	Front_Error 				= 0;
unsigned char	Back_Error 					= 0;
unsigned char Error_Mode 					= 0;
unsigned char	Handback_Error 			= 0;								//把手失效錯誤
extern unsigned char Voltage_Error;							//max2014

////////////////////////////////////Timer///////////////////////////////////////

unsigned char _1S_CNT = 8;											//RPM訊號斷線超過1秒偵測
unsigned char	_5S_CNT = _5S_Val;								
unsigned char Work_status = 0;
unsigned int  _5S_Status_Temp;


unsigned int LED1_Count = 0;
unsigned int LED2_Count = 0;
unsigned int LED3_Count = 0;
unsigned int LED13_Count = 0;

unsigned char	Special = 0,i,j,k;
unsigned int  temp;
unsigned char Moving_Status;

//for 320 model
unsigned char IsChangeSpeed = 0;
unsigned char IsStart = 0;
unsigned char SpeedCunt = 0;
#define SPEED_CUNT_3KM  20
#define SPEED_CUNT_MAX  25 


///////////////////////////////
unsigned char Speed = 30,Speed_U = 0,Speed_H = 0,Speed_L = 0;
unsigned char Speed_Work_Status = 0,Seep_256ms_Cnt = 2,Speed_rd = 0;;
///////////////////////////////

void Motor1_F(void);
void Motor1_R(void);
void Motor1_S(void);
void Motor2_F(void);
void Motor2_R(void);
void Motor2_S(void);
void T2_Start(void);
void T2_Stop(void);
void Change_Func(unsigned char Goto,unsigned char Status);
void T1G_RPM_Init(void);
//void PAC1710_Error_Func(void);
void Error_Exit_Func(void);
void Delay_128msec(unsigned int Time);
void LED1_FLASH(unsigned int Time);
void LED2_FLASH(unsigned int Time);
void LED3_FLASH(unsigned int Time);
void LED13_FLASH(unsigned int Time);
void Self_Test(void);
//void ReadFeedback(void);
void Front_Position(void);
void Back_Position(void);
void _2WD_Position(void);
void _4WDL_Position(void);

void Check_Status(void);
void Error_Mode_Func(unsigned char Goto,unsigned char Status);
void Check_Hand_Status(void);
void Check_Motor_Status(void);
//void Check_First_Status(void);
void Output_ECU(void);


extern union
{ 	
	unsigned int Full_Val ;
	unsigned char Temp[2] ;
}PAC1710;



unsigned char tmp,error_cnt = 3;

void interrupt ISRs(void)
{
	 //if(INTF & INTE)
	 //{
	 	INTF = 0;
	 //}
	 

	 if(IOCIE & IOCIF)	 	
	{
		if(IOCBF5)
		{

		//	LATB0 = 1;

			IOCBF5 = 0;
			IOCIF = 0;
            if (IsStart)
            {
                IsStart = 0;
                IOCIE = 0;
                return;
            }
            if (SpeedCunt > SPEED_CUNT_3KM)
            {
                SpeedCunt = 0;
                IsChangeSpeed = 1;
            }
            else
            {
                SpeedCunt = 0;
                IsChangeSpeed = 0;
            }


			IOCIE = 0;

		}
	}
// ==================T2計算方式===========================================  
//			1/(8M/4/16/64/250)= 128 msec
//
//			1/(8M/4/16/64/19.53)= 10 msec
// =======================================================================

	if(TMR2IF & TMR2IE)				// 128mS
	{	
		TMR2IF = 0;

        if (IsStart) IsChangeSpeed = 1;
		
        if (SpeedCunt > SPEED_CUNT_MAX )
        {
            SpeedCunt = 0;
            IsChangeSpeed = 1;
        }
        else
        {
            SpeedCunt++;
            //IsChangeSpeed = 0;
        }
#if(AUTORUN)
        if (SpeedCunt ==8)
        {
            switch(Gear_Status_OLD)
            {
                case _LOCK:
                    Gear_Status_NEW = _UNLOCK;
                    break;
                case _UNLOCK:
                    Gear_Status_NEW = _LOCK;
                    break;
            }
        }
#endif //end of AUTORUN
//轉速為0時
		if (_1S_CNT == 0)			
		{
			RPM_Zero = 1 ;
			//LED3=1; 
		}
		else				
		{	_1S_CNT--;
			RPM_Zero = 0 ;
			//LED3=0;	
		}
  	
  	
			
//	5秒Time out旗標		
		if (Work_status == 1)
		{
			if (_5S_CNT == 0)
			{
				Error_Flag = 1;
				//_5S_Flage_Error=1;
				Error_Mode = 1;
				Pull_Error = 1; 
				//LED2 = 1;
			}
			else				
			{	_5S_CNT--;
		
			}
		}
		
		//錯誤模式下啟動拉纜計數
		
		if (Pull_Error == 1)
		{
			if (Pull_5S_CNT == 0)
			{
				Pull = 1;
			}
			else				
			{	
				Pull_5S_CNT--;
				Pull = 0;
//				if(Pull == 3)
//					Pull = 0;
			}
			
		}
		DelayTime_Count ++;
		LED1_Count ++;
		LED2_Count ++;
		LED3_Count ++;
		LED13_Count ++;
	}
	
}



/******************************************************************************
*   
*
*No_Feedback_EN START   不使用回授控制
*
*
******************************************************************************/	 	
void main(void)
{
	OSCCON = 0x70 ;								//8Mhz 
	
	while(!HFIOFR);								//INTOSC ready
//	while(!PLLR);               //PLL ready
																
	//ANSELA = 0b00010000 ;		
	ANSELA = 0b00010000 ;					//max2014
	ANSELB = 0x00 ;		
	ANSELD = 0x00 ;	
	ANSELE = 0x00 ;		
	
	LATA = 0;
	LATB = 0;
	LATC = 0;
	LATD = 0;
	
	
	TRISA = 0b00101011;
	TRISB = 0b00100001;						//RB0 = RA2 input
	//TRISC = 0b00011111;
	TRISC = 0b01001111;

	TRISBbits.TRISB0 = 0;
	
#if(_4WD_Test_EN)
	TRISD = 0b00110111;
#else
	TRISD = 0b00110011;
#endif


	TRISE = 0b00000000;						//RE0,RE1,RE2由input改為output
	
	INTCON = 0b11000000;					//GIE & PEIE
	//T1G_RPM_Init();
	//T0
	IOCBF5 = 0;
	IOCBP5 = 1;
    IOCBN5 = 0;
	//IOCBP5 = 0;
	//IOCBN5 = 1;
	IOCIF = 0;
	IOCIE = 1;

	OPTION_REG = 0b11000001;				//1:4
	TMR0 = 0;
    TMR0IF = 0;

	//T2
	T2CON = 0b01111011;
	TMR2IF = 0;
	TMR2IE = 1;
	TMR2 = 0;
	PR2 = 250 - 1;
	T2_Start();

	INTEDG = 0;
	INTF = 0;
	INTE = 0;
	
/******************************************************************************
*   
*定位CHECK
*
******************************************************************************/		
#if(Front_Position_EN)
	Front_Position();
#endif

#if(Back_Position_EN)
	Back_Position();
#endif

#if(_2WD_Position_EN)
	_2WD_Position();
#endif

#if(_4WDL_Position_EN)
	_4WDL_Position();
#endif		
												
	Check_Motor_Status();
	Check_Hand_Status();
    IsStart = 1;
    IsChangeSpeed = 1;
	switch(Gear_Status_NEW)
	{
				case _LOCK:
				//		 L1_Out = 0; L2_Out = 1; L3_Out = 1;
						 Handback_Error = 0;
						 break;	
				case _UNLOCK:
				//		 L1_Out = 1; L2_Out = 0; L3_Out = 1;	
					 	 Handback_Error = 0;
					 	 break;
				default:
						 Handback_Error = 1;
	}			
//	Gear_Status_OLD = Gear_Status_NEW;														//將開機後初始狀態儲存
	if( Error_Mode == 1)
	{	
//		LED1 = 1;
//		while(1);
		Special = 1;
		//Pull = 1;
		Gear_Status_NEW = _UNLOCK;
	}
		
	
/******************************************************************************
*********************************LOOP 迴圈開始*********************************
******************************************************************************/		
	
	while(1)
	{	
		//IOCBP5 = 0;
		//IOCBN5 = 1;
        IOCBP5 = 1;
		IOCBN5 = 0;

		if (Special == 1)												//開機第一次會做
		{
			Special = 0;
		}
		else
		{	
#if(!AUTORUN)
			Check_Hand_Status();
#endif // end of AUTORUN
			Check_Motor_Status();	
		}

		ADC_Func();			

/******************************************************************************
*   
*LED 顯示控制區
*
******************************************************************************/			
#if(LED_EN)		
	
	//if(Handback_Error == 1) 	{ LED1_FLASH(3);  }
	//轉速為"0"、接地或轉速<1600rpm
		if(RPM_Zero == 1 || Over_Speed_Error == 1)					
		{	
			LED3_FLASH(3); 
		}
		else
		{
			LED3 = 0;
		}
	
	
		if((Error_Mode  == 1)|| (Handback_Error == 1))	{	LED2_FLASH(1); } else{ LED2 = 0; }
		if(Voltage_Error  == 1)	{	LED1 = 1; } else{ LED1 = 0; }       //暫時關閉
		
#endif

		while( Voltage_Error == 1)
		{
			ADC_Func();
		}
		
		if(((IsChangeSpeed) && (Voltage_Error == 0) && (Over_Speed_Error ==0)))	
		{	
			if(Pull_Error == 1 && Pull_Count < Pull_Value)												//錯誤模式下
			{	
                if( Pull ==1)
				{	
					Pull_Count ++;
					switch(Gear_Status_OLD)
					{
						case _LOCK:
								 Error_Mode_Func(_LOCK,Motor_LOCK_Status);	
						break;
						case _UNLOCK:
								 Error_Mode_Func(_UNLOCK,Motor_UNLOCK_Status);	
						break;		
					}
					Pull_5S_CNT = Pull_Count_Val;
				}
				
			}															   	
			if (Gear_Status_NEW != Gear_Status_OLD)			 //把手狀態
			{
                Pull_Error = 0;														 //把手有變化，拉的次數重新計數
				Pull_Count = 0;
				Pull_5S_CNT = Pull_Count_Val;
				switch(Gear_Status_NEW)
				{
					case _LOCK:
							 if(IsChangeSpeed)
							 {
							 //	L1_Out = 1; L2_Out = 0; L3_Out = 1;								//2WL輸出給ECU為"010"
							 	Change_Func(_LOCK,Motor_LOCK_Status);	
								Gear_Status_OLD = Gear_Status_NEW;				 		 
							 }
					break;
					case _UNLOCK:
							if(IsChangeSpeed)
							{
								
							//	L1_Out = 1; L2_Out = 0; L3_Out = 0;	
							 	Change_Func(_UNLOCK,Motor_UNLOCK_Status);
								
							}
					break;
				}
                
                switch(Motor_Temp)  //1221修改
                {		
                    case Motor_UNLOCK_Status :
                             L1_Out = 1; L2_Out = 0; L3_Out = 0;	//0126修改
                             break;
                    case Motor_LOCK_Status :
                             L1_Out = 0; L2_Out = 0; L3_Out = 0;    //0126修改	
                             break;
                }
			}
			Check_Status();
            Output_ECU();
			
		}
	}
}	


/******************************************************************************
*    Check_Hand_Status
******************************************************************************/

void Check_Hand_Status(void)
{ 
	unsigned char Loop = 1, k = 3;
	do
	{	Delay_128msec(1);
		Gear_Status_NEW = PORTD & 0b00100000;
		
		switch(Gear_Status_NEW)
		{
					case _LOCK:
					case _UNLOCK:
							 Handback_Error = 0;
							 Loop = 0;
							 break;
					default:
							 Handback_Error = 1;
							 k--;
							 if( k== 0)
							 {	
							   Loop = 0;
							 }	
							 
		}
	}
	while(Loop == 1);
					
}
/******************************************************************************
*    Check_Status
******************************************************************************/

void Check_Status(void)
{				
	                                       
	switch(Gear_Status_NEW)
	{
                case _LOCK:
						 if(Motor_Temp == Motor_LOCK_Status )
						 {		
						 	Error_Mode = 0;
						 	Pull_Error = 0;
						 }	
						 break;
				case _UNLOCK:
						 if(Motor_Temp == Motor_UNLOCK_Status )
						 {		
						 	Error_Mode = 0;
						 	Pull_Error = 0;
						 }
						 break;
				default :
						 Error_Mode = 1;
						 Pull_Error = 1;
	}
}	

/******************************************************************************
*    Error_Exit_Func
******************************************************************************/
void Error_Exit_Func(void)
{
	Motor1_S();
	Motor2_S();
	Motor1_Y_out = 0;
	Motor1_W_out = 0;
	Motor2_Y_out = 0;
	Motor2_W_out = 0;
	Work_status =  0;
//	Front_Error = 0;
//	Back_Error = 0;
	Error_Flag =0; 	
	
} 	
/******************************************************************************
*    Change_Func
******************************************************************************/

void Change_Func(unsigned char Goto,unsigned char Status)
{
	
  Moving_Status = Status;
  _5S_CNT = _5S_Val;															
  Work_status = 1;
  ADC_Func();
  Front_Error = 0 ;
  Back_Error = 0 ;
  
    switch(Goto)
    {
        case _LOCK :
/////////////////////////LOCK後差馬達/////////////////////////////////////////////////////////////
/////////////////////////LOCK後差馬達/////////////////////////////////////////////////////////////						
            Motor2_W_out = 1;
            Work_status = 1;
            _5S_CNT = _5S_Val;
            while((Motor2_B_in == 1) && (Back_Error == 0) ) 
            { Motor2_R();	                                        
                if (Error_Flag == 1 )             
                {	           
                    Back_Error = 1 ;                
                    Error_Exit_Func();                                 
                }                                 
            } 
            Back_Error = 0 ;
            Error_Exit_Func();
        break;
        case _UNLOCK:
/////////////////////////////////////2WD後差馬達/////////////////////////////////////////////////////////////	
/////////////////////////////////////2WD後差馬達/////////////////////////////////////////////////////////////					
            Work_status = 1;
            Motor2_W_out = 1;
            _5S_CNT = _5S_Val;
            for(i = 0 ; i < 2 ; i++)
            {
                while( (Back_Error == 0) && (Motor2_Y_in == 1))
                {	Motor2_F();
                    if (Error_Flag == 1 )
                    {	
                        Back_Error = 1 ;
                        Error_Exit_Func();
                    }
                }
            }
            Back_Error = 0 ;
            Error_Exit_Func();
        break;
    }
}


/******************************************************************************
*    Motor1_F
******************************************************************************/
	
void Motor1_F(void)
{
	LATB = LATB & 0b11100001;
	LATBbits.LATB4 = 1;
}

/******************************************************************************
*    Motor1_R
******************************************************************************/

void Motor1_R(void)
{
	LATB = LATB & 0b11100001;
	LATBbits.LATB3 = 1;
}

/******************************************************************************
*    Motor1_S
******************************************************************************/

void Motor1_S(void)
{
	LATB = LATB & 0b11100001;
}

/******************************************************************************
*    Motor2_F
******************************************************************************/

void Motor2_F(void)
{
	LATB = LATB & 0b11100001;
	LATBbits.LATB2 = 1;
}

/******************************************************************************
*    Motor2_R
******************************************************************************/

void Motor2_R(void)
{
	LATB = LATB & 0b11100001;
	LATBbits.LATB1 = 1;
}

/******************************************************************************
*    Motor2_S
******************************************************************************/

void Motor2_S(void)
{
	LATB = LATB & 0b11100001;
}

/******************************************************************************
*    T2_Start
******************************************************************************/

void T2_Start(void)
{
	TMR2 = 0;
	TMR2IF = 0;
	TMR2ON = 1;
}

/******************************************************************************
*    T2_Stop
******************************************************************************/

void T2_Stop(void)
{
	TMR2IF = 0;
	TMR2ON = 0;	
}

/******************************************************************************
*    T1G_RPM_Init
******************************************************************************/

void T1G_RPM_Init(void)
{
	T1CON = 0b00010000;			//INTRC 1:2 
	T1GCON = 0b11110000;
	TMR1H = 0;
	TMR1L = 0;
	TMR1GIF = 0;
	TMR1GIE = 1;
	TMR1ON = 1;
	T1GGO = 1;
}

/******************************************************************************
*    Error_Mode_Func
******************************************************************************/
void Error_Mode_Func(unsigned char Goto,unsigned char Status)
{
	
  Moving_Status = Status;
  _5S_CNT = _5S_Val;															
  Work_status = 1;
  ADC_Func();
  Front_Error = 0 ;
  Back_Error = 0 ;

	switch(Goto)
    {
        case _LOCK :
/////////////////////////2WL後差馬達/////////////////////////////////////////////////////////////
/////////////////////////2WL後差馬達/////////////////////////////////////////////////////////////						
            Motor2_W_out = 1;
            Work_status = 1;
            _5S_CNT = _5S_Val;
                while((Motor2_B_in == 1) && (Back_Error == 0) ) 
                { Motor2_R();	                                        
                    if (Error_Flag == 1 )             
                    {	           
                        Back_Error = 1 ;                
                        Error_Exit_Func();                                 
                    }                                 
                } 
            Back_Error = 0 ;
            Error_Exit_Func();
    break;
        case _UNLOCK:
/////////////////////////////////////2WD後差馬達/////////////////////////////////////////////////////////////	
/////////////////////////////////////2WD後差馬達/////////////////////////////////////////////////////////////					
            Work_status = 1;
            Motor2_W_out = 1;
            _5S_CNT = _5S_Val;
            for(i = 0 ; i < 2 ; i++)
            {
                while( (Back_Error == 0) && (Motor2_Y_in == 1))
                {	Motor2_F();
                    if (Error_Flag == 1 )
                    {	
                        Back_Error = 1 ;
                        Error_Exit_Func();
                    }
                }
            }
            Back_Error = 0 ;
            Error_Exit_Func();
        break;
    }
			

}


/******************************************************************************
*    Delay_128msec
******************************************************************************/

void Delay_128msec(unsigned int Time)
{	
	DelayTime_Count=0;
	while(DelayTime_Count < Time );                       
//	{
//		NOP();
//	}
}
/******************************************************************************
*    LED1_FLASH
******************************************************************************/

void LED1_FLASH(unsigned int Time)
{	

	if(LED1_Count >= Time)
	{	LED1_Count =0;
		LED1 =!LED1;
	}
				

	 
}

/******************************************************************************
*    LED2_FLASH
******************************************************************************/

void LED2_FLASH(unsigned int Time)
{	
//	LED2 = 1;
//	Delay_128msec(Time);
//	LED2 = 0;
	//Delay_128msec(Time);
	if(LED2_Count >= Time)
	{	LED2_Count =0;
		LED2 =!LED2;
	}
}

/******************************************************************************
*    LED3_FLASH
******************************************************************************/

void LED3_FLASH(unsigned int Time)
{	
	if(LED13_Count >= Time)
	{	LED13_Count =0;
		LED3 =!LED3;
	}
}

/******************************************************************************
*    LED13_FLASH
******************************************************************************/

void LED13_FLASH(unsigned int Time)
{	
	if(LED13_Count >= Time)
	{	LED13_Count =0;
		LED1 =!LED1;
		LED3 =!LED3;
	}
	 
}

/******************************************************************************
*    ReadFeedback
******************************************************************************/

void ReadFeedback(void)
{	
	TD_CNT = 0;
	do
	{
		TD_Temp = PORTE & 0x07;     					//變速箱齒輪位置 
		if (TD_Temp != TD_OLD_Status)
		{
			TD_OLD_Status = PORTE & 0x07;     	//變速箱齒輪位置 	
			TD_CNT = 0;
		}
		else
		{
			TD_CNT++;
		}
	}
	while(TD_CNT < 200);
}

/******************************************************************************
*    Front_Position
******************************************************************************/

void Front_Position(void)
{
	//Check_Motor_Status();	
		Motor1_W_out = 1;
		for(i = 0 ; i < 200 ; i++);
		for(i = 0 ; i < 200 ; i++);
		//if(Motor_4WDLOCK_Gear == 1)
		if(Motor1_Y_in == 1 && Motor1_B_in == 1 && Motor1_W_in == 0)										//4WL位置
		{	LED1 =1;
			
			for(i = 0 ; i < 2 ; i++)
			{
				while(Motor1_Y_in == 1)
				{
					Motor1_R();
				}
			}
			Motor1_S();
			
								
		}
		//else if(Motor_2WDLOCK_Gear == 1 || Motor_2WD_Gear ==1)
		else if (Motor1_Y_in == 1 && Motor1_B_in == 0 && Motor1_W_in == 0)							//2WD/2WL位置
		{	LED2 =1;
			
			for(i = 0 ; i < 2 ; i++)
			for(i = 0 ; i < 2 ; i++)
			{
				while(Motor1_Y_in == 1)
				{
					Motor1_F();
				}
			}
			Motor1_S();
			
		}
		else
		{	LED3 =1;
			
			
		}
		Motor1_W_out = 0;
		while(1);
}

/******************************************************************************
*    Back_Position
******************************************************************************/

void Back_Position(void)
{
	while(1)
	{	Motor2_W_out = 1;
		for(i = 0 ; i < 200 ; i++);
		if(Motor2_Y_in == 1)
		{
			Motor2_F();				
			for(i = 0 ; i < 2 ; i++)
			{
				while(Motor2_Y_in == 1)
				{
					NOP();
				}
			}
			Motor2_S();
		}		
		Motor2_W_out = 0;
	}
}	

/******************************************************************************
* _2WD_Position
******************************************************************************/
void _2WD_Position(void)
{
		Motor1_W_out = 1;
		for(i = 0 ; i < 200 ; i++);
		for(i = 0 ; i < 200 ; i++);
		
		
			
		Motor1_R();
		for(i = 0 ; i < 2 ; i++)
		{
			while(Motor1_B_in == 1);
//			
		}
		Motor1_S();
		Motor1_W_out = 0;
		while(1);						
}	

/******************************************************************************
* _4WDL_Position
******************************************************************************/
void _4WDL_Position(void)
{
	Motor1_Y_out = 1;
	for(i = 0 ; i < 200 ; i++);
	for(i = 0 ; i < 200 ; i++);
	if(Motor1_B_in == 1)
	{	Motor1_F();
		for(i = 0 ; i < 2 ; i++)
		{
			while(Motor1_B_in == 1);
		}
		Motor1_S();
	}	
		Motor1_Y_out = 0;
		while(1);
}	


/******************************************************************************
*    Check_Motor_Status
******************************************************************************/

void Check_Motor_Status(void)
{	
	Motor_Temp							= 0;
	
	Motor2_W_out = 1;
    for(i = 0 ; i < 200 ; i++);
    for(i = 0 ; i < 200 ; i++);
    Motor_Back_Status =  PORTC & 0x07;						//後差RC0/W,RC1/B,RC2/Y

    Motor_Temp = Motor_Back_Status;
    Motor1_W_out = 0;
    Motor2_W_out = 0;
	switch( Motor_Temp )
 	{
 		case Motor_UNLOCK_Status :	
 				 Gear_Status_OLD = _UNLOCK;
 				 Error_Mode = 0;
 				 break;
 		case Motor_LOCK_Status :
 				 Gear_Status_OLD = _LOCK;
 				 Error_Mode = 0;
 				 break;
 		default:
  				 Error_Mode = 1;
 	}		
}
				

/******************************************************************************
*    Output_ECU
******************************************************************************/
void Output_ECU(void)
{	
	
	if(	Handback_Error == 1)
	{
		L1_Out = 1; L2_Out = 1; L3_Out = 1;
        //LED1 = 1;
	}	
	else
	{	
		switch(Motor_Temp)  
		{		
			case Motor_UNLOCK_Status :
					 L1_Out = 1; L2_Out = 0; L3_Out = 0;	
					 break;
			case Motor_LOCK_Status :
					 L1_Out = 0; L2_Out = 0; L3_Out = 0;	
					 break;
        }
	}
}
