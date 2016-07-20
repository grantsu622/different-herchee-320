#include <pic.h>
#include "main.h"

unsigned char ADC_Val[4] = {0,0,0,0};
unsigned char ADC_Cnt = 0,ADC_Temp = 0,i,j;
unsigned int ADC_Data , FVR_Data = 0,Normal_Data = 0;

unsigned char Voltage_Error = 0;
unsigned char UnderVoltage_Error  = 0;

//unsigned int  FVR = 0,Normal=0,Over=0;
//ADC設定值需參考電壓量測表

// 放大1000倍 5V = 5000 , 4.7V = 4700 , 2V = 2000
#define OverVoltage			0xC8//3920 / ( 5000 / 256 )				//16V
//#define OverVoltage			0xC7
//#define UnderVoltage		0x60//1940 / ( 5000 / 256 )				//7.5V
//#define ADC_LowVoltage  0x64															//7.5V，ADC抓到的數值
#define ADC_LowVoltage  0x6C															//7.5V，ADC抓到的數值
#define FVR_LowVoltage  0x35															//7.5V，FVR抓到的數值
//#define OverVoltage		3880 / ( 5000 / 256 )
//#define UnderVoltage	1895 / ( 5000 / 256 )
//#define OverVoltage		3880 / ( 5000 / 256 )						//實際電壓4.07V
//#define OverVoltage		4420 / ( 5000 / 256 )
//#define UnderVoltage	2880 / ( 5000 / 256 )						//實際電壓1.798V

unsigned int Init_ADC(void)
{
	
	
	ADCON0 = 0b10010001;																//10bit取樣
	ADCON1 = 0b01110000;																//數值靠左
	ADCON2 = 0xFF;
	NOP();
	NOP();
	NOP();
	NOP();
	for(j = 0 ; j < 4 ; j++)
		{
			for(i = 0 ; i < 200 ; i++);
			for(i = 0 ; i < 200 ; i++);
			GO_nDONE = 1;
			while(GO_nDONE == 1);
				
			ADC_Val[ADC_Cnt++] = ADRESH;
			if(ADC_Cnt == 4) ADC_Cnt = 0;
		}
	
		ADC_Data = ADC_Val[0] + ADC_Val[1] + ADC_Val[2] + ADC_Val[3];
		ADC_Data = ADC_Data  >> 2;										//除4取平均
	return ADC_Data;  		
}

unsigned int FVR_Test(void)
{
	ADCON0 = 0b11111101;													//CHS<4:0> =11111  ,11111 = FVR (Fixed Voltage Reference) Buffer 1 Output
	ADCON1 = 0b01110000;													// VREF+ is connected to VDD
	ADCON2 = 0xFF;
	FVRCON = 0b11000001 ;													//ADC Fixed Voltage Reference Peripheral output is 1x (1.024V)
	NOP();
	NOP();
	NOP();
	NOP();
	
	
		for(j = 0 ; j < 4 ; j++)
		{
			for(i = 0 ; i < 200 ; i++);
			for(i = 0 ; i < 200 ; i++);
			GO_nDONE = 1;
			while(GO_nDONE == 1);
				
			ADC_Val[ADC_Cnt++] = ADRESH;
			if(ADC_Cnt == 4) ADC_Cnt = 0;
		}
		ADC_Data = ADC_Val[0] + ADC_Val[1] + ADC_Val[2] + ADC_Val[3];
		ADC_Data = ADC_Data  >> 2;								//除4取平均
		return ADC_Data;  		
	
}

void ADC_Func(void)
{	
	FVR_Data = FVR_Test();
	
	Normal_Data =	Init_ADC();
	if ((FVR_Data > FVR_LowVoltage) || (Normal_Data < ADC_LowVoltage))											//低於下限值7.5V,此為使用FVR 1.024V狀態下抓到的值，和ADC抓到的值
	{	
//		if(((FVR_Data - FVR_LowVoltage)<=5)|| ((Normal_Data - ADC_LowVoltage)<=5))						
//		{	Voltage_Error = 1;
//
//		}
		Voltage_Error = 1;
	}
	else if(FVR_Data <= FVR_LowVoltage)
	{	
		if (Normal_Data > OverVoltage)
		{	
//			if ((Normal_Data - OverVoltage)<=5)
//			{	Voltage_Error = 1;
//				
//			}
			Voltage_Error = 1;		
		}
		else
			Voltage_Error = 0;	
		
	}	
		
			
//			if (ADC_Data > OverVoltage)
//			{
//				Voltage_Error = 1;
//				//LED3 = 1;
//			}
//			else if (ADC_Data < UnderVoltage)
//			{
//				Voltage_Error = 1;
//			}
//			else
//			{	
//				Voltage_Error = 0;
				//LED3 = 0;
				
		//	}
		//}
//	}
//	while( (ADC_Data > OverVoltage)|| (ADC_Data < UnderVoltage) );
////	if(UnderVoltage_Error == 1)
////	{	LED1 = 1;
////		UnderVoltage_Error = 0;
////	}
////	else	
////	{	Voltage_Error = 0;
////		LED1 = 0;
////	}	
			
}




//void ADC_Func(void)
//{
//	ADC_Cnt=0;
//	if(GO_nDONE == 0)
//	{
//		ADC_Val[ADC_Cnt++] = ADRESH;
//		GO_nDONE = 1;
//		if(ADC_Cnt == 4) ADC_Cnt = 0;
//		if(ADC_Temp < 4)ADC_Temp++;
//	}
//	if(ADC_Temp > 4)
//	{
//		ADC_Data = ADC_Val[0] + ADC_Val[1] + ADC_Val[2] + ADC_Val[3];
//		ADC_Data = ADC_Data >> 2;
//		
//		if (ADC_Data > OverVoltage)
//		{
//			Voltage_Error = 1;
//		}
//		else if (ADC_Data < UnderVoltage)
//		{
//			Voltage_Error = 1;
//		}
//		else
//		{
//			Voltage_Error = 0;
//		}
//	}
//		
//}