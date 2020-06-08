#ifndef _main_h
#define _main_h

//#define TEST
#define SERIALDEBUG

#include "stm32f3xx.h"
#include "arm_math.h"

#ifdef SERIALDEBUG
#include "stdio.h"
#include "stdarg.h"
#endif

#define OUTPUT		0x01UL 
#define AF 		    0x02UL
#define ANALOG		0x03UL
#define PULL_UP 	0x01UL
#define PULL_DOWN 	0x02UL
#define MS		    0x01UL
#define HS		    0x02UL

#define MOTORPULSELEN 	200
#define WAVELEN		    10000

#define HALLDELTAMAX	60000

#define MINRPM		4000

#define BUFFSIZE	255

#define NumbRegA	0x0950		//0b0000B00A0D0G0000	GPIOA = 0950
#define NumbRegB	0x5082		//0b0C0p0000F00000E0	GPIOB = 5082
const uint32_t NumbRegAset[10] = 
		{0x0940, 0x0800, 0x0950, 0x0950, 0x0810, 0x0150, 0x0150, 0x0900, 0x0950, 0x0950};
const uint32_t NumbRegBset[10] = 
		{0x4082, 0x4000, 0x0002, 0x4000, 0x4080, 0x4080, 0x4082, 0x4000, 0x4082, 0x4080};
		
void RCC_Init(void);
void TIM_Init(void);
void EXTI_Init(void);
void UART_Init(void);
void display_Init(void);

void buttonTask(void);
void PIDcalculate(void);
void printValue(void);		
void displayPrint(uint8_t number, uint8_t dpPosition);

volatile uint8_t PulseAllowed = 0;
volatile uint32_t MotorPulse = 0;
volatile uint32_t HallDeltaTime = HALLDELTAMAX;
uint32_t RPMset = 0;

arm_pid_instance_f32 PID;
int32_t PIDoutputMax;
int32_t PIDoutputMin;
void PIDreset(void);
volatile uint8_t PIDcalculateFlag = 0;
uint8_t SoftStartFlag = 0;
		
uint8_t Mode = 0;   // Printing value by default mode(0: RPM)
		
uint16_t KF = 1000; // Second zero crossing time tuning

#ifdef SERIALDEBUG
void USARTprint(char* format, ...);
void UARTdataSend(char* str);
char rxBuff[BUFFSIZE+1];
char txBuff[BUFFSIZE+1];
uint8_t UARTonFlag = 1; 
#endif

#endif

/*
//0b0000B00A0D0G0000//GPIOA = 0950	//7-segment pinout
//0b0C0p0000F00000E0//GPIOB = 5082
                                                a
//0b0000100101000000//0x0940		//0      -------
//0b0100000010000010//0x4082				|	    |
							                |f	    |b
//0b0000100000000000//0x0800		//1		|   g   |
//0b0100000000000000//0x4000				 -------
							                |	    |
//0b0000100101010000//0x0950		//2		|e	    |c
//0b0000000000000010//0x0002				|	    |
							                 -------  () dp
//0b0000100101010000//0x0950		//3		    d
//0b0100000000000000//0x4000

//0b0000100000010000//0x0810		//4
//0b0100000010000000//0x4080
  
//0b0000000101010000//0x0150		//5
//0b0100000010000000//0x4080
  
//0b0000000101010000//0x0150		//6
//0b0100000010000010//0x4082
  
//0b0000100100000000//0x0900		//7
//0b0100000000000000//0x4000
  
//0b0000100101010000//0x0950		//8
//0b0100000010000010//0x4082
  
//0b0000100101010000//0x0950		//9
//0b0100000010000000//0x4080
*/
