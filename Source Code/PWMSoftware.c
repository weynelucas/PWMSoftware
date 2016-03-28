// PWMSoftware.c
// Runs on LM4F120/TM4C123
// A library to perform a software PWM on ARM Cortex-M4 Microncontroller (TM4C123 - Tiva C Series Launchpad)
// Repository 
// 		https://github.com/weynelucas/PWMSoftware
// Lucas Weyne 
//		weynelucas@gmail.com
// 		https://github.com/weynelucas
// March 13, 2016

/* 
	Specifications:
		* The port must be initialized and PWM pin congigured as output before initialize this library
		* Use enum constants defined on header file to select wich pin you want to use
		* Using this library, you cannot use SysTick Timer in your project because is used by PWMSoftware
		* Periodic interrupts has priority 2
	
	References:
		* Valvano, Jonathan W. Embedded Systems: Introduction to Arm Cortex-M Microcontrollers.  5th Edition, 2014.
*/

#include "PWMSoftware.h"

// ***********************************************************************
//	SysTick registers
// ***********************************************************************
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value


// ***********************************************************************
//	GPIO ports (base address)
// ***********************************************************************
#define GPIO_PORTA_BASE		0x40004000
#define GPIO_PORTB_BASE		0x40005000
#define GPIO_PORTC_BASE		0x40006000
#define GPIO_PORTD_BASE		0x40007000
#define GPIO_PORTE_BASE		0x40024000
#define GPIO_PORTF_BASE		0x40025000

// ***********************************************************************
//	GPIO pins offset (bit-specific addressing)
// ***********************************************************************
#define BIT0_OFFSET		0x0004
#define BIT1_OFFSET		0x0008
#define BIT2_OFFSET		0x0010
#define BIT3_OFFSET		0x0020
#define BIT4_OFFSET		0x0040
#define BIT5_OFFSET		0x0080
#define BIT6_OFFSET		0x0100
#define BIT7_OFFSET   0x0200

// ***********************************************************************
//	Port and pin manipulation (auxiliar macros)
// ***********************************************************************
#define GetPinReg(pin) 		((volatile unsigned long *) (PortBaseAddress[(pin&0xF00)>>8] + PinAddressOffset[pin&0x0FF]))
#define GetPortDataReg(port)	((volatile unsigned long *) (PortBaseAddress[port] + 0x03FC))
#define GetPinMask(pin)		(1<<(pin&0x0FF))

// Ports base address table
const unsigned long PortBaseAddress[6] = {
	GPIO_PORTA_BASE,
	GPIO_PORTB_BASE,
	GPIO_PORTC_BASE,
	GPIO_PORTD_BASE,
	GPIO_PORTE_BASE,
	GPIO_PORTF_BASE
};

// Pins offset address table (bit-specific addressing)
const unsigned long PinAddressOffset[8] = {
	BIT0_OFFSET,
	BIT1_OFFSET,
	BIT2_OFFSET,
	BIT3_OFFSET,
	BIT4_OFFSET,
	BIT5_OFFSET,
	BIT6_OFFSET,
	BIT7_OFFSET
};

// Struct to represent GPIO pin
typedef struct{
	volatile unsigned long *addr;
	unsigned char mask;
}GPIOPin;

// Global variables
GPIOPin PWMPin;			// struct to represent pin to perform PWM
unsigned long High;		// duration of high phase 
unsigned long Low;		// duration of low phase    
unsigned long PWMPeriod;   	// PWM period count (High + Low = PWMPeriod)


// **************PWMSoftware_Init************************
// Initialize library with 0% duty cycle
// Input: Pin to perform software PWM
//	  Period of PWM (depends of the bus clock)
//		  * Period = Bus clock frequency / PWM frequency
// Output: None
void PWMSoftware_Init(Pin pin, unsigned long period){
	PWMPin.addr = GetPinReg(pin);
	PWMPin.mask = GetPinMask(pin);
	
	PWMPeriod = period;
	PWMSoftware_SetDuty(0);			// initialize 0% of duty cycle 	
	
	// Initialize SysTick Timer
	NVIC_ST_CTRL_R = 0;								// clear SysTick during initialization
	NVIC_ST_RELOAD_R = (Low - 1)&0x00FFFFFF;					// set reload value		
	NVIC_ST_CURRENT_R = 0;								// any value written to current clears it
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; 			// priority 2  
	NVIC_ST_CTRL_R = (NVIC_ST_CTRL_CLK_SRC|NVIC_ST_CTRL_ENABLE|NVIC_ST_CTRL_INTEN); // enable SysTick with core clock and interrupts
}

// **************PWMSoftware_SetDuty*********************
// Configure duty cycle of PWM
// Input:  Duty cycle in percentage (0 - 100)
// Output: None
void PWMSoftware_SetDuty(unsigned char duty){
	if(duty>100) return;
	
	if(duty==0){
		Low = PWMPeriod;
	} else if(duty==100){
		Low = 0;
	} else{
		Low = (unsigned long) ((double) duty/100 * PWMPeriod);
	}
	
	High =  PWMPeriod - Low;
}

void SysTick_Handler(void){
	// Toggle PWM pin
	if(*PWMPin.addr&PWMPin.mask){
		if(Low){																					
			*PWMPin.addr &= ~PWMPin.mask; 	// make PWM pin low
			NVIC_ST_RELOAD_R = Low - 1;	// reload value for low phase
		}

	} else{
		if(High){
			*PWMPin.addr |= PWMPin.mask; 	// make PWM pin high
			NVIC_ST_RELOAD_R = High - 1;	// reload value for high phase
		}
	}
}
