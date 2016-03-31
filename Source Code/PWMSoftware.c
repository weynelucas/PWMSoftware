// PWMSoftware.c
// Runs on LM4F120/TM4C123
// A library to perform a software PWM on ARM Cortex-M4 Microncontroller (TM4C123 - Tiva C Series Launchpad)
// Repository 
// 		https://github.com/weynelucas/PWMSoftware.git
// Lucas Weyne 
//		weynelucas@gmail.com
// 		https://github.com/weynelucas
// March 13, 2016

/* 
	Specifications:
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
//	Run Mode Clock Gating Control Register 2
// ***********************************************************************
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))

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
#define BIT7_OFFSET   		0x0200

// ***********************************************************************
//	Port and pin manipulation (auxiliar macros)
// ***********************************************************************
#define GetPinReg(pin) 		((volatile unsigned long *) (PortBaseAddress[(pin&0xF00)>>8] + PinAddressOffset[pin&0x0FF]))
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
	unsigned char pinNumber;
	unsigned char portNumber;
}GPIOPin;

// Struct to represent GPIO port
typedef struct{
	volatile unsigned long BASE_ADDRESS;
	volatile unsigned long *AFSEL;
	volatile unsigned long *AMSEL;
	volatile unsigned long *PCTL;
	volatile unsigned long *DEN;
	volatile unsigned long *DIR;
} GPIOPort;

// Global variables
GPIOPin PWMPin;			// struct to represent pin to perform PWM
GPIOPort PWMPort;		// struct to represent registers of PWM port
unsigned long High;		// duration of high phase 
unsigned long Low;		// duration of low phase    
unsigned long PWMPeriod;   	// PWM period count (High + Low = PWMPeriod)


// ***********************************************************************
//	Private functions prototypes
// ***********************************************************************
void Port_Init(void);

// **************PWMSoftware_Init************************
// Initialize library with 0% duty cycle
// Input: Pin to perform software PWM
//	  Period of PWM (depends of the bus clock)
//		  * Period = Bus clock frequency / PWM frequency
// Output: None
void PWMSoftware_Init(Pin pin, unsigned long period){
	PWMPin.addr = GetPinReg(pin);
	PWMPin.mask = GetPinMask(pin);
	PWMPin.pinNumber = pin&0x0FF;
	PWMPin.portNumber = (pin&0xF00)>>8;
	
	// Get GPIO registers
	PWMPort.BASE_ADDRESS = PortBaseAddress[PWMPin.portNumber];
	PWMPort.AFSEL = (volatile unsigned long *) (PWMPort.BASE_ADDRESS + GPIO_AFSEL_OFFSET);
	PWMPort.AMSEL = (volatile unsigned long *) (PWMPort.BASE_ADDRESS + GPIO_AMSEL_OFFSET);
	PWMPort.PCTL = (volatile unsigned long *) (PWMPort.BASE_ADDRESS + GPIO_PCTL_OFFSET);
	PWMPort.DIR = (volatile unsigned long *) (PWMPort.BASE_ADDRESS + GPIO_DIR_OFFSET);
	PWMPort.DEN = (volatile unsigned long *) (PWMPort.BASE_ADDRESS + GPIO_DEN_OFFSET);
	
	Port_Init();
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


// ***********************************************************************
//	Private functions implementations
// ***********************************************************************
void Port_Init(void){volatile unsigned long delay;
	SYSCTL_RCGC2_R  |= (1<<PWMPin.portNumber);		// activate clock for PWM port
	delay = SYSCTL_RCGC2_R;					// allow time for clock to stabilize
	*PWMPort.AFSEL &= ~PWMPin.mask;				// disable alternate function on PWM pin
	*PWMPort.PCTL &= ~(0x0000000F<<(PWMPin.pinNumber*4));	// clear PCTL bits on PWM pin to select regular I/O
	*PWMPort.DIR |= PWMPin.mask;				// make PWM pin output
	*PWMPort.AMSEL &= ~PWMPin.mask;				// disable analog functionality on PWM pin
	*PWMPort.DEN |= PWMPin.mask;				// Enable I/O on PWM pin
}
