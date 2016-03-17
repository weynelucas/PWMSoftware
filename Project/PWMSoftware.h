// PWMSoftware.c
// Runs on LM4F120/TM4C123
// A library to perform a software PWM on ARM Cortex-M4 Microncontroller (TM4C123 - Tiva C Series Launchpad)
// Lucas Weyne 
//		weynelucas@gmail.com
//		https://gitlab.com/u/weynelucas
// 		https://github.com/weynelucas
// March 13, 2016

/* 
	Specifications:
		* The port must be initialized and PWM pin congigured as output before initialize this library
		* Using this library, you cannot use SysTick Timer in your project because is used by PWMSoftware
		* Periodic interrupts has priority 2
	
	References:
		* Valvano, Jonathan W. Embedded Systems: Introduction to Arm Cortex-M Microcontrollers.  5th Edition, 2014.
*/

//*****************************************************************************
//
// PWM Periods
// period * (1/Bus Clock Frequency)
//
//*****************************************************************************

#define PWM_1KHZ_SYSCLK_80MHZ			80000					// PWM period for 1KHz frequency with 80MHz bus clock
#define PWM_1KHZ_SYSCLK_50MHZ			50000					// PWM period for 1KHz frequency with 50MHz bus clock
#define PWM_1KHZ_SYSCLCK_16MHz		16000					// PWM period for 1KHz frequency with 16MHz bus clock

//*****************************************************************************
//
// GPIO pins mask (PORTA)
//
//*****************************************************************************
#define PA0					0x01
#define PA1					0x02
#define PA2					0x04
#define PA3					0x08
#define PA4					0x10
#define PA5					0x20
#define PA6					0x40
#define PA7					0x80

//*****************************************************************************
//
// GPIO pins mask (PORTB)
//
//*****************************************************************************
#define PB0					0x01
#define PB1					0x02
#define PB2					0x04
#define PB3					0x08
#define PB4					0x10
#define PB5					0x20
#define PB6					0x40
#define PB7					0x80

//*****************************************************************************
//
// GPIO pins mask (PORTC)
//
//*****************************************************************************
#define PC4					0x10
#define PC5					0x20
#define PC6					0x40
#define PC7					0x80

//*****************************************************************************
//
// GPIO pins mask (PORTD)
//
//*****************************************************************************
#define PD0					0x01
#define PD1					0x02
#define PD2					0x04
#define PD3					0x08
#define PD4					0x10
#define PD5					0x20
#define PD6					0x40
#define PD7					0x80

//*****************************************************************************
//
// GPIO pins mask (PORTE)
//
//*****************************************************************************
#define PE0					0x01
#define PE1					0x02
#define PE2					0x04
#define PE3					0x08
#define PE4					0x10
#define PE5					0x20

//*****************************************************************************
//
// GPIO pins mask (PORTF)
//
//*****************************************************************************
#define PF0					0x01
#define PF1					0x02
#define PF2					0x04
#define PF3					0x08
#define PF4					0x10


// Struct to represent PWM pin
typedef struct {
	volatile unsigned long *GPIO_Data_Addr;			// adress of Port data register 
	unsigned long GPIO_Pin_Mask;								// mask of pin
} Port;


// **************PWMSoftware_Init************************
// Initialize library configure periodic interrupt
// Input:  Base address of GPIO Port
//				 Mask of the pin to perform the software PWM
//				 Period of PWM (depends of the bus clock)
//					 		period = Bus clock frequency / PWM frequency
// Output: None
void PWMSoftware_Init(volatile unsigned long *GPIO_Port_Base_Addr, unsigned long GPIO_Pin_Mask, unsigned long period);

// **************PWMSoftware_SetDuty*********************
// Configure duty cycle of PWM
// Input:  Duty cycle in percentage (0 - 100)
// Output: None
void PWMSoftware_SetDuty(unsigned char duty);
