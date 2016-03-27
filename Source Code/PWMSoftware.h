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


//*****************************************************************************
// PWM Periods
// period * (1/Bus Clock Frequency)
//*****************************************************************************
#define PWM_1KHZ_SYSCLK_80MHZ			80000					// PWM period for 1KHz frequency with 80MHz bus clock
#define PWM_1KHZ_SYSCLK_50MHZ			50000					// PWM period for 1KHz frequency with 50MHz bus clock
#define PWM_1KHZ_SYSCLCK_16MHz		16000					// PWM period for 1KHz frequency with 16MHz bus clock


// Enum constans of GPIO pins
typedef const enum{
	PA0 = 0x000, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
	PB0 = 0x100, PB1, PB2, PB3, PB4, PB5, PB6, PB7,
	PC0 = 0x200, PC1, PC2, PC3, PC4, PC5, PC6, PC7,
	PD0 = 0x300, PD1, PD2, PD3, PD4, PD5, PD6, PD7,
	PE0 = 0x400, PE1, PE2, PE3, PE4, PE5,
	PF0 = 0x500, PF1, PF2, PF3, PF4
}Pin;



// **************PWMSoftware_Init************************
// Initialize library with 0% duty cycle
// Input: Pin to perform software PWM
//				Period of PWM (depends of the bus clock)
//					* Period = Bus clock frequency / PWM frequency
// Output: None
void PWMSoftware_Init(Pin pin, unsigned long period);

// **************PWMSoftware_SetDuty*********************
// Configure duty cycle of PWM
// Input:  Duty cycle in percentage (0 - 100)
// Output: None
void PWMSoftware_SetDuty(unsigned char duty);
