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

#include "tm4c123gh6pm.h"
#include "PWMSoftware.h"

Port PWMPin;								// struct to represent pin to perform PWM
unsigned long High;					// duration of high phase 
unsigned long Low;					// duration of low phase    
unsigned long PWMPeriod;   	// PWM period count (High + Low = PWMPeriod)

// **************PWMSoftware_Init************************
// Initialize library with 0% duty cycle
// Input:  Base address of GPIO Port
//				 Mask of the pin to perform the PWM
//				 Period of PWM (depends of the bus clock)
//					 		Period = Bus clock frequency / PWM frequency
// Output: None
void PWMSoftware_Init(volatile unsigned long *GPIO_Port_Base_Addr, unsigned long GPIO_Pin_Mask, unsigned long period){
	PWMPin.GPIO_Data_Addr = GPIO_Port_Base_Addr;
	PWMPin.GPIO_Pin_Mask = GPIO_Pin_Mask&0xFF;
	
	PWMPeriod = period;
	PWMSoftware_SetDuty(0);			// initialize 0% of duty cycle 	
	
	// Initialize SysTick Timer
	NVIC_ST_CTRL_R = 0;																															// clear SysTick during initialization
	NVIC_ST_RELOAD_R = (Low - 1)&0x00FFFFFF;																				// set reload value		
	NVIC_ST_CURRENT_R = 0;																													// any value written to current clears it
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; 											// priority 2  
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
	if(*PWMPin.GPIO_Data_Addr&PWMPin.GPIO_Pin_Mask){
		if(Low){																					
			*PWMPin.GPIO_Data_Addr &= ~PWMPin.GPIO_Pin_Mask; 	// make PWM pin low
			NVIC_ST_RELOAD_R = Low - 1;												// reload value for low phase
		}

	} else{
		if(High){
			*PWMPin.GPIO_Data_Addr |= PWMPin.GPIO_Pin_Mask; 	// make PWM pin high
			NVIC_ST_RELOAD_R = High - 1;											// reload value for high phase
		}
	}
}
