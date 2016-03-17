// PWMSoftwareTestMain.c
// Runs on LM4F120/TM4C123
// Software to exemplify PWMSoftware library usage
// Lucas Weyne 
//		weynelucas@gmail.com
//		https://gitlab.com/u/weynelucas
// 		https://github.com/weynelucas
// March 13, 2016


#include "PLL.h"
#include "PWMSoftware.h"
#include "tm4c123gh6pm.h"


void PortF_Init(void);

int main(void){
	PLL_Init();
	PortF_Init();
	PWMSoftware_Init(&GPIO_PORTF_DATA_R, PF1, PWM_1KHZ_SYSCLK_50MHZ);
	PWMSoftware_SetDuty(40);
	
	while(1){
		// Other stuffs
  }
}


void PortF_Init(void){volatile unsigned long delay;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;		// activate clock for Port F
	delay = SYSCTL_RCGC2_R;									// allow time for clock to stabilize
	GPIO_PORTF_AFSEL_R &= ~0x02;						// disable alternate function for PF1
	GPIO_PORTF_PCTL_R &= ~0x000000F0;				// clear PCTL bits toselect regular function
	GPIO_PORTF_DIR_R |= 0x02;								// select PF1 as output
	GPIO_PORTF_AMSEL_R &= ~0x02;						// disable analog mode on PF1
	GPIO_PORTF_DEN_R |= 0x02;								// enable digital I/O PF1
}
