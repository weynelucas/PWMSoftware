# PWMSoftware
A software PWM library for TM4C123GH6PM (ARM Cortex-M4F based microcontrollers from Texas Instruments) to perform PWM signals with any digital I/O pin of microcontroller.


##Usage
###Initialization
```c
void PWMSoftware_Init(volatile unsigned long *GPIO_Port_Base_Addr, unsigned long GPIO_Pin_Mask, unsigned long period)
```
Where:
* `GPIO_Port_Base_Addr` is the base address of GPIO Port
* `GPIO_Pin_Mask` is the mask of pin to perform PWM
* `period` is the period of PWM in bus clock counts, its depends of the system clock

For calculate the `period` value you can use the simple formula bellow:

<p align="center">
<img src="http://latex.codecogs.com/gif.latex?%5Cmathtt%7BPeriod%20%3D%20%5Cfrac%7BSystem%5C%2CClock%5C%2CFrequency%5C%2C%28Hz%29%7D%7BPWM%5C%2CFrequency%5C%2C%28Hz%29%7D%7D">
</p>

For example, if you working with 80 MHz system clock and want a 1 KHz PWM signal, the `period` value must be equal to 80000.

###Configure Duty Cycle
```c
void PWMSoftware_SetDuty(unsigned char duty)
```

Where:
* `duty` is the duty cycle of PWM in percentage, value between 0 and 100


### Code Example

The code bellow is a example to perform a pulse-width-modulation with 70% of duty cycle

```c
#include "PLL.h"
#include "PWMSoftware.h"

int main(void){
  PortF_Init();      // initialize Port F (PF1 as output)
  PWMSoftware_Init((volatile unsigned long*) 0x400253FC, 0x02, 80000);    // configure PWM on PF1 at 1 KHz
  PWM_SetDuty(70);   // set a 70% duty cycle
  
  while(1){
    // Other stuff
  }
}
```

To make your code more easy to understand is possible use the constants defined in `PWMSoftware` library 
(see all constants defined in header file `PWMSoftware.h`) and also the `tm4c123gh6pm.h`, library provided by Texas Instruments
its contais the TM4C123GH6PM Register Definitions. See bellow:

```c
#include "PLL.h"
#include "PWMSoftware.h"
#include "tm4c123gh6pm.h"

int main(void){
  PLL_Init();        // initialize system clock with 80 MHz
  PortF_Init();      // initialize Port F (PF1 as output)
  PWMSoftware_Init(&GPIO_PORTF_DATA_R, PF1, PWM_1KHZ_SYSCLK_80MHZ);    // configure PWM on PF1 at 1 KHz
  PWM_SetDuty(70);   // set a 70% duty cycle
  
  while(1){
    // Other stuff
  }
}
```
