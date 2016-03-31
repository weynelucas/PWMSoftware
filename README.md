# PWMSoftware
A software PWM library for TM4C123GH6PM (ARM Cortex-M4F based microcontrollers from Texas Instruments) to perform PWM signals with any digital I/O pin of microcontroller.


##Usage
###Initialization
```c
void PWMSoftware_Init(Pin pin, unsigned long period)
```
Where:
* `pin` is a pin to perform the software PWM, it is an enum constant with all pins that can be used on microcontroller
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
  PLL_Init();        // configure 80 MHz for bus clock
  PortF_Init();      // initialize Port F (PF1 as output)
  PWMSoftware_Init(PF1, 80000);    // configure PWM on PF1 at 1 KHz
  PWM_SetDuty(70);   // set a 70% duty cycle
  
  while(1){
    // Other stuff
  }
}
```

## Specifications

* Use only values presents on enum constant to select the pin to perform PWM (see on header file `PWMSoftware.h`)
* Using PWMSoftware you cannot use SysTick Timer in your code to make other stuff because is used by library for periodic interrupts
* Periodic interrupts of SysTick have priority 2, consider this if your code has others interrupts
