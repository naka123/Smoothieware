#include "mbed.h"
#include "millis.h"
 
volatile unsigned long  _millis;
 
void millisStart(void) {
    SysTick_Config(SystemCoreClock / 1000);
}
 
extern "C" void SysTick_Handler(void) {
    _millis++;
}
 
unsigned long millis(void) {
    return _millis;
}
