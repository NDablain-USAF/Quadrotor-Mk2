#include "main.h"

int main(void){
	AVR_Setup();
	BAR barometer = {0};
	BAR_Setup(&barometer);
    while(1){
        __builtin_avr_nop();
    }
	return 0;
}
