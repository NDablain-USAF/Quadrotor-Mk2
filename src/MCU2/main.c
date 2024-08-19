#include "main.h"

int main(void){
	SPI_Init();
	LoRa_Init();
    while(1){
		LoRa_Read();
    }
}
