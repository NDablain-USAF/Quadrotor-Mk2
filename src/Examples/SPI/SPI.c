#include "spi.h"

int main(void){
	SPI_Init();
	while (1) {
		int x = -1234;
		Transmit((long)x);
	}
}
