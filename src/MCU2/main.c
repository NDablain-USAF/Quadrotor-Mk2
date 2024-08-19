#include "main.h"

int main(void){
	I2C_Init();
	SPI_Init();
	LoRa_Init();
    while(1){
		unsigned char data = 187;
		LoRa_Read();
		if ((TWCR&(1<<TWINT))==(1<<TWINT)){
			unsigned char status = I2C_Slave_Transmit(data);
		}
    }
}
