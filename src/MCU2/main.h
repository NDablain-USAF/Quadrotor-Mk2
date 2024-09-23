#ifndef MAIN_H
#define MAIN_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>

#define SS_ICE_PIN PINB2
#define SS_LORA_PIN PIND6
#define SS_BMP_PIN PINB0

#define LORA_REG_OP_MODE 0x01
#define LORA_REG_F_MSB 0x06
#define LORA_REG_F_MIDB 0x07
#define LORA_REG_F_LSB 0x08
#define LORA_REG_RX_N_BYTES 0x13
#define LORA_REG_FIFO_ADR_PTR 0x0D
#define LORA_REG_RX_ADR 0x10
#define LORA_REG_FIFO 0x00
#define LORA_REG_PAYLOAD_LENGTH 0x22
#define LORA_REG_PA_CONFIG 0x09

#define BMP_ID 0x00
#define BMP_PWR 0x1B
#define BMP_OSR 0x1C
#define BMP_ODR 0x1D
#define BMP_INT 0x19
#define BMP_CONFIG 0x1F
#define BMP_DATA 0x04
#define BMP_CAL_DATA 0x31
#define BMP_CON1 0.00390625f // 2^-8
#define BMP_CON2 (1<<30)L 
#define BMP_CON3 (1<<48)LL 
#define BMP_CON4 (1<<14)L
#define BMP_CON5 (1<<20)L
#define BMP_CON6 (1<<29)L
#define BMP_CON7 (1<<32)L
#define BMP_CON8 (1<<37)LL
#define BMP_CON9 0.125f // 2^-3
#define BMP_CON10 (1<<6)c
#define BMP_CON11 (1<<8)c
#define BMP_CON12 (1<<15)i
#define BMP_CON13 (1<<64)LL
#define BMP_CAL_TIME 1000
#define BMP_C1 0.95f // height measurement IIR coefficients
#define BMP_C2 0.05f
#define TB 288.15f // Standard temperature at sea level (K)
#define LB -0.0065f // Standard temperature lapse rate (K/m)
#define PB 101325f // Standard static pressure at sea level (Pa)
#define R 8.31432f // Universal gas constant (N-m)/(mol-K)
#define G 9.80665f // Acceleration due to gravity (m/s^2)
#define M 0.0289644f // Molar mass of air (kg/mol)

volatile unsigned char g_start_rst_flag = 0;

struct BMP {
	float PAR_T[3];
	float PAR_P[11];
    float height;
};
void SPI_Init(){
	DDRB |= (1<<PINB5) | (1<<PINB3) | (1<<PINB2); // Set SCK, MOSI, and SS as outputs
	PORTB |= (1<<PINB2); // Set SS high
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0); // Enables SPI, sets AVR as master, sets clock to f_osc/16
}
void Detain(unsigned long duration){
	for (volatile unsigned long i = 0;i<duration;i++){};
}
void SPI_Transmit_Char_Data_Visualizer(unsigned char* outbound, unsigned char length){
	PORTB &= ~(1<<PINB2); // Set B2 LOW
	unsigned char i = 0;
	while (i++<length){
		SPDR = *outbound++; // Put data into data register
		while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	}
	SPDR = 10; // Line feed
	while (!(SPSR & (1<<SPIF))){asm("");}
	PORTB |= (1<<PINB2); // Set B2 HIGH
}
void SPI_Transmit_Long_Data_Visualizer(long outbound, unsigned char length){
	char buffer[length];
	unsigned char n = snprintf(buffer,length,"%ld",outbound);
	PORTB &= ~(1<<PINB2); // Set B2 LOW
	for (unsigned char i=0;i<n;i++){
		SPDR = buffer[i]; // Put data into data register
		while (!(SPSR & (1<<SPIF))); // Wait for flag to be set in status register to continue
	}
	SPDR = 10; // Line feed
	while (!(SPSR & (1<<SPIF)));
	PORTB |= (1<<PINB2); // Set B2 HIGH
}
void SPI_Transmit_Float_Data_Visualizer(float outbound, unsigned char length){
	char buffer[length];
	unsigned char n = snprintf(buffer,length,"%f",outbound);
	PORTB &= ~(1<<PINB2); // Set B2 LOW
	for (unsigned char i=0;i<n;i++){
		SPDR = buffer[i]; // Put data into data register
		while (!(SPSR & (1<<SPIF))); // Wait for flag to be set in status register to continue
	}
	SPDR = 10; // Line feed
	while (!(SPSR & (1<<SPIF)));
	PORTB |= (1<<PINB2); // Set B2 HIGH
}
unsigned char SPI_Write_Register(char Port, char Pin, char Register, char Data){
	if (Port==1){PORTB &= ~(1<<Pin);}
	else if (Port==2){PORTC &= ~(1<<Pin);}
	else if (Port==3){PORTD &= ~(1<<Pin);}

	SPDR = Register; // Send register to write to, 7th bit must be 1 for write
	while (!(SPSR & (1<<SPIF))); // Wait for flag to be set in status register to continue
	SPDR = Data; // Send new value
	while (!(SPSR & (1<<SPIF)));
	char inbound = SPDR;
	
	if (Port==1){PORTB |= (1<<Pin);}
	else if (Port==2){PORTC |= (1<<Pin);}
	else if (Port==3){PORTD |= (1<<Pin);}
	
	return inbound;
}
void SPI_Read_Register(char Port, char Pin, char Register, unsigned char* buffer, unsigned char length){
	if (Port==1){PORTB &= ~(1<<Pin);}
	else if (Port==3){PORTD &= ~(1<<Pin);}
	else if (Port==2){PORTC &= ~(1<<Pin);}

	SPDR = Register; // Send register to read from, 0 in 7th bit signifies read command
	while (!(SPSR & (1<<SPIF))); // Wait for flag to be set in status register to continue
	unsigned char i = 0;
	
	while(i++<length){
		SPDR = 1;
		while (!(SPSR & (1<<SPIF))); // Wait for flag to be set in status register to continue
		*buffer = SPDR;
		++buffer;
	}
	
	if (Port==1){PORTB |= (1<<Pin);}
	else if (Port==3){PORTD |= (1<<Pin);}
	else if (Port==2){PORTC |= (1<<Pin);}
}
void ATMega328P_Init(){
	DDRD |= (1<<DDD4);
	PORTD |= (1<<PORTD7);
	PORTD &= ~(1<<PORTD4);
	Detain(10000UL);
}
void LoRa_Init(){
// Control bit in SPI must be 1 for write, 0 for read
	DDRD |= (1<<PIND6); // Set LoRa SS as output
	(void)SPI_Write_Register(3,SS_LORA_PIN,(LORA_REG_OP_MODE|0x80),0b10000000); // Set LoRa mode, still in standby
	(void)SPI_Write_Register(3,SS_LORA_PIN,(LORA_REG_F_MSB|0x80),0b11100100); // Set frequency to 915 MHz
	(void)SPI_Write_Register(3,SS_LORA_PIN,(LORA_REG_F_MIDB|0x80),0b11000000);
	(void)SPI_Write_Register(3,SS_LORA_PIN,(LORA_REG_F_LSB|0x80),0b00000000);
	(void)SPI_Write_Register(3,SS_LORA_PIN,(LORA_REG_OP_MODE|0x80),0b10000101); // Set LoRa mode into RXCONTINUOUS
	(void)SPI_Write_Register(3,SS_LORA_PIN,(LORA_REG_PA_CONFIG|0x80),0b01111111); // more..MORE...MORE!!!
}
void BMP_Init(struct BMP* sensor){
// Control bit in SPI must be 0 for write, 1 for read
	DDRB |= (1<<DDB0);
	PORTB |= (1<<PORTB0); // Dropping CS enables SPI mode
	PORTB &= ~(1<<PORTB0);
	Detain(1000UL);
	PORTB |= (1<<PORTB0);
	unsigned char data[21];
	(void)SPI_Write_Register(1,SS_BMP_PIN,BMP_PWR,0b00110011);
	(void)SPI_Write_Register(1,SS_BMP_PIN,BMP_OSR,0b00010101);
	(void)SPI_Write_Register(1,SS_BMP_PIN,BMP_ODR,0x04);
	(void)SPI_Write_Register(1,SS_BMP_PIN,BMP_INT,0b01000000);
	(void)SPI_Write_Register(1,SS_BMP_PIN,BMP_CONFIG,0b0001000);
	SPI_Read_Register(1,SS_BMP_PIN,(BMP_CAL_DATA|0x80),data,21);
	for (int i=0;i<21;i++){
		SPI_Transmit_Long_Data_Visualizer((long)data[i],10);
	}
	unsigned int NVM_PAR_T1 = data[1]*256+data[0];
  	unsigned int NVM_PAR_T2 = data[3]*256+data[2];
  	char NVM_PAR_T3 = data[4];
  	int NVM_PAR_P1 = (char)data[6]*256+(char)data[5];
  	int NVM_PAR_P2 = (char)data[8]*256+(char)data[7];
  	char NVM_PAR_P3 = data[9];
  	char NVM_PAR_P4 = data[10];
  	unsigned int NVM_PAR_P5 = data[12]*256+data[11];
  	unsigned int NVM_PAR_P6 = data[14]*256+data[13];
  	char NVM_PAR_P7 = data[15];
  	char NVM_PAR_P8 = data[16];
 	int NVM_PAR_P9 = (char)data[18]*256+(char)data[17];
  	char NVM_PAR_P10 = data[19];
  	char NVM_PAR_P11 = data[20];
  	// Convert to floats, equations located on page 55 of BST-BMP390-DS002.pdf
  	sensor->PAR_T[0] = NVM_PAR_T1/BMP_CON1;
  	sensor->PAR_T[1] = NVM_PAR_T2/BMP_CON2;
  	sensor->PAR_T[2] = NVM_PAR_T3/BMP_CON3;
  	sensor->PAR_P[0] = (NVM_PAR_P1-BMP_CON4)/BMP_CON5;
  	sensor->PAR_P[1] = (NVM_PAR_P2-BMP_CON4)/BMP_CON6;
  	sensor->PAR_P[2] = NVM_PAR_P3/BMP_CON7;
  	sensor->PAR_P[3] = NVM_PAR_P4/BMP_CON8;
  	sensor->PAR_P[4] = NVM_PAR_P5/BMP_CON9;
  	sensor->PAR_P[5] = NVM_PAR_P6/BMP_CON10;
  	sensor->PAR_P[6] = NVM_PAR_P7/BMP_CON11;
  	sensor->PAR_P[7] = NVM_PAR_P8/BMP_CON12;
  	sensor->PAR_P[8] = NVM_PAR_P9/BMP_CON3;
  	sensor->PAR_P[9] = NVM_PAR_P10/BMP_CON3;
 	sensor->PAR_P[10] = NVM_PAR_P11/BMP_CON13;
}
void LoRa_Read(int* value){
	unsigned char data_available;
	SPI_Read_Register(3,SS_LORA_PIN,LORA_REG_RX_N_BYTES,&data_available,1);
	if (data_available>13){
		unsigned char buffer[14];
		unsigned char RX_Adrs;
		SPI_Read_Register(3,SS_LORA_PIN,LORA_REG_RX_ADR,&RX_Adrs,1);
		(void)SPI_Write_Register(3,SS_LORA_PIN,(LORA_REG_FIFO_ADR_PTR|0x80),RX_Adrs); // Set FIFO ptr to current FIFO RX address
		SPI_Read_Register(3,SS_LORA_PIN,LORA_REG_FIFO,buffer,14);
		int check_sum = 0;
		int sum = 0;
		for (unsigned char i=0;i<12;i++){
			check_sum+=buffer[i];
		}
		if (check_sum==1132){sum = buffer[13]*256+buffer[12];}
		if (sum==5000){
			if (g_start_rst_flag==0){
				PORTD |= (1<<PORTD4);
				g_start_rst_flag = 1;
			}
			else{
				DDRD |= (1<<DDD7);
				PORTD &= ~((1<<PORTD7) | (1<<PORTD4));
				Detain(1000UL);
				DDRD &= ~(1<<DDD7);
				PORTD |= (1<<PORTD7);
				g_start_rst_flag = 0;
			}
			Detain(200000UL);
		}
		else{
			*value = sum;
		}
	}
}
void BMP_Read(struct BMP* sensor){
    static int cal_wait;
    static float height_bias;
    static float pressure_comp;

    unsigned char data[6];
    SPI_Read_Register(1,SS_BMP_PIN,(BMP_DATA|0x80),data,6);
    unsigned long pressure = data[2]*65536+data[1]*256+data[0];
    unsigned long temperature = data[5]*65536+data[4]*256+data[3];

    // Compensation equations located on pages 56-57 of BST-BMP390-DS002.pdf
    float tpartial_data[2];
    float ppartial_data[4];
    float ppartial_out[2];
    tpartial_data[0] = (float)(temperature-sensor->PAR_T[0]);
    tpartial_data[1] = (float)(tpartial_data[0]*sensor->PAR_T[1]);
    float Temperature_Comp = tpartial_data[1]+pow(tpartial_data[0],2)*sensor->PAR_T[2];
    float temp1 = pow(Temperature_Comp,2);
    float temp2 = pow(Temperature_Comp,3);
    ppartial_data[0] = sensor->PAR_P[5]*Temperature_Comp;
    ppartial_data[1] = sensor->PAR_P[6]*temp1;
    ppartial_data[2] = sensor->PAR_P[7]*temp2;
    ppartial_out[0] = sensor->PAR_P[4]+ppartial_data[0]+ppartial_data[1]+ppartial_data[2];
    ppartial_data[0] = sensor->PAR_P[1]*Temperature_Comp;
    ppartial_data[1] = sensor->PAR_P[2]*temp1;
    ppartial_data[2] = sensor->PAR_P[3]*temp2;
    ppartial_out[1] = (float)pressure*(sensor->PAR_P[0]+ppartial_data[0]+ppartial_data[1]+ppartial_data[2]);
    ppartial_data[0] = pow((float)pressure,2);
    ppartial_data[1] = sensor->PAR_P[8] + sensor->PAR_P[9]*Temperature_Comp;
    ppartial_data[2] = ppartial_data[0]*ppartial_data[1];
    ppartial_data[3] = ppartial_data[2] + pow((float)pressure,3)*sensor->PAR_P[10];
    // IIR implemented on compensated pressure, this way height does not need to be passed back into the function (meh)
    pressure_comp = pressure_comp*BMP_C1 + (ppartial_out[0]+ppartial_out[1]+ppartial_data[3])*BMP_C2;
    // The constants used below are for sea level, to get relative height we will remove a bias from the height calculation
    // The equation used is valid for any altitude under 11,000 meters MSL

    if (++cal_wait<BMP_CAL_TIME){height_bias = (TB/LB)*(pow(pressure_comp/PB,(-R*LB)/(G*M))-1);}
    else {sensor->height = ((TB/LB)*(pow(pressure_comp/PB,(-R*LB)/(G*M))-1))-height_bias;}

}
void I2C_Init(){
	TWAR = (0x10<<1); // Slave address
	TWCR |= (1<<TWEA) | (1<<TWEN);
}

unsigned char I2C_Slave_Transmit(unsigned char* data, unsigned char length){
	// returns 3 if successful, lower numbers indicate stage of error
	if ((TWSR & 0xF8) != 0xA8){return 0;} // SLA+R received
	unsigned char j = 0;
	while (++j<length){
		TWDR = *data++;
		TWCR |= (1<<TWINT) | (1<<TWEA);
		while(!(TWCR & (1<<TWINT))){asm("");}
		if ((TWSR & 0xF8) != 0xB8){return 1;}
	}
	TWDR = *data;
	data++;
	TWCR |= (1<<TWINT);
	TWCR ^= (1<<TWEA);
	while(!(TWCR & (1<<TWINT))){asm("");}
	if ((TWSR & 0xF8) != 0xC8){return 2;}
	TWCR |= (1<<TWINT) | (1<<TWEA);

	return 3;
}

#endif
