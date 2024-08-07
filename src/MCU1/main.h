#ifndef MAIN_H
#define MAIN_H

#ifndef F_CPU
#define F_CPU 20000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#define ACC_ADRS 0x18 // or 0x19 if SDO1 pulled VDIO
#define ACC_DATA 0x12 // Start address of acc data, 16-bit x-y-z, LSB first
#define ACC_CONF 0x40 
#define ACC_RANGE 0x41 
#define INT2_IO_CONF 0x54 
#define INT1_INT2_MAP_DATA 0x58 
#define ACC_PWR_CTRL 0x7D
#define ACC_IIR1 0.95
#define ACC_IIR2 0.05

#define GYR_ADRS 0x68 // or 0x69 if SDO2 pulled VDIO
#define GYR_DATA 0x02 // Start address of gyro data, 16-bit x-y-z, LSB first
#define GYR_RANGE 0x0F 
#define GYR_BANDWIDTH 0x10
#define GYR_INT_CTRL 0x15
#define INT3_INT4_IO_MAP 0x18
#define GYR_IIR1 0.9
#define GYR_IIR2 0.1

volatile char g_Acc_Flag = 0;
volatile char g_Gyr_Flag = 0;

struct BMI088 {
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float W_X;
    float W_Y;
    float W_Z;
    float W_X_bias;
    float W_Y_bias;
    float W_Z_bias;
};

void Detain(unsigned long length){
	// A reproduction of the delay() function, prevents overwhelming the serial monitor
	for (unsigned long i=0;i<length;i++){asm("");}
}
unsigned char I2C_Transmit(unsigned char Slave_Address, unsigned char Register_Address, unsigned char Register_Data){
// Transmit: Writes one byte of data into a slave device register
// returns 4 if successful, lower numbers indicate stage of error 
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // TWINT starts operation of TWI, TWSTA sends START condition, TWEN enables TWI
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set indicating START condition set
  if (((TWSR & 0xF8) != TW_REP_START)&&((TWSR & 0xF8) != TW_START)){return 0;} // Mask prescaler bits in TWSR, check if not equal to start condition to trigger error
  TWDR = Slave_Address<<1; // Write slave address to data register as write
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MT_SLA_ACK){return 1;} // Mask prescaler bits in TWSR, check if slave acknowledge
  TWDR = Register_Address; // Send data (address to write to)
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MT_DATA_ACK){return 2;} // Mask prescaler bits in TWSR, check if data sent
  TWDR = Register_Data; // Send data (new value of address)
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MT_DATA_ACK){return 3;} // Mask prescaler bits in TWSR, check if data sent
  TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); // Send Stop
  return 4; // Success
}
unsigned char I2C_Read(unsigned char Slave_Address, unsigned char Register_Address, unsigned char *data, unsigned char length){
// Read: Reads up to 256 bytes of data from the specified slave address
// IMPORTANT, is looking for an array of data, pass address if only reading a single value
// Returns 8 if successful, lower numbers indicate stage of error
  // start write write start write/read read ack read ack... stop
  unsigned char j = 0;
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // TWINT starts operation of TWI, TWSTA sends START condition, TWEN enables TWI
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set indicating START condition set
  if (((TWSR & 0xF8) != TW_REP_START)&&((TWSR & 0xF8) != TW_START)){return 0;} // Mask prescaler bits in TWSR, check if not equal to start condition to trigger error
  TWDR = Slave_Address<<1; // Write slave address to data register as write
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MT_SLA_ACK){return 1;} // Mask prescaler bits in TWSR, check if slave acknowledge
  TWDR = Register_Address; // Send data (address device looks at)
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MT_DATA_ACK){return 2;} // Mask prescaler bits in TWSR, check if data sent
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // TWINT starts operation of TWI, TWSTA sends START condition, TWEN enables TWI
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set indicating START condition set
  if ((TWSR & 0xF8) != TW_REP_START){return 3;} // Mask prescaler bits in TWSR, check if not equal to start condition to trigger error

  TWDR = (Slave_Address<<1) | (1<<0); // Write slave address to data register as read
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MR_SLA_ACK){return 4;} // Mask prescaler bits in TWSR, check if slave acknowledge

  unsigned char temp = TWDR; // Throw away first value
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // Data byte received and ACK transmitted back to slave
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MR_DATA_ACK){return 5;} // Mask prescaler bits in TWSR, send acknowledge to slave  

  while(j<length){
    *data = TWDR; // Read data sent from slave
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // Data byte received and ACK transmitted back to slave
    while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
    if ((TWSR & 0xF8) != TW_MR_DATA_ACK){return 6;} // Mask prescaler bits in TWSR, send acknowledge to slave   
    ++data; // Increment pointer
    ++j;
  }
 
  *data = TWDR; // Read last byte of data sent from slave
  TWCR = (1<<TWINT) | (1<<TWEN) | (0<<TWEA); // Data byte received and NACK transmitted back to slave
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MR_DATA_NACK){return 7;} // Mask prescaler bits in TWSR, send not acknowledge to slave 
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // Send STOP

  return 8; // Success
}
void Read_Acc(struct BMI088* Sensor){
	unsigned char data[6];
	unsigned char length = sizeof(data);
	unsigned char result = I2C_Read(ACC_ADRS,ACC_DATA,data,length);
	if (result==8){
		int Accel_X_B = (data[1]*256) + data[0];
		int Accel_Y_B = (data[3]*256) + data[2];
		int Accel_Z_B = (data[5]*256) + data[4];
		Sensor->Accel_X = (Sensor->Accel_X*ACC_IIR1) + (((float)Accel_X_B/32678)*6*ACC_IIR2);
		Sensor->Accel_Y = (Sensor->Accel_Y*ACC_IIR1) + (((float)Accel_Y_B/32678)*6*ACC_IIR2);
		Sensor->Accel_Z = (Sensor->Accel_Z*ACC_IIR1) + (((float)Accel_Z_B/32678)*6*ACC_IIR2);
	}
}
void Read_Gyr(struct BMI088* Sensor){
	unsigned char data[6];
	unsigned char length = sizeof(data);
	unsigned char result = I2C_Read(GYR_ADRS,GYR_DATA,data,length);
	if (result==8){
		int Gyr_X_B = (data[1]*256) + data[0];
		int Gyr_Y_B = (data[3]*256) + data[2];
		int Gyr_Z_B = (data[5]*256) + data[4];
		Sensor->W_X = (Sensor->W_X*GYR_IIR1) + (((float)Gyr_X_B/32678)*500*GYR_IIR2);
		Sensor->W_Y = (Sensor->W_Y*GYR_IIR1) + (((float)Gyr_Y_B/32678)*500*GYR_IIR2);
		Sensor->W_Z = (Sensor->W_Z*GYR_IIR1) + (((float)Gyr_Z_B/32678)*500*GYR_IIR2);
	}
}
char Initialize_BMI088(struct BMI088* Sensor){
// returns 9 if successful, lower numbers indicate stage of error     
    // Accelerometer initialization
    unsigned char status = I2C_Transmit(ACC_ADRS,ACC_PWR_CTRL,0x04); // Turn on accelerometer
    if (status!=4){return 0;}
    Detain(5000); // Give time for accelerometer to power on
    unsigned char outbound = ((0x09)<<4) | (0x08);
    status = I2C_Transmit(ACC_ADRS,ACC_CONF,outbound); // Set OSR2 and 100Hz ODR
    if (status!=4){return 1;}
    status = I2C_Transmit(ACC_ADRS,ACC_RANGE,0x01); // Set range to +-6g
    if (status!=4){return 2;}
    status = I2C_Transmit(ACC_ADRS,INT2_IO_CONF,0x08); // Enable INT2 as output pin
    if (status!=4){return 3;}
    status = I2C_Transmit(ACC_ADRS,INT1_INT2_MAP_DATA,0x40); // Map data ready interrupt to INT2 pin
    if (status!=4){return 4;}
    // Gyroscope initialization
    status = I2C_Transmit(GYR_ADRS,GYR_RANGE,0x02); // Set range to +-500deg/s
    if (status!=4){return 5;}
    status = I2C_Transmit(GYR_ADRS,GYR_BANDWIDTH,0x07); // Set 100Hz ODR, filter bandwidth of 32Hz
    if (status!=4){return 6;}
    status = I2C_Transmit(GYR_ADRS,GYR_INT_CTRL,0x80); // Enable new data interrupt
    if (status!=4){return 7;}
    status = I2C_Transmit(GYR_ADRS,INT3_INT4_IO_MAP,0x01); // Map data ready interrupt to INT3 pin
    if (status!=4){return 8;}
    // Calibrate gyroscope
	struct BMI088 CalSensor = {0};
    volatile unsigned long i = 0;
    while(++i<10000000){
        if (g_Gyr_Flag==1){
			g_Gyr_Flag = 0;
			Read_Gyr(&CalSensor);
		}
	}
    Sensor->W_X_bias = CalSensor.W_X;
    Sensor->W_Y_bias = CalSensor.W_Y;
    Sensor->W_Z_bias = CalSensor.W_Z;

    return 9;
}
void Initialize_ATMEGA328P(){
// Use to group MCU one time register setups prior to running while loop
  sei(); 
  // f_SCL = 20000000/(16+2*(TWBR*Prescaler))
  TWBR = 17;
  // f_SCL = 400kHz with above settings, limit of ATMega328P with internal pullup resistors
  PCICR |= (1<<2); // Enable PC interrupts on port D
  PCMSK2 |= (1<<3); // Enable D3 for PC interrupts
}
/*
ISR(){
    g_Acc_Flag == 1;
}

ISR(){
    g_Gyr_Flag == 1;
}
*/
#endif
