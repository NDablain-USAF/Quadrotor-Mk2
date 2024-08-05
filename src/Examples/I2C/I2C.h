#ifndef I2C_H
#define I2C_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#ifndef F_CPU
#define F_CPU 20000000UL
#endif

#define BMTR_ADRS 0x77 // BMP 390 Address
#define BMTR_PWR_CTRL 0x1B // Power control register
#define BMTR_OSR 0x1C // Output sample rate register
#define BMTR_ODR 0x1D // Output data rate register
#define BMTR_CON_START 0x31 // Start register of factory calibration data

// Transmit: Writes one byte of data into a slave device register
// Returns 4 if successful, lower numbers indicate stage of error 
unsigned char Transmit (unsigned char Slave_Address, unsigned char Register_Address, unsigned char Register_Data){
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
};
// Read: Reads up to 256 bytes of data from the specified slave address
// IMPORTANT, is looking for an array of data, pass address if only reading a single value
// Returns 8 if successful, lower numbers indicate stage of error
unsigned char Read(unsigned char Slave_Address, unsigned char Register_Address, unsigned char *data, unsigned char length){
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
};
// Detain: Reproduction of the arduino.h delay() function, prevents overwhelming the serial monitor
void Detain (unsigned int length){
  for (unsigned int i=0;i<length;i++){asm("");}
};
// Initialize: Groups Transmit calls when sensor is being set up 
unsigned char Initialize_BMP(){
  // Throw initial register setting on sensor in this function, results will ==4 if all checks passed
  unsigned char Register_Data0 = (1<<5) | (1<<4) | (1<<1) | (1<<0);
  unsigned char result0 = Transmit(BMTR_ADRS,BMTR_PWR_CTRL,Register_Data0);
  unsigned char Register_Data1 = (1<<4) | (1<<2) | (1<<0);
  unsigned char result1 = Transmit(BMTR_ADRS,BMTR_OSR,Register_Data1);
  unsigned char Register_Data2 = (1<<2);
  unsigned char result2 = Transmit(BMTR_ADRS,BMTR_ODR,Register_Data2);
  if ((result0==4)&&(result1==4)&&(result2==4)){return 1;}
  else {return 0;}
};
void Initialize(){
  sei(); 
  // f_SCL = 16000000/(16+2*(TWBR*Prescaler))
  TWBR = 1;
  TWSR = (1<<TWPS1); // Sets prescaler to 16
  // f_SCL = 400kHz with above settings, limit of ATMega328P with internal pullup resistors
}

#endif
