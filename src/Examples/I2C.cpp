// Example to show how to transmit and receive data as Master using ATMega328P Two Wire Interface (TWI)
// Test sensor is BMP 390 Barometer, internal pullup resistors are utilized on SDA/SCL pins of MCU
// Can be changed to use any sensor, just change the addresses defined at the start as well as the
// Register Data being written and length of the data array to be read
// Created by Nathan Dablain

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
uint8_t Transmit (uint8_t Slave_Address, uint8_t Register_Address, uint8_t Register_Data);
// Read: Reads up to 256 bytes of data from the specified slave address
// IMPORTANT, is looking for an array of data, pass address if only reading a single value
// Returns 8 if successful, lower numbers indicate stage of error
uint8_t Read(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *data, uint8_t length);
// Detain: Reproduction of the arduino.h delay() function, prevents overwhelming the serial monitor
void Detain (uint16_t length);
// Initialize: Groups Transmit calls when sensor is being set up 
uint8_t Initialize();

int main(){
  sei(); 
  // f_SCL = 16000000/(16+2*(TWBR*Prescaler))
  TWBR = 1;
  TWSR = (1<<TWPS1); // Sets prescaler to 16
  // f_SCL = 400kHz with above settings, limit of ATMega328P with internal pullup resistors
  Serial.begin(115200);
  uint8_t Status = Initialize(); // Group sensor setup in this function
  while(1){
    if (Status==1){ // If initialization was successful
      uint8_t data[21]; 
      uint8_t length = sizeof(data);
      // Again if data is a scaler, add an & before it in the function call below
      uint8_t result = Read(BMTR_ADRS,BMTR_CON_START,data,length); 
      Detain(65000);
      if (result==8){
        // Remove for loop if data is a scaler
        for (uint8_t i=0;i<sizeof(data);i++){
          Serial.print(data[i]);
          Serial.print(",");
        }
        Serial.println();
      }
      else {
        Serial.println("Bad Read");
      }
    }
    Detain(65000);
  }

  return 0;
}

uint8_t Initialize(){
  // Throw initial register setting on sensor in this function, results will ==4 if all checks passed
  uint8_t Register_Data0 = (1<<5) | (1<<4) | (1<<1) | (1<<0);
  uint8_t result0 = Transmit(BMTR_ADRS,BMTR_PWR_CTRL,Register_Data0);
  uint8_t Register_Data1 = (1<<4) | (1<<2) | (1<<0);
  uint8_t result1 = Transmit(BMTR_ADRS,BMTR_OSR,Register_Data1);
  uint8_t Register_Data2 = (1<<2);
  uint8_t result2 = Transmit(BMTR_ADRS,BMTR_ODR,Register_Data2);
  if ((result0==4)&&(result1==4)&&(result2==4)){return 1;}
  else {return 0;}
}

void Detain(uint16_t length){
      for (uint16_t i=0;i<length;i++){asm("");}
}

uint8_t Transmit (uint8_t Slave_Address, uint8_t Register_Address, uint8_t Register_Data){
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

uint8_t Read(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *data, uint8_t length){
  // start write write start write/read read ack read ack... stop
  uint8_t j = 0;
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

  uint8_t temp = TWDR; // Throw away first value
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
