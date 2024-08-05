#ifndef MAIN_H
#define MAIN_H

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <math.h>

// Transmit: Writes one byte of data into a slave device register
// returns 4 if successful, lower numbers indicate stage of error 
unsigned char Transmit(unsigned char Slave_Address, unsigned char Register_Address, unsigned char Register_Data){
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
}
// A reproduction of the delay() function, prevents overwhelming the serial monitor
void Detain(unsigned int length){
  for (unsigned int i=0;i<length;i++){asm("");}
}
// Use to group Transmit calls when sensor is being set up 
// Returns 2 if successful, lower numbers indicate stage of error
unsigned char Initialize_BMP390(float *PAR_T, float *PAR_P){
  // Throw initial register setting on sensor in this function, results will ==4 if all checks passed
  unsigned char Register_Data0 = (1<<5) | (1<<4) | (1<<1) | (1<<0);
  unsigned char result0 = Transmit(BMTR_ADRS,BMTR_PWR_CTRL,Register_Data0);
  unsigned char Register_Data1 = (1<<4) | (1<<2) | (1<<0);
  unsigned char result1 = Transmit(BMTR_ADRS,BMTR_OSR,Register_Data1);
  unsigned char Register_Data2 = (1<<2);
  unsigned char result2 = Transmit(BMTR_ADRS,BMTR_ODR,Register_Data2);
  unsigned char Register_Data3 = (1<<3);
  unsigned char result3 = Transmit(BMTR_ADRS,BMTR_IIR,Register_Data3);
  unsigned char Register_Data4 = (1<<6);
  unsigned char result4 = Transmit(BMTR_ADRS,BMTR_INT,Register_Data4);
  if ((result0!=4)||(result1!=4)||(result2!=4)||(result3!=4)||(result4!=4)){return 0;}
  // Establish factory constants
  unsigned char data[21];
  unsigned char length = sizeof(data);
  // Read factory calibration data
  // Addresses, sizes, and sign data located on page 28 of BST-BMP390-DS001.pdf
  unsigned char result = Read(BMTR_ADRS,BMTR_CON_START,data,length);
  if (result!=8){return 1;}
  unsigned int NVM_PAR_T1 = data[1]<<8;
  NVM_PAR_T1 += data[0];
  unsigned int NVM_PAR_T2 = data[3]<<8;
  NVM_PAR_T2 += data[2];
  char NVM_PAR_T3 = data[4];
  int NVM_PAR_P1 = data[6]<<8;
  NVM_PAR_P1 += data[5];
  int NVM_PAR_P2 = data[8]<<8;
  NVM_PAR_P2 += data[7];
  char NVM_PAR_P3 = data[9];
  char NVM_PAR_P4 = data[10];
  unsigned int NVM_PAR_P5 = data[12]<<8;
  NVM_PAR_P5 += data[11];
  unsigned int NVM_PAR_P6 = data[14]<<8;
  NVM_PAR_P6 += data[13];
  char NVM_PAR_P7 = data[15];
  char NVM_PAR_P8 = data[16];
  int NVM_PAR_P9 = data[18]<<8;
  NVM_PAR_P9 += data[17];
  char NVM_PAR_P10 = data[19];
  char NVM_PAR_P11 = data[20];

  *PAR_T = NVM_PAR_T1/BMTR_cT1; // PAR_T1
  ++PAR_T;
  *PAR_T = NVM_PAR_T2/BMTR_cT2; // PAR_T2
  ++PAR_T;
  *PAR_T = NVM_PAR_T3/BMTR_cT3; // PAR_T3

  *PAR_P = (NVM_PAR_P1-BMTR_cP0);
  *PAR_P /= BMTR_cP1; // PAR_P1
  ++PAR_P;
  *PAR_P = (NVM_PAR_P2-BMTR_cP0);
  *PAR_P /= BMTR_cP2; // PAR_P2
  ++PAR_P;
  *PAR_P = NVM_PAR_P3/BMTR_cP3; // PAR_P3
  ++PAR_P;
  *PAR_P = NVM_PAR_P4/BMTR_cP4; // PAR_P4
  ++PAR_P;
  *PAR_P = NVM_PAR_P5/BMTR_cP5; // PAR_P5
  ++PAR_P;
  *PAR_P = NVM_PAR_P6/BMTR_cP6; // PAR_P6
  ++PAR_P;
  *PAR_P = NVM_PAR_P7/BMTR_cP7; // PAR_P7
  ++PAR_P;
  *PAR_P = NVM_PAR_P8/BMTR_cP8; // PAR_P8
  ++PAR_P;
  *PAR_P = NVM_PAR_P9/BMTR_cP9; // PAR_P9
  ++PAR_P;
  *PAR_P = NVM_PAR_P10/BMTR_cP10; // PAR_P10
  ++PAR_P;
  *PAR_P = NVM_PAR_P11/BMTR_cP11; // PAR_P11
  return 2;
}
// Use this to read the Temperature data off the sensor
float Read_BMP390(float *PAR_T, float *PAR_P){
  static volatile float Height_Bias;
  static volatile unsigned char calWait;
  const float c1 = BMTR_Tb/BMTR_Lb;
  const float c2 = (-BMTR_R*BMTR_Lb)/(BMTR_g*BMTR_M);
  unsigned char data[6];
  unsigned char length = sizeof(data);
  unsigned char result = Read(BMTR_ADRS,BMTR_PRES_DATA,data,length);
  if (result==8){
    // Read Data
    unsigned long Pressure = data[2];
    Pressure = Pressure<<16;
    unsigned int MRFudge = data[1];
    MRFudge = MRFudge<<8;
    MRFudge += data[0];
    Pressure += MRFudge;
    unsigned long Temperature = data[5];
    Temperature = Temperature<<16;
    unsigned int MRSFudge = data[4];
    MRSFudge = MRSFudge<<8;
    MRSFudge += data[3];
    Temperature += MRSFudge;

    // Calibrate Data
    float tpartial_data[2];
    float ppartial_data[4];
    float ppartial_out[2];

    tpartial_data[0] = (float)(Temperature-*PAR_T);
    ++PAR_T; // Move to PAR_T2
    tpartial_data[1] = (float)(tpartial_data[0]**PAR_T);
    ++PAR_T; // Move to PAR_T3
    float Temperature_Comp = tpartial_data[1]+(tpartial_data[0]*tpartial_data[0]**PAR_T);
    float temp1 = Temperature_Comp*Temperature_Comp;
    float temp2 = temp1*Temperature_Comp;
    PAR_P += 5; // Move to PAR_P6
    ppartial_data[0] = *PAR_P*Temperature_Comp;
    ++PAR_P; // Move to PAR_P7
    ppartial_data[1] = *PAR_P*temp1;
    ++PAR_P; // Move to PAR_P8
    ppartial_data[2] = *PAR_P*temp2;
    PAR_P -= 3; // Move to PAR_P5
    ppartial_out[0] = *PAR_P+ppartial_data[0]+ppartial_data[1]+ppartial_data[2];
    PAR_P -= 3; // Move to PAR_P2
    ppartial_data[0] = *PAR_P*Temperature_Comp;
    ++PAR_P; // Move to PAR_P3
    ppartial_data[1] = *PAR_P*temp1;
    ++PAR_P; // Move to PAR_P4
    ppartial_data[2] = *PAR_P*temp2;
    PAR_P -= 3; // Move to PAR_P1
    ppartial_out[1] = (float)Pressure*(*PAR_P+ppartial_data[0]+ppartial_data[1]+ppartial_data[2]);
    ppartial_data[0] = Pressure*Pressure;
    PAR_P += 8; // Move to PAR_P9
    ppartial_data[1] = *PAR_P;
    ++PAR_P; // Move to PAR_P10
    ppartial_data[1] += (*PAR_P*Temperature_Comp);
    ppartial_data[2] = ppartial_data[0]*ppartial_data[1];
    ++PAR_P; // Move to PAR_P11
    ppartial_data[3] = ppartial_data[2] + (ppartial_data[0]*Pressure**PAR_P);
    float Pressure_Comp = ppartial_out[0]+ppartial_out[1]+ppartial_data[3];
    float Height_Compensated = c1*(pow((Pressure_Comp/BMTR_Pb),c2)-1); // Height in m
    if (calWait<50){
      ++calWait;
      Height_Bias = Height_Compensated;
    }
    else if (calWait==50){
      float Height_Out = Height_Compensated-Height_Bias;
      return Height_Out;
    }
    
  }
  
  return 0;
}
// Use to group MCU one time register setups prior to running while loop
void Initialize_ATMEGA328P(){
  sei(); 
  // f_SCL = 16000000/(16+2*(TWBR*Prescaler))
  TWBR = 1;
  TWSR = (1<<TWPS1); // Sets prescaler to 16
  // f_SCL = 400kHz with above settings, limit of ATMega328P with internal pullup resistors
  PCICR |= (1<<2); // Enable PC interrupts on port D
  PCMSK2 |= (1<<3); // Enable D3 for PC interrupts
}

#define BMTR_ADRS 0x77 // BMP 390 Address
#define BMTR_PWR_CTRL 0x1B // Power control register
#define BMTR_TEMP_DATA 0x07 // First bit of temperature data
#define BMTR_PRES_DATA 0x04 // First bit of pressure data
#define BMTR_OSR 0x1C // Output sample rate register
#define BMTR_ODR 0x1D // Output data rate register
#define BMTR_IIR 0x1F // IIR filter register
#define BMTR_INT 0x19 // Interrupt Control register
#define BMTR_CON_START 0x31 // Start register of factory calibration data
// Below constants are scaled to use integers instead of floats 
// Temperature calibration constants
#define BMTR_cT1 pow(2,-8) // 2^-8 
#define BMTR_cT2 pow(2,30) // 2^30 
#define BMTR_cT3 pow(2,48) // 2^48 
// Pressure calibration constants
#define BMTR_cP0 pow(2,14) // 2^14
#define BMTR_cP1 pow(2,20) // 2^20 
#define BMTR_cP2 pow(2,29) // 2^29 
#define BMTR_cP3 pow(2,32) // 2^32 
#define BMTR_cP4 pow(2,37)// 2^37 
#define BMTR_cP5 pow(2,-3) // 2^-3 
#define BMTR_cP6 pow(2,6) // 2^6
#define BMTR_cP7 pow(2,8) // 2^8
#define BMTR_cP8 pow(2,15) // 2^15
#define BMTR_cP9 pow(2,48) // 2^48 
#define BMTR_cP10 pow(2,48) //281474976710656 // 2^48 
#define BMTR_cP11 pow(2,65)//3689348814741910323 // 2^65 scaled by 1e-1
// Height calculation constants
#define BMTR_Tb 288.15 
#define BMTR_Lb -0.0065
#define BMTR_Pb 101325
#define BMTR_R 8.31432
#define BMTR_g 9.80665
#define BMTR_M 0.0289644

volatile unsigned char bmtr_flag;


ISR (PCINT2_vect){
  bmtr_flag ^= 1;
}

#endif
