#ifndef MAIN_H
#define MAIN_H

#ifndef F_CPU
#define F_CPU 20000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <stdio.h>

#define MCU2_ADRS 0x10

#define ACC_ADRS 0x18 // or 0x19 if SDO1 pulled VDIO
#define ACC_DATA 0x12 // Start address of acc data, 16-bit x-y-z, LSB first
#define ACC_CONF 0x40 
#define ACC_RANGE 0x41 
#define INT2_IO_CONF 0x54 
#define INT1_INT2_MAP_DATA 0x58 
#define ACC_PWR_CTRL 0x7D
#define ACC_IIR1 0.95f
#define ACC_IIR2 0.05f

#define GYR_ADRS 0x68 // or 0x69 if SDO2 pulled VDIO
#define GYR_DATA 0x02 // Start address of gyro data, 16-bit x-y-z, LSB first
#define GYR_RANGE 0x0F 
#define GYR_BANDWIDTH 0x10
#define GYR_INT_CTRL 0x15
#define INT3_INT4_IO_MAP 0x18
#define GYR_IIR1 0.9f
#define GYR_IIR2 0.1f

#define MAG_ADRS 0x14 // or 0x15 if ADSEL pulled VDDIO
#define MAG_DATA 0x31 // Start address of mag data, 24-bit x-y-z, LSB first
#define MAG_ODR 0x04
#define MAG_MODE 0x06 
#define MAG_INT_CTRL 0x2E
#define MAG_OTP_CMD 0x50
#define MAG_XY_GAIN 0.007069978f
#define MAG_Z_GAIN 0.007174964f
#define MAG_IIR1 0.95f
#define MAG_IIR2 0.05f

#define MOTOR_CONTROL_TIME 19
#define E_INT_LIMIT 10000
#define MOTOR_C1 1009 // Motor back-EMF IIR filter coefficients
#define MOTOR_C2 15
#define MOTOR_K_P 10UL // Motor proportional control gain
#define MOTOR_K_I 4UL // Motor integral control gain
#define MOTOR_K_D 128UL // Motor derivative control gain
#define ref 150
#define MOTOR1_MUX (1<<REFS0)
#define MOTOR2_MUX ((1<<REFS0) | (1<<MUX0))
#define MOTOR3_MUX ((1<<REFS0) | (1<<MUX1))
#define MOTOR4_MUX ((1<<REFS0) | (1<<MUX0) | (1<<MUX1))

volatile char g_Start_Flag = 0;
volatile char g_Acc_Flag = 0;
volatile char g_Gyr_Flag = 0;
volatile char g_Mag_Flag = 0;
volatile char g_Motor_Flag = 0;
volatile char g_ADC_Flag = 0;

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

struct BMM350 {
	float Mag_X;
	float Mag_Y;
	float Mag_Z;
};

struct Motor {
	int e_int;
	unsigned long x;
	int e_last;
	unsigned int r;
	unsigned char mux;
};

void Detain(unsigned long length){
	// A reproduction of the delay() function, prevents overwhelming the serial monitor
	for (unsigned long i=0;i<length;i++){asm("");}
}

void SPI_Int(){
	DDRB |= (1<<PINB5) | (1<<PINB3) | (1<<PINB2); // Set SCK, MOSI, and SS as outputs
	PORTB |= (1<<PINB2); // Set SS high
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0); // Enables SPI, sets AVR as master, sets clock to f_osc/16
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

void SPI_Transmit_Long_Data_Visualizer(long outbound, unsigned char length, unsigned char option){
	char buffer[length];
	unsigned char n = snprintf(buffer,length,"%ld",outbound);
	PORTB &= ~(1<<PINB2); // Set B2 LOW
	for (unsigned char i=0;i<n;i++){
		SPDR = buffer[i]; // Put data into data register
		while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	}
	if (option==2){SPDR = 10;} // Line feed
	else {SPDR = 44;} // ,
	while (!(SPSR & (1<<SPIF))){asm("");}
	PORTB |= (1<<PINB2); // Set B2 HIGH
}

unsigned char SPI_Write_Register(char Port, char Pin, char Register, unsigned char Data){
	if (Port==1){PORTB &= ~(1<<Pin);}
	else if (Port==2){PORTC &= ~(1<<Pin);}
	else if (Port==3){PORTD &= ~(1<<Pin);}

	SPDR = (Register|(1<<7)); // Send register to write to, 7th bit must be 1 for write
	while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	SPDR = Data; // Send new value
	while (!(SPSR & (1<<SPIF))){asm("");}
	unsigned char inbound = SPDR;
	
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
	while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	unsigned char i = 0;
	while(i++<length){
		SPDR = 1; // Must perform a dummy write even during read
		while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
		*buffer = SPDR;
		++buffer;
	}
	
	if (Port==1){PORTB |= (1<<Pin);}
	else if (Port==3){PORTD |= (1<<Pin);}
	else if (Port==2){PORTC |= (1<<Pin);}
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
  if (Slave_Address!=MCU2_ADRS){
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
  }
  TWDR = (Slave_Address<<1) | (1<<0); // Write slave address to data register as read
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MR_SLA_ACK){return 4;} // Mask prescaler bits in TWSR, check if slave acknowledge
 
  while(j<length){
    *data++ = TWDR;
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // Data byte received and ACK transmitted back to slave
    while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
    if ((TWSR & 0xF8) != TW_MR_DATA_ACK){return 6;} // Mask prescaler bits in TWSR, send acknowledge to slave   
    ++j;
  }
 
  *data = TWDR; // Read last byte of data sent from slave
  TWCR = (1<<TWINT) | (1<<TWEN) | (0<<TWEA); // Data byte received and NACK transmitted back to slave
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MR_DATA_NACK){return 7;} // Mask prescaler bits in TWSR, send not acknowledge to slave 
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // Send STOP

  return 8; // Success
}
void Read_Acc(struct BMI088** Sensor){
	unsigned char data[6];
	unsigned char length = sizeof(data);
	unsigned char result = I2C_Read(ACC_ADRS,ACC_DATA,data,length);
	if (result==8){
		int Accel_X_B = ((int)data[1]<<8) + data[0];
		int Accel_Y_B = ((int)data[3]<<8) + data[2];
		int Accel_Z_B = ((int)data[5]<<8) + data[4];
		(*Sensor)->Accel_X = ((*Sensor)->Accel_X*ACC_IIR1) + (((float)Accel_X_B/32678)*6*ACC_IIR2);
		(*Sensor)->Accel_Y = ((*Sensor)->Accel_Y*ACC_IIR1) + (((float)Accel_Y_B/32678)*6*ACC_IIR2);
		(*Sensor)->Accel_Z = ((*Sensor)->Accel_Z*ACC_IIR1) + (((float)Accel_Z_B/32678)*6*ACC_IIR2);
	}
}
void Read_Gyr(struct BMI088** Sensor){
	unsigned char data[6];
	unsigned char length = sizeof(data);
	unsigned char result = I2C_Read(GYR_ADRS,GYR_DATA,data,length);
	if (result==8){
		int Gyr_X_B = ((int)data[1]<<8) + data[0];
		int Gyr_Y_B = ((int)data[3]<<8) + data[2];
		int Gyr_Z_B = ((int)data[5]<<8) + data[4];
		(*Sensor)->W_X = ((*Sensor)->W_X*GYR_IIR1) + (((float)Gyr_X_B/32678)*500*GYR_IIR2);
		(*Sensor)->W_Y = ((*Sensor)->W_Y*GYR_IIR1) + (((float)Gyr_Y_B/32678)*500*GYR_IIR2);
		(*Sensor)->W_Z = ((*Sensor)->W_Z*GYR_IIR1) + (((float)Gyr_Z_B/32678)*500*GYR_IIR2);
	}
}
void Read_Mag(struct BMM350** Sensor){
	// two dummy reads, one is already built in so check data, parenthesis is for the dummy reads
	unsigned char data[11];
	unsigned char length = sizeof(data);
	unsigned char result = I2C_Read(MAG_ADRS,MAG_DATA,data,length);
	if (result==8){
		long Mag_X_B = ((long)data[2]<<16) + ((long)data[1]<<8) + data[0];
		// 		long Mag_X_B = (data[4]*65536) + (data[3]*256) + data[2];
		long Mag_Y_B = ((long)data[5]<<16) + ((long)data[4]<<8) + data[3];
		// 		long Mag_X_B = (data[7]*65536) + (data[6]*256) + data[5];
		long Mag_Z_B = ((long)data[8]<<16) + ((long)data[7]<<8) + data[6];
        // 		long Mag_X_B = (data[10]*65536) + (data[9]*256) + data[8];
		(*Sensor)->Mag_X = ((*Sensor)->Mag_X*MAG_IIR1) + ((float)Mag_X_B*MAG_XY_GAIN*MAG_IIR2);
		(*Sensor)->Mag_Y = ((*Sensor)->Mag_Y*MAG_IIR1) + ((float)Mag_Y_B*MAG_XY_GAIN*MAG_IIR2);
		(*Sensor)->Mag_Z = ((*Sensor)->Mag_Z*MAG_IIR1) + ((float)Mag_Z_B*MAG_Z_GAIN*MAG_IIR2);

	}
}
char BMI088_Int(struct BMI088* Sensor){
// returns 9 if successful, lower numbers indicate stage of error     
    // Accelerometer initialization
    /*
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
	*/
    // Gyroscope initialization
    unsigned char status = I2C_Transmit(GYR_ADRS,GYR_RANGE,0x02); // Set range to +-500deg/s
    if (status!=4){return status;}
    status = I2C_Transmit(GYR_ADRS,GYR_BANDWIDTH,0x07); // Set 100Hz ODR, filter bandwidth of 32Hz
    if (status!=4){return 6;}
    status = I2C_Transmit(GYR_ADRS,GYR_INT_CTRL,0x80); // Enable new data interrupt
    if (status!=4){return 7;}
    status = I2C_Transmit(GYR_ADRS,INT3_INT4_IO_MAP,0x01); // Map data ready interrupt to INT3 pin
    if (status!=4){return 8;}
    // Calibrate gyroscope
	//struct BMI088 CalSensor = {0};
    volatile unsigned long i = 0;
    while(++i<10000000){
        if (g_Gyr_Flag==1){
			g_Gyr_Flag = 0;
			Read_Gyr(&Sensor);
		}
	}
    Sensor->W_X_bias = Sensor->W_X;
    Sensor->W_Y_bias = Sensor->W_Y;
    Sensor->W_Z_bias = Sensor->W_Z;

    return 9;
}
char BMM350_Int(){
// returns 3 if successful, lower numbers indicate stage of error
	unsigned char status = I2C_Transmit(MAG_ADRS,MAG_OTP_CMD,0x80); // Power off OTP
	if (status!=4){return 0;}
	Detain(500UL);
	status = I2C_Transmit(MAG_ADRS,MAG_MODE,0x01); // Set normal mode
	if (status!=4){return 1;}
	status = I2C_Transmit(MAG_ADRS,MAG_INT_CTRL,0x8C); // Enable interrupts, set to push pull
	if (status!=4){return 2;}
		
	return 3;
}
void Motor_Int(){
	// Timer 0 PWM Setup
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM00);
	TCCR0B |= (1<<CS01) | (1<<CS00);
	// Timer 1 Setup
	TCCR1B |= (1<<WGM12) | (1<<CS12) | (1<<CS10); // CTC, /1024 prescaler
	TIMSK1 |= (1<<OCIE1A); // Enable output compare match A interrupt
	OCR1A = MOTOR_CONTROL_TIME;
	// Timer 2 PWM Setup
	TCCR2A |= (1<<COM2A1) | (1<<COM2B1) | (1<<WGM20);
	TCCR2B |= (1<<CS22);
	// ADC Setup
	ADMUX |= (1<<REFS0); // Use Vcc as reference voltage for ADC
	ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Enable ADC, enable interrupt, set /128 prescaler
	// Pin Setup
	DDRD |= (1<<DDD6) | (1<<DDD5) | (1<<DDD3); // Set PD6, PD5, PD3 as output
	DDRB |= (1<<DDB3); // Set PB3 as output
}
void ATMega328P_Int(){
// Use to group MCU one time register setups prior to running while loop
  sei(); 
  // f_SCL = 20000000/(16+2*(TWBR*Prescaler))
  TWBR = 42;
  // f_SCL = 400kHz with above settings, limit of ATMega328P with internal pullup resistors
  PCICR |= (1<<2) | (1<<0); // Enable PC interrupts on port D and B
  PCMSK2 |= (1<<7) | (1<<4) | (1<<2); // Enable PD7, PD4, and PD2 for PC interrupts
  PCMSK0 |= (1<<0); // Enable PB0 for PC interrupts
  DDRB |= (1<<DDB1); // Enable PB1 for OE on logic level shifter
  PORTB |= (1<<PORTB1); // Write PB1 high
  SPI_Int();
  Motor_Int();
}
unsigned char Motor_Control(struct Motor** M){
	volatile int e = (*M)->x-(*M)->r;
	volatile int d_e = e-(*M)->e_last;
	(*M)->e_last = e;
	// Integrate at rate set by timer 0, e_int_limit prevents integrator windup
	(*M)->e_int+=e;
	if ((*M)->e_int>E_INT_LIMIT){(*M)->e_int=E_INT_LIMIT;}
	else if((*M)->e_int<-E_INT_LIMIT){(*M)->e_int=-E_INT_LIMIT;}
	volatile long u = MOTOR_K_P*e + MOTOR_K_I*(*M)->e_int + MOTOR_K_D*d_e;
	u = u>>7; // Divides by 128
	if (u<0){u=0;} else if (u>255){u=255;}
	return u;
}
void Motor_Measure(struct Motor** M){
	ADMUX = (*M)->mux;
	ADCSRA |= (1<<ADSC); // Begin conversion
	while (g_ADC_Flag!=1){asm("");}
	(*M)->x = (*M)->x*MOTOR_C1 + ADC*MOTOR_C2;
	(*M)->x = (*M)->x>>10; // Divides by 1024
	g_ADC_Flag = 0;
}
void Run_BMI(char BMI_Status, struct BMI088* Sensor){
	if (BMI_Status==9){
		if (g_Acc_Flag==1){
			g_Acc_Flag = 0;
			Read_Acc(&Sensor);
		}
		if (g_Gyr_Flag==1){
			g_Gyr_Flag = 0;
			Read_Gyr(&Sensor);
		}
	}
}
void Run_BMM(char BMM_Status, struct BMM350* Sensor){
	if (BMM_Status==3){
		if (g_Mag_Flag==1){
			g_Mag_Flag = 0;
			Read_Mag(&Sensor);
		}
	}
}
void Run_Motors(struct Motor* Motor1, struct Motor* Motor2, struct Motor* Motor3, struct Motor* Motor4){
	if (g_Motor_Flag==1){
		Motor_Measure(&Motor1);
		//Motor_Measure(&Motor2);
		//Motor_Measure(&Motor3);
		//Motor_Measure(&Motor4);
		
		//OCR0A = Motor_Control(&Motor1);
		//OCR0B = Motor_Control(&Motor2);
		//OCR2A = Motor_Control(&Motor3);
		//OCR2B = Motor_Control(&Motor4);
		g_Motor_Flag = 0;
		SPI_Transmit_Long_Data_Visualizer((long)Motor1->r,10,1);
		SPI_Transmit_Long_Data_Visualizer((long)Motor1->x,10,2);
	}
}

ISR(PCINT2_vect){
	if (PIND&(1<<7)){g_Start_Flag = 1;}
    if ((PIND&(1<<4))==(1<<4)){g_Gyr_Flag = 1;}
	if ((PIND&(1<<2))==(1<<2)){g_Acc_Flag = 1;}
}

ISR(PCINT0_vect){
    g_Mag_Flag = 1;
}

ISR (TIMER1_COMPA_vect){
	g_Motor_Flag = 1;
	OCR1A = MOTOR_CONTROL_TIME;
}

ISR (ADC_vect){
	g_ADC_Flag = 1;
}

#endif
