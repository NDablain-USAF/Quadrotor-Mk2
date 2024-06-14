// This example shows using timers, pwm, and the analog-to-digital convertor on the ATMega328P without Arduino.h
// Hardware used includes an ATMega328P, 6V DC motor and battery, voltage divider, RC low pass filter, and linear voltage regulator
// Created by Nathan Dablain

#include <avr/io.h>
#include <avr/interrupt.h>

#define controlTime 6
#define e_int_limit 10000

const uint16_t c1 = 985; // IIR filter coefficients for back-emf measurement
const uint16_t c2 = 15;
const uint32_t K_P = 8; // Proportional control gain
const uint32_t K_I = 3; // Integral control gain
const uint16_t r = 37; // Reference voltage measurement, lower measurement means higher back-emf and higher motor speed

// Flags are set in ISRs, affect changes in while loop
volatile uint8_t controlFlag;
volatile uint8_t ADCFlag;

int main(){
  // Timer1/PWM Setup
  sei(); // Enable Interrupts
  TCCR1A |= (1<<COM1A1) | (1<<WGM10); // Clear counter A on match, set 8 bit phase correct PWM
  TCCR1B |= (1<<CS11) | (1<<CS10); // Set /64 prescaler
  // Timer 0 Setup
  // f = f_io/(2*N*(1+OCR0A))... N = prescaler... f_io = 16MHz
  // f = 16e6/(2*1024*(1+6)) = 1116 Hz
  TCCR0A |= (1<<COM0A1) | (1<<WGM01); // Clear on match, set clear timer on compare match mode
  TCCR0B |= (1<<CS02) | (1<<CS00); // Set prescaler to /1024
  TIMSK0 |= (1<<OCIE0A); // Enable Timer 0 compare match A interrupt
  OCR0A = controlTime;
  // ADC Setup
  ADMUX |= (1<<REFS0); // Use Vcc as reference voltage for ADC
  ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Enable ADC, enable interrupt, set /128 prescaler
  // Pin Setup
  DDRB |= (1<<1); // Set D9 as output
  while(1){
    static volatile int16_t e_int;
    static uint32_t x; 

    if (controlFlag==1){
        ADCSRA |= (1<<ADSC); // Begin conversion
        volatile int16_t e = x-r;
        // Integrate at rate set by timer 0, e_int_limits prevent integrator windup
        e_int+=e;
        if (e_int>e_int_limit){e_int=e_int_limit;}
        else if (e_int<-e_int_limit){e_int=-e_int_limit;}
        volatile int32_t u = K_P*e + K_I*e_int;
        u /= 100;
        if (u<0){u=0;} else if (u>255){u=255;}
        OCR1A = u;
        controlFlag = 0;
    }  
    if (ADCFlag==1){
        x = (x*c1 + ADC*c2);
        x /= 1000;
        ADCFlag = 0;
    }
  }
  return 0;
}

 ISR (TIMER0_COMPA_vect){
    controlFlag = 1;
    OCR0A = controlTime;
}

ISR (ADC_vect){
    ADCFlag = 1;
} 
