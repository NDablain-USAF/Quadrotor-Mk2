// An oldie but a goodie... 
// Practice blinking LED on Arduino Uno board without using Arduino.h
// Created by Nathan Dablain

#include <avr/io.h>
#include <avr/interrupt.h>

// Higher value for output_compare will decrease the frequency
// f_OC = 16000000/(2*N*(1+OCR1A)), N is the prescaler, OCR1A is the output compare value
# define OUTPUT_COMPARE 50000;

int main(){
    sei(); // Enable Interrupts
    DDRB |= (1<<5); // Set D13 as output
    TCCR1A |= (1<<COM1A1); // Set OC1A to clear on compare match
    TCCR1B |= (1<<WGM12) | (1<<CS10) | (1<<CS11); // Set timer 1 to clear on compare match, set prescaler to /64
    TIMSK1 |= (1<<OCIE1A); // Enable compare match A interrupt
    OCR1A = OUTPUT_COMPARE; // Initialize compare match value
    while(1){

    }
}

ISR (TIMER1_COMPA_vect){
    PORTB ^= (1<<5); // Cycle D13
    OCR1A = OUTPUT_COMPARE; // Reset compare match value
}
