#ifndef SPI_H
#define SPI_H

#include <avr/io.h>
#include <stdio.h>

#define F_CPU 20000000UL

void SPI_Init();
void Transmit(long outbound);

#endif