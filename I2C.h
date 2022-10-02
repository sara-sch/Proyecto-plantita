/*
 * File            : I2C.h
 * Author          : Ligo George
 * Company         : electroSome
 * Project         : I2C Library for MPLAB XC8
 * Microcontroller : PIC 16F877A
 * Created on April 15, 2017, 5:59 PM
 * Link: https://electrosome.com/i2c-pic-microcontroller-mplab-xc8/
 * Modificada por: Pablo Mazariegos con la ayuda del auxiliar Gustavo Ordoñez 
 * Basado en Link: http://microcontroladores-mrelberni.com/i2c-pic-comunicacion-serial/
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef __I2C_H
#define	__I2C_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <pic16f887.h>
#include <stdint.h>

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 4000000
#endif


void I2C_Master_Init(const unsigned long c);

void I2C_Master_Wait(void);

void I2C_Master_Start(void);

void I2C_Master_RepeatedStart(void);

void I2C_Master_Stop(void);

void I2C_Master_Write(unsigned d);

unsigned short I2C_Master_Read(unsigned short a);

void I2C_Slave_Init(uint8_t address);

void I2C_Init (uint32_t frecuencia);//Función de inicialización del periferico I2C.
void I2C_Start (void);              // Función que inicia la comunicación I2C.
void I2C_ReStart (void);            // Función que reinicia la comuncación I2C.
void I2C_Stop (void);               // Función que detiene la comunicación I2c.
void I2C_Ack (void);                // Función para transmitir Acknowledge.
void I2C_NO_Ack (void);             // Función para transmitir NO Acknowledge.
void I2C_Write (uint8_t data);      // Función para escribir el SSPBUF.
uint8_t I2C_Read (void);            // Función para leer el SSPBUF.

#endif	/* __I2C_H */



