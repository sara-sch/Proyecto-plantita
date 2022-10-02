/* 
 * File:   adc.h
 * Author: saras
 *
 * Created on September 16, 2022, 4:23 PM
 */

#ifndef ADC_H
#define	ADC_H

#include <xc.h>
#include <stdint.h>

//Definimos funciones
void adc_init(uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus);
void adc_start(uint8_t channel);
int adc_read(void);

#endif	/* ADC_H */

