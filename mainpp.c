/* 
 * File:   mainpp.c
 * Author: saras & alegangster
 *
 * Created on September 9, 2022, 11:56 AM
 */

// CONFIG1
#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "oscilador_1.h"
#include "adc_1.h"
#include "I2C.h"
#include "lcd.h"

#define _XTAL_FREQ 4000000
#define ADDRESS 0b01001000


uint8_t POT = 0;
uint8_t BUTT = 0;
unsigned short GRADOS_POT = 0;


float temp;

uint8_t z = 0;
uint8_t hours = 0;                             // Variable de 8 bits para las horas.
uint8_t minutes = 0;                           // Variable de 8 bits para los minutos
uint8_t seconds = 0; 
uint8_t temp1 = 0; 
uint8_t temp2 = 0; 
uint8_t flag = 0;
uint8_t dato = 0;

//char s[]; // prueba lcd
//char a[]; // prueba lcd

uint8_t BCD_a_Decimal (uint8_t numero);
uint8_t Decimal_a_BCD (uint8_t numero);    // Función que convierte un número decimal a BCD.

void setup(void);

unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, //Función de map
            unsigned short out_min, unsigned short out_max);


void __interrupt() isr(void){   
    if(PIR1bits.ADIF){
            POT = adc_read();
        }
    if(PIR1bits.RCIF){
            flag = RCREG;
            PORTD = flag;
            if(PIR1bits.TXIF)
            {
                if(flag == 1){
                    TXREG = hours;
                }
                else if (flag == 2){
                    TXREG = minutes;
                }
                else if (flag == 3){
                    TXREG = seconds;
                }
                else if (flag == 4){
                    TXREG = GRADOS_POT;
                }
                else if (flag == 5){
                    TXREG = BUTT;
                }
                else if (flag == 6){
                    TXREG = temp;
                }
            }
        }
}

void main(void){
    setup();
    
//    Lcd_Init(); //inicialización
//    Lcd_Clear(); //limpiamos LCD
    
    // sensor de reloj
    seconds = Decimal_a_BCD(seconds);
    minutes = Decimal_a_BCD(minutes);
    hours = Decimal_a_BCD(hours);
    
    I2C_Start();                // Llamamos a la función Start.
    I2C_Write(0xD0);            // Escribimos en SSPBUF la dirección de DS1307 1101000 + 0 de escritura.
    I2C_Write(0x00);            // Dirección de segundos.
    I2C_Write(seconds);            // Reiniciamos los segundos.
    I2C_Write(minutes);         // Cargamos el valor de minutos en la dirección de minutos.
    I2C_Write(hours);           // Cargamos el valor de horas en la dirección de horas.
    I2C_Stop();                 // Llamamos a la función Stop.
    __delay_ms(200);            // Retardo de 200 ms.
    
    
    
    while(1){
        
         adc_start(0); //iniciamos lectura ADC
         
         BUTT = PORTAbits.RA1;
         
         I2C_Start();                        // I2C para RTC
            I2C_Write(0xD0);                    // Escribimos en SSPBUF la dirección de DS1307 1101000 + 0 de escritura.
            I2C_Write(0);                       // Dirección de segundos.
            I2C_ReStart();                      // Llamamos a la función ReStart.
            I2C_Write(0xD1);                    // Escribimos en SSPBUF la dirección de DS1307 1101000 +1 de lectura.
            seconds=I2C_Read();                 // Cargamos la variable "seconds" con el valor de SSPBUF.
            I2C_Ack();                          // ACK.
            minutes=I2C_Read();                 // Cargamos la variable "minutes" con el valor de SSPBUF.
            I2C_Ack();                          // ACK.
            hours=I2C_Read();                   // Cargamos la variable "hours" con el valor de SSPBUF.
            I2C_NO_Ack();                       // NO ACK.
            I2C_Stop();                         // Llamamos a la función Stop.

            __delay_ms(50);                     // Retardo de 50 ms.   
            
            //sensor de temperatura 
            dato = (uint8_t)((ADDRESS<<1)+ 0b1);
            I2C_Start();                        // I2C para temp
            I2C_Write(dato);                    // Escribimos en SSPBUF la dirección de CJMCU-75 01001000 
            temp = I2C_Read();                 // Cargamos la variable "seconds" con el valor de SSPBUF.
            I2C_NO_Ack();
            I2C_Stop();                         // Llamamos a la función Stop.

            __delay_ms(250);                     // Retardo  
            

         
            GRADOS_POT = map(POT, 0, 255, 0, 180); //map los grados del servo de 0 a 180
            

    }
}
    


void setup (void){
    
    int_osc_MHz(4); // Oscilador a 4MHz
    
    TXSTAbits.SYNC = 0; // Config de TX y RX
    TXSTAbits.BRGH = 1;
    BAUDCTLbits.BRG16 = 0;
    
    SPBRG = 25; // Baudrate 9600
    SPBRGH = 0;
    
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;
    
    TXSTAbits.TXEN = 1;   // cero para que no transmitiera 
    
    adc_init(1,0,0);        // ADC
    ANSEL = 0b1;
    ANSELH = 0;
    TRISA = 0b00000011;     // A0 pot A1 bot
    PORTA = 0;
    

    TRISD = 0;
    PORTD = 0;   
    
    //interrupciones 
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    PIR1bits.RCIF = 0;  //Config int
    PIE1bits.RCIE = 1;
    
    I2C_Master_Init(100000);
}


uint8_t Decimal_a_BCD (uint8_t numero)            // Función que convierte un número decimal a BCD. 
{
    return (((numero / 10) << 4) + (numero % 10));// Retornamos la decena de la variable "numero" desplazado 4 bits a la derecha y las unidades de la variable "numero" en el nibble menos significativo
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,   // funcion para map de servo 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
