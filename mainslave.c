/* 
 * File:   mainslave.c
 * Author: saras
 *
 * Created on September 9, 2022, 12:22 PM
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
#include "lcd.h"
#include "spi.h"

#define _XTAL_FREQ 4000000
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 180              // Valor máximo de entrada del potenciometro
#define OUT_MIN 3               // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 6             // Valor máximo de ancho de pulso de señal PWM

void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);


unsigned short CCPR = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal

uint8_t BCD_a_Decimal (uint8_t numero);


char s[];
char a[];
uint8_t flag = 0;
uint8_t horas = 0;                             // Variable de 8 bits para las horas.
uint8_t minutos = 0;                           // Variable de 8 bits para los minutos.
uint8_t segundos = 0;
uint8_t pot = 0;                            
uint8_t old_pot = 0;
uint8_t butt = 0;                           
uint8_t temp = 0;
uint8_t horas_C = 0;                             // Variable de 8 bits para las horas.
uint8_t minutos_C = 0;                           // Variable de 8 bits para los minutos.
uint8_t segundos_C = 0;
uint8_t temp_C = 0;
uint8_t bandera_butt = 0;
uint8_t flag_esp = 0;



void __interrupt() isr(void){
    if(PIR1bits.RCIF)
    {
        if (flag == 1){ // horas
            horas = RCREG;  //Recibo datos
            return;
        }
        
        if (flag == 2){ //minutos
            minutos = RCREG;  //Recibo datos
            return;
        }
        
        if (flag == 3){     //segundos
            segundos = RCREG;  //Recibo datos
            return;
        }
        
        if (flag == 4){     //segundos
            pot = RCREG;  //Recibo datos
            
            if(pot!=old_pot){
            CCPR = map(pot, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                old_pot = pot;
        }
            else{
                return;
            }
            return;
        }
        
        if (flag == 5){     //segundos
            butt = RCREG;  //Recibo datos
            if(butt == 1){
                bandera_butt++;
            }
            return;
        }
        
        if (flag == 6){     //segundos
            temp = RCREG;  //Recibo datos
                if(temp > 27){
                PORTDbits.RD3 = 1;
                PORTDbits.RD2 = 0;
            }
            else{
                PORTDbits.RD3 = 0;
                PORTDbits.RD2 = 0;
            }
            return;
        }
    }
    else if (PIR1bits.SSPIF){
        flag_esp = spiRead(); 
        if(flag_esp == 0){
        spiWrite(horas);
        }
        else if(flag_esp == 1){
        spiWrite(minutos);
        }
        else if(flag_esp == 2){
        spiWrite(segundos);
        }
        else if(flag_esp == 3){
        spiWrite(pot); //pot
        }
        else if(flag_esp == 4){
        spiWrite(butt);
        }
        else if(flag_esp == 5){
        spiWrite(temp);
        }
        PIR1bits.SSPIF = 0;             // Limpiamos bandera de interrupción
    }
    return;
}

void main(void){
    setup();
    
    Lcd_Init(); //inicialización
    Lcd_Clear(); //limpiamos LCD
    

    
    while(1){
        
        if(flag==6){
            flag = 0;
        }
        else{
            flag++;
            if(PIR1bits.TXIF){
                TXREG = flag;
                __delay_ms(100);
            }
        }
        
        if(bandera_butt == 1){
            PORTEbits.RE0 = 1; // dir
        
            for(int x = 0; x < 100; x++){ //steps
            PORTEbits.RE1 = 1;
            __delay_ms(30);
            PORTEbits.RE1 = 0;
            __delay_ms(30);
            }
            bandera_butt++;
        }
        else if(bandera_butt == 3){
            PORTEbits.RE0 = 0; // dir
        
            for(int x = 0; x < 100; x++){ //steps
            PORTEbits.RE1 = 1;
            __delay_ms(30);
            PORTEbits.RE1 = 0;
            __delay_ms(30);
            }
            bandera_butt = 0;
        }
        
        segundos_C = BCD_a_Decimal(segundos);
        minutos_C = BCD_a_Decimal(minutos);
        horas_C = BCD_a_Decimal(horas);
        
        
        Lcd_Set_Cursor(1,1); 
            sprintf(s, "HORA  P:%d B:%d",pot,butt);
            Lcd_Set_Cursor(1,1); 
            Lcd_Write_String(s);
            
            Lcd_Set_Cursor(2,1);
            sprintf(a, "%d%d:%d%d:%d%d  T:%d", horas_C/10 ,horas_C%10, minutos_C/10, minutos_C%10, segundos_C/10, segundos_C%10, temp); 
            Lcd_Set_Cursor(2,1);

            Lcd_Write_String(a);
            
    }
    
   
}





void setup (void){
ANSEL = 0;
ANSELH = 0;
    
TRISB = 0;  //LCD
PORTB = 0;

TRISD = 0b00000010; // Motor dc
PORTD = 0;

TRISE = 0; // Motor stepper
PORTE = 0; 

TRISCbits.TRISC5 = 0;
TRISCbits.TRISC4 = 1;
TRISCbits.TRISC3 = 1;

int_osc_MHz(4); // Oscilador a 4MHz
    
TXSTAbits.SYNC = 0; // Config de TX y RX
TXSTAbits.BRGH = 1;
BAUDCTLbits.BRG16 = 0;

SPBRG = 25; // Baudrate 9600
SPBRGH = 0;

RCSTAbits.SPEN = 1;
RCSTAbits.RX9 = 0;
RCSTAbits.CREN = 1;     
TXSTAbits.TXEN = 1; 
// TXIE solo se enciende cuando se transmiten datos

PIR1bits.RCIF = 0;  //Config int
PIE1bits.RCIE = 1;
INTCONbits.PEIE = 1;
INTCONbits.GIE = 1;


// Configuración PWM
TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
PR2 = 12;                  // periodo de 20ms

// Configuración CCP
CCP1CON = 0;                // Apagamos CCP1
CCP1CONbits.P1M = 0;        // Modo single output
CCP1CONbits.CCP1M = 0b1100; // PWM

CCPR1L = 5;
CCP1CONbits.DC1B0 = 5 & 0b11;    // 2ms ancho de pulso / 25% ciclo de trabajo

PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
T2CONbits.TMR2ON = 1;       // Encendemos TMR2
while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente

TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM

PIE1bits.SSPIE = 1;
PIR1bits.SSPIF = 0; 


spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_ACTIVE_2_IDLE);
}

uint8_t BCD_a_Decimal (uint8_t numero)            // Función que convierte un número BCD a decimal.
{
  return ((numero >> 4) * 10 + (numero & 0x0F));  // Retornamos la variable "numero" desplazado 4 posiciones a la izquierda, multiplicado por 10 más "numero" &&  1111.
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}