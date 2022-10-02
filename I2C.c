 /*
 * File            : I2C.c
 * Author          : Ligo George
 * Company         : electroSome
 * Project         : I2C Library for MPLAB XC8
 * Microcontroller : PIC 16F877A
 * Created on April 15, 2017, 5:59 PM
 * Link: https://electrosome.com/i2c-pic-microcontroller-mplab-xc8/
 * Modificada por: Pablo Mazariegos con la ayuda del auxiliar Gustavo Ordo�ez 
 * Basado en Link: http://microcontroladores-mrelberni.com/i2c-pic-comunicacion-serial/
 */
#include "I2C.h"

void I2C_Master_Init(const unsigned long c)
{
    SSPCON = 0b00101000;
    SSPCON2 = 0;
    SSPADD = (_XTAL_FREQ/(4*c))-1;
    SSPSTAT = 0;
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
}

void I2C_Master_Wait()
{
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Master_Start()
{
    I2C_Master_Wait();
    SSPCON2bits.SEN = 1;
}

void I2C_Master_RepeatedStart()
{
    I2C_Master_Wait();
    SSPCON2bits.RSEN = 1;
}

void I2C_Master_Stop()
{
    I2C_Master_Wait();
    SSPCON2bits.PEN = 1;
}

void I2C_Master_Write(unsigned d)
{
    I2C_Master_Wait();
    SSPBUF = d;             
}

unsigned short I2C_Master_Read(unsigned short a)
{
    unsigned short temp;
    I2C_Master_Wait();
    SSPCON2bits.RCEN = 1;
    I2C_Master_Wait();
    temp = SSPBUF;
    I2C_Master_Wait();
    if(a == 1){
        SSPCON2bits.ACKDT = 0;
    }else{
        SSPCON2bits.ACKDT = 1;
    }
    SSPCON2bits.ACKEN = 1;
    return temp;
}

void I2C_Slave_Init(uint8_t address)
{ 
    SSPADD = address;
    SSPCON = 0x36;
    SSPSTAT = 0x80;
    SSPCON2 = 0x01;
    TRISC3 = 1;
    TRISC4 = 1;
    GIE = 1;
    PEIE = 1;
    SSPIF = 0;
    SSPIE = 1;
}

void I2C_Init (uint32_t frecuencia)//Funci�n de inicializaci�n del periferico I2C.    
{
    TRISB|=(1<<0);           // Configuramos como entrada digital el pin RB0 (SDA) debido a las resistencias Pull Up.
    TRISB|=(1<<1);           // Configuramos como entrada digital el pin RB1 (SCL) debido a las resistencias Pull Up.
    
    SSPSTATbits.CKE=0;       // Registro SSPSTAT MSSP STATUS REGISTER.
                             // CKE (bit 6) SMBus Select bit.
                             // CKE=0 >> SMbus deshabilitado entradas espec�ficas.
                             // CKE=1 >> SMbus habilitado entradas espec�ficas.
    
    SSPSTATbits.SMP=1;       // Registro SSPSTAT MSSP STATUS REGISTER.
                             // SMP (bit 7) Slew Rate Control bit
                             // SMP=0 >> modo velocidad standard 100KHz.
                             // SMP=1 >> modo alta velocidad 400KHz.
    
    SSPCONbits.SSPEN=1;     // Registro SSPCON1 MSSP CONTROL REGISTER 1 (MODO I2C).
                             // SSPEN (bit 5) Master Synchronous Serial Port Enable bit
                             // SSPEN=0 >> Deshabilita el puerto serie y los pines RB0 y RB1 son configurados como I/O digitales.
                             // SSPEN=1 >> Habilita el puerto serie y los pines RB0 y RB1 son establecidos como SDA y SCL respectivamente.
    
    SSPCONbits.SSPM=0b1000; // Registro SSPCON1 MSSP CONTROL REGISTER (MODO I2C).
                             // SSPM3-SSPM0 Master Synchronous Serial Port Mode Select bits.
                             // 1111 >> I2C modo esclavo, direcci�n 10 bits, con bits Start Stop, Interrupci�n habilitada.
                             // 1110 >> I2C modo esclavo, direcci�n 7 bits, con bits Start Stop, Interrupci�n habilitada.
                             // 1011 >> Firmware controlado por el maestro (esclavo deshabilitado).
                             // 1000 >> Modo Maestro, clock=Fosc/(4*(SSPADD+1)).
                             // 0111 >> Modo esclavo, direccion 10 bits.
                             // 0110 >> Modo esclavo, direccion 7 bits.
    
    SSPCON2=0x00;            // Registro SSPCON2 MSSP CONTROL REGISTER 2 (modo I2C).
    
    SSPADD = ((_XTAL_FREQ/(4*frecuencia))-1); // clock=Fosc/(4*(SSPADD+1)).                               
}

/*==========================================================================================================
 ===========================================================================================================*/
void I2C_Start (void)       // Funci�n que inicia la comunicaci�n I2C.
{
    SSPCON2bits.SEN=1;      // Registro SSPCON2 MSSP CONTROL REGISTER 2 (modo I2C)
                            // SEN (bit 0) Start Condition Enable/Stretch Enable bit.
                            // SEN=0 >> Condici�n Start a la espera.
                            // SEN=1 >> Condici�n Start habilitada para SDA y SCL, SEN se pone a 0 automaticamente por hardware.
    while(SSPCON2bits.SEN); // Esperamos hasta que se termine de establecer la condici�n de inicio.
}

/*==========================================================================================================
 ===========================================================================================================*/
void I2C_ReStart (void)     // Funci�n que reinicia la comuncaci�n I2C.
{
    SSPCON2bits.RSEN=1;     // Registro SSPCON2 MSSP CONTROL REGISTER 2 (modo I2C)
                            // RSEN (bit 1) Repeated Start Condition Enable bit.
                            // RSEN=0 >> Condici�n Restart a la espera.
                            // RSEN=1 >> Condici�n Restart habilitada para SDA y SCL, SEN se pone a 0 automaticamente por hardware.
    while(SSPCON2bits.RSEN); // Esperamos hasta que se termine de establecer la condici�n de inicio.
}

/*==========================================================================================================
 ===========================================================================================================*/
void I2C_Stop (void)        // Funci�n que detiene la comunicaci�n I2c.
{
    SSPCON2bits.PEN=1;      // Registro SSPCON2 MSSP CONTROL REGISTER 2 (modo I2C).
                            // PEN (bit 2) Stop Condition Enable bit.
                            // PEN=0 >> Condici�n de Stop a la espera.
                            // PEN=1 >> Condici�n Stop habilitada para SDA y SCL, PEN se pone a 0 automaticamente por hardware.
    while(SSPCON2bits.PEN); // Esperamos hasta que se termine de establecer la condici�n de parada.
}

/*==========================================================================================================
 ===========================================================================================================*/
void I2C_Ack (void)         // Funci�n para transmitir Acknowledge.
{
    PIR1bits.SSPIF=0;       // Master Synchronous Serial Port Interrupt Flag bit
                            // SSPIF=0 >> A la espera de la transmisi�n o recepci�n 
                            // SSPIF=1 >> La transmisi�n o recepci�n ha sido completada, este bit se limpia por hardware.
            
    SSPCON2bits.ACKDT=0;    // Registro SSPCON2 MSSP CONTROL REGISTER 2 (modo I2C).
                            // ACKDT (bit 5) Acknowledge Data bit
                            // ACKDT=0 >> Acknowledge
                            // ACKDT=1 >> NO Acknowledge.
    
    SSPCON2bits.ACKEN=1;    // Registro SSPCON2 MSSP CONTROL REGISTER 2 (modo I2C). 
                            // ACKEN (bit 4) Acknowledge Sequence Enable bit.
                            // ACKEN=0 >> Secuencia Acknowledge a la espera.
                            // ACKEN=1 >> Inicia la secuencia Acknowledge, este bit se limpia automaticamente por hardware.
    
    while(PIR1bits.SSPIF==0);//Esperamos hasta que se complete la transmisi�n/recepci�n.
}

/*==========================================================================================================
 ===========================================================================================================*/
void I2C_NO_Ack (void)      // Funci�n para transmitir NO Acknowledge.
{
    PIR1bits.SSPIF=0;       // Master Synchronous Serial Port Interrupt Flag bit
                            // SSPIF=0 >> A la espera de la transmisi�n o recepci�n 
                            // SSPIF=1 >> La transmisi�n o recepci�n ha sido completada, este bit se limpia por hardware.
            
    SSPCON2bits.ACKDT=1;    // Registro SSPCON2 MSSP CONTROL REGISTER 2 (modo I2C).
                            // ACKDT (bit 5) Acknowledge Data bit
                            // ACKDT=0 >> Acknowledge
                            // ACKDT=1 >> NO Acknowledge.
    
    SSPCON2bits.ACKEN=1;    // Registro SSPCON2 MSSP CONTROL REGISTER 2 (modo I2C). 
                            // ACKEN (bit 4) Acknowledge Sequence Enable bit.
                            // ACKEN=0 >> Secuencia Acknowledge a la espera.
                            // ACKEN=1 >> Inicia la secuencia Acknowledge, este bit se limpia automaticamente por hardware.
    
    while(PIR1bits.SSPIF==0);//Esperamos hasta que se complete la transmisi�n/recepci�n.
}

/*==========================================================================================================
 ===========================================================================================================*/
void I2C_Write (uint8_t data)// Funci�n para escribir el SSPBUF.
{
    PIR1bits.SSPIF=0;        // Master Synchronous Serial Port Interrupt Flag bit
                             // SSPIF=0 >> A la espera de la transmisi�n o recepci�n.
                             // SSPIF=1 >> La transmisi�n o recepci�n ha sido completada, este bit se limpia por hardware.
    
    SSPBUF=data;             // Cargamos el registro SSPBUF con los valores de la variable "data".
    
    while(PIR1bits.SSPIF==0);//Esperamos hasta que se complete la transmisi�n.
}

/*==========================================================================================================
 ===========================================================================================================*/
uint8_t I2C_Read (void)      // Funci�n para leer el SSPBUF.
{
    PIR1bits.SSPIF=0;        // Master Synchronous Serial Port Interrupt Flag bit
                             // SSPIF=0 >> A la espera de la transmisi�n o recepci�n.
                             // SSPIF=1 >> La transmisi�n o recepci�n ha sido completada, este bit se limpia por hardware.
    
    SSPCON2bits.RCEN=1;      // Registro SSPCON2 MSSP CONTROL REGISTER 2 (modo I2C).
                             // RCEN (bit 3) Receive Enable bit (Master Receive mode only).
                             // RCEN=0 >> recepci�n a la espera.
                             // RCEN=1 >> recepci�n habilitada.
    
    while(PIR1bits.SSPIF==0);// Esperamos hasta que se complete la recepci�n.
    
    return SSPBUF;           // Retornamos el valor de SSPBUF en una variable entera de 8 bits.
}
//*****************************************************************************

/*

Ejemplo de interrupci�n para el dispositivo Perif�rico. Note que no todas las
variables est�n definidas dentro de la interrupci�n.

void __interrupt() isr(void){
   if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL))
        {
            z = SSPBUF;
            SSPCONbits.SSPOV = 0;
            SSPCONbits.WCOL = 0;
            SSPCONbits.CKP = 1;
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW)
        {
            //__delay_us(7);
            z = SSPBUF;
            //__delay_us(2);
            PIR1bits.SSPIF = 0;
            SSPCONbits.CKP = 1;
            while(!SSPSTATbits.BF);
            PORTD = SSPBUF;
            __delay_us(250);
            
        }
        else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW)
        {
            z = SSPBUF;
            BF = 0;
            SSPBUF = PORTB;
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0;    
    }
}

*/


