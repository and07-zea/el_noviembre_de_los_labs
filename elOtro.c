/* File:   elOtro.c
 * Author: andre
 *
 * Created on May 11, 2022, 3:59 PM
 */

// CONFIG 1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG 2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 1000000

uint8_t potenciometro = 0;
uint8_t flags = 0;
uint8_t contador = 0;

void setup(void);

// CPNFIG INT

void __interrupt() isr (void)
{
    if(PORTAbits.RA0){
        if(PIR1bits.ADIF){
            potenciometro = ADRESH;
            PIR1bits.ADIF = 0;
        }
    }
    return;
}

// Main Loop
void main(void) {
    setup();
    while(1){
 
        if(PORTAbits.RA0){                              //Se comprueba que no sea maestro          
            
            if(SSPSTATbits.BF){              //Se comprueba que no hay comunicaci�n en el proceso     
                flags = 1;
                SSPBUF = potenciometro;                  //Se mueve el valor del contador para as� enviarlo
            }
           
            if(ADCON0bits.GO == 0){
                ADCON0bits.GO = 1;          // Configuracion
            }
        }
    }
    return;
}

// CONFIGURACION
void setup(void){
    ANSEL = 0;
    ANSELH = 0;              

    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno

    TRISA = 0b00000111;         // SS y RA0 como entradas
    PORTA = 0;

    TRISD = 0;
    PORTD = 0;

    // SPI CONFIG
    
    if(PORTAbits.RA0){
        
        ANSEL = 0b00001000;
        TRISA = 0b00001111;         // SS y RA0 como entradas
        TRISC = 0b00010000;         // -> SDI entrada, SCK y SD0 como salida
        PORTC = 0;

        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
        SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
        SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
        SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
        SSPBUF = potenciometro;              // Enviamos un dato inicial

        // Configuracion interrupciones
        PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
        PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
        INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
        INTCONbits.GIE = 1;         // Habilitamos int. globales
        
        // Configuraci�n ADC
        ADCON0bits.ADCS = 0b01;     // Fosc/8
        ADCON1bits.VCFG0 = 0;       // VDD
        ADCON1bits.VCFG1 = 0;       // VSS
        ADCON0bits.CHS = 0b0011;    // Seleccionamos el AN3
        ADCON1bits.ADFM = 0;        // Justificado a la izquierda
        ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
        __delay_us(40);             // Sample time
    }
}
