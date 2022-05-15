/*
 * File:   Pre.c
 * Author: andre
 *
 * Created on May 11, 2022, 3:24 PM
 */


/// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT                // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF                           // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF                          // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF                          // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF                             // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF                            // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF                          // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF                           // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF                          // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                            // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V                       // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF                            // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 1000000
#define FLAG_SPI 0x0F
#define IN_START_VAL 0           
#define IN_END_VAL 255        
#define MIN_OUTPUT 40       
#define MAX_OUTPUT 130

uint8_t potenciometro = 0;
uint8_t cont_slave = 0;
char val_temporal = 0;
unsigned short CCPR = 0;

void setup(void);
unsigned short map(uint8_t val, 
                   uint8_t in_start_val, 
                   uint8_t in_end_val,
unsigned short min_output, 
unsigned short max_output);

// CONFIG INT
void __interrupt() isr (void){
    if(PORTAbits.RA0){                              // INT in Master
        
        //MASTER INT
        if(PIR1bits.ADIF){                          // ADC INT confirmation
            potenciometro = ADRESH;
            PIR1bits.ADIF = 0;                      // Limpiar
        }
    }
    
    else {                                          // INT in Esclavo
        if(PIR1bits.SSPIF){                         // Data reception
            val_temporal = SSPBUF;
            if (val_temporal != FLAG_SPI){          // CLK pulse
                PORTD = val_temporal;               // DISPLAY valor recibido en PORTD
                
                SSPBUF = cont_slave;                // Contador del esclavo para buffer
            }
            else {
                SSPBUF = cont_slave;
            }
            PIR1bits.SSPIF = 0;                     // Limpiamos
        }
        else if(INTCONbits.RBIF){                   // INT en PORTB
            
            if(!PORTBbits.RB0)                      // RB0 generó?
                cont_slave++;                       // ++ PORTA
            
            else if (!PORTBbits.RB1)                // Si es RB2
                cont_slave--;                       // -- PORTA
            INTCONbits.RBIF = 0;                    // Limpiamos
        }
        
    }
    return;
}

// MAIN LOOP
void main(void) {
    setup();
    while(1){
        
        if(PORTAbits.RA0){ 
            
            if(ADCON0bits.GO == 0){ 
                ADCON0bits.GO = 1;       
            }
            
            SSPBUF = potenciometro;                           // Se envia el valor del potenciometro
            while(!SSPSTATbits.BF){}                // Espera
            
            PORTAbits.RA7 = 1;                      // SLAVE ON
            __delay_ms(10);  
            PORTAbits.RA7 = 0;                      // SLAVE OFF
            __delay_ms(10);

            SSPBUF = FLAG_SPI;                      // Call para que el esclavo retorne el valor
 
            while(!SSPSTATbits.BF){}                // Espera
            PORTD = SSPBUF;                         // Mostrar lo que se recibio en el PUERTOD
            __delay_ms(10);
        }
        else{
            CCPR = map(PORTD, IN_START_VAL, IN_END_VAL, MIN_OUTPUT, MAX_OUTPUT); // PWM
            CCPR1L = (uint8_t)(CCPR>>2);            // 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11;         // 2 bits menos significativos en DC1B
        }
        if(PORTAbits.RA2){
            PORTD = 0;                              // Ignorar para ESclavo 2
        }
    }
    return;
}

// CONFIG
void setup(void){
    ANSEL = 0x02;         
    ANSELH = 0;            

    OSCCONbits.IRCF = 0b100; 
    OSCCONbits.SCS = 1;     

    TRISA = 0b00100111;  
    PORTA = 0;

    

    TRISD = 0;
    PORTD = 0;

    if(PORTAbits.RA0){

        // Configuraciones del SPI
        TRISC = 0b00010000;      
        PORTC = 0;

        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0000;  
        SSPCONbits.CKP = 0;         
        SSPCONbits.SSPEN = 1;     
        
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        
        SSPSTATbits.SMP = 1;     
        SSPBUF = potenciometro;            


        // CONFIG ADC
        ADCON0bits.ADCS = 0b00;                     // Fosc/2
        ADCON1bits.VCFG0 = 0;                       // VDD
        ADCON1bits.VCFG1 = 0;                       // VSS
        ADCON0bits.CHS = 1;                         // Seleccionamos el AN1
        ADCON1bits.ADFM = 0;                        // Justificado a la izquierda
        ADCON0bits.ADON = 1;                        // Habilitamos modulo ADC
        __delay_us(40);                             // Sample time

        // CONFIG INT
        PIR1bits.ADIF = 0;                          // Limpiamos bandera de ADC
        PIE1bits.ADIE = 1;                          // Habilitamos interrupcion de ADC
        INTCONbits.PEIE = 1;                        // Habilitamos int. de perifericos
        INTCONbits.GIE = 1;                         // Habilitamos int. globales
    }
    // SLAVE CONFIG
    else{
        TRISC = 0b00011000; 
        PORTC = 0;
        TRISB = 0xFF;           

        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0100;  
        SSPCONbits.CKP = 0;     
        SSPCONbits.SSPEN = 1;  
        
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;   
        SSPSTATbits.SMP = 0;     

        PIR1bits.SSPIF = 0; 
        PIE1bits.SSPIE = 1;     
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
        
        OPTION_REGbits.nRBPU = 0; 
        WPUB = 0x03;
        INTCONbits.RBIE = 1;   
        IOCB = 0x03;         
        INTCONbits.RBIF = 0;
        
        // Configuracion PWM
        TRISCbits.TRISC2 = 1;                       // Deshabilitamos salida de CCP1
        PR2 = 250;                                  // 2ms

        // Configuraci?n CCP
        CCP1CON = 0;                                // OFF CCP1
        CCP1CONbits.P1M = 0;                        // Modo single output
        CCP1CONbits.CCP1M = 0b1100;                 // PWM

        CCPR1L = 5>>2;
        CCP1CONbits.DC1B = 5 & 0b11;                // .5 ms ancho de pulso / 25% ciclo de trabajo

        PIR1bits.TMR2IF = 0;                        // Limpiamos
        T2CONbits.T2CKPS = 0b11;                    // Prescaler 1:16
        T2CONbits.TMR2ON = 1;                       // TMR2 ON
        while(!PIR1bits.TMR2IF);                    // Esperar un ciclo
        PIR1bits.TMR2IF = 0;                        // Limpiamos

        TRISCbits.TRISC2 = 0;                       // Habilitamos salida de PWM

    }
}

// Funcion para el mapeo
unsigned short map(uint8_t x, 
                   uint8_t x0, 
                   uint8_t x1,
unsigned short y0, 
unsigned short y1)
{
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}