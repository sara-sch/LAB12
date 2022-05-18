/* 
 * File:   mainl12.c
 * Author: saras
 *
 * Created on May 17, 2022, 12:13 PM
 */
// CONFIG1
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

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t address = 0, cont = 0, sleep = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);


/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){                  // Fue interrupción del ADC?
            if(ADCON0bits.CHS == 1){    // Verificamos sea AN0 el canal seleccionado
                PORTC = ADRESH;         // Mostramos ADRESH en PORTC
            }
        PIR1bits.ADIF = 0;              // Limpiamos bandera de interrupci?n  
    } 
    else if(INTCONbits.RBIF){            // Fue interrupción del PORTB  
            if(!PORTBbits.RB0){                     // Verificamos si fue RB0 quien generó la interrupción
                sleep = 0;
            }
            else if(!PORTBbits.RB1){               //Sie estamos despiertos, sleep
                sleep = 1;
            }
            INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción del puerto B
            }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){        
        if(ADCON0bits.GO == 0){ 
            __delay_us(40);
            ADCON0bits.GO = 1;              // Iniciamos proceso de conversi?n
        }
        if(sleep == 1){       //si la bandera esta activada deshabilitar la interrupción ADC        
            PIE1bits.ADIE = 0;
            SLEEP();                  // Pasar al modo sleep
        }
        else if(sleep == 0){            
            PIE1bits.ADIE = 1;
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b10;
    ANSELH = 0;
    
    TRISB = 0b011;
    PORTB = 0;
    
    TRISA = 0b10;
    TRISC = 0;
    
    PORTA = 0;
    PORTC = 0;
    
    OPTION_REGbits.nRBPU = 0;
    WPUB = 0b0011;
    IOCB = 0b0011;
    
    // ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 1;         // Seleccionamos el AN1
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);  
    
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.RBIF = 0;
    INTCONbits.RBIE = 1;
    INTCONbits.GIE = 1;
}

