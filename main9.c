/*
 * File:   main9.c
 * Author: Oscar Escriba
 *
 * Created on 21 de abril de 2023, 10:14 AM
 */

// CONFIG1
/* Created on 21 de abril de 2023, 10:14 AM
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
#include <stdint.h>
//DEFINCION DE FRECUENCIA PARA DELAY 
#define _XTAL_FREQ 1000000 

//variables globales 
uint8_t pot_in =0; 
uint8_t adress =0x00; 
int pic_sleep =0; 
//proto funciones 
void setup(void); 
uint8_t EEPROM_read(uint8_t adress); 
void EEPROM_write(uint8_t adress, uint8_t data); 
//interrupciones  
void __interrupt() isr(void ) {  
    if (PIR1bits.ADIF) { 
        if (ADCON0bits.CHS==0) { 
            pot_in = ADRESH; 
            PORTC = pot_in; 
        } 
        PIR1bits.ADIF=0; 
    } 
    if (INTCONbits.RBIF){ 
        if (PORTBbits.RB0==0){ 
            pic_sleep =0; 
            PORTEbits.RE0=0; 
        } 
        else if (PORTBbits.RB1 ==0){ 
            pic_sleep =1; 
            PORTEbits.RE0=1; 
            SLEEP(); 
        } 
        else if (PORTBbits.RB2==0){ 
            pic_sleep=0; 
            PORTEbits.RE0=0; 
            EEPROM_write(adress, pot_in); 
        } 
        INTCONbits.RBIF=0;
    } 
    return; 
}
void main(void) { 
    setup(); 
    while (1){ 
        if (pic_sleep ==0){ 
            if (ADCON0bits.GO==0) { 
                ADCON0bits.GO=1; 
                __delay_us(40); 
            }
        } 
        PORTD = EEPROM_read(adress); 
    }
    return;
} 
//configuracion de los puertos...
void setup(void) {
    ANSEL =0b00000001; 
    ANSELH=0; 
    TRISA=0b00000001; 
    PORTA=0; 
    TRISC =0; 
    PORTC=0; 
    TRISD=0; 
    PORTD=0; 
    TRISE=0; 
    PORTE=0; 
    
    //cofiguracion del osccon
    OSCCONbits.IRCF=0b0100; 
    OSCCONbits.SCS=1; 
    //config de interrupciones 
    INTCONbits.GIE=1; 
    INTCONbits.PEIE=1; 
    INTCONbits.RBIE=1; 
    IOCBbits.IOCB0=1; 
    IOCBbits.IOCB1=1; 
    IOCBbits.IOCB2=1; 
    INTCONbits.RBIF=0;
    PIR1bits.ADIF=0; 
    PIE1bits.ADIE=1; 
    
    //configuracion pushbuttoms en portb 
    TRISBbits.TRISB0=1; 
    TRISBbits.TRISB1=1; 
    TRISBbits.TRISB2=1; 
    OPTION_REGbits.nRBPU=0; 
    WPUBbits.WPUB0=1; 
    WPUBbits.WPUB1=1; 
    WPUBbits.WPUB2=1; 
    //configuraciin del adc  
    ADCON0bits.ADCS=0b01; 
    ADCON1bits.VCFG0=0; 
    ADCON1bits.VCFG1=0; 
    
    ADCON0bits.CHS=0b0000; 
    ADCON1bits.ADFM=0; 
    ADCON0bits.ADON=1; 
    __delay_us(40); 
} 
//lectura de la eeprom 
uint8_t EEPROM_read(uint8_t adress){ 
    EEADR == adress; 
    EECON1bits.EEPGD=0; 
    EECON1bits.RD=1; 
    return EEDAT; 
}

//escritura de la eeprom 
void EEPROM_write (uint8_t adress, uint8_t data){ 
    EEADR = adress; 
    EEDAT =data; 
    EECON1bits.EEPGD=0; 
    EECON1bits.WREN=1; 
    INTCONbits.GIE=0; 
    EECON2 = 0x55; 
    EECON2 = 0xAA; 
    EECON1bits.WR=1; 
    EECON1bits.WREN=0;
    INTCONbits.RBIF=0; 
    INTCONbits.GIE=1; 
}
