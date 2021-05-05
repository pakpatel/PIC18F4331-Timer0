/*
 * File:   homework6.c
 * Author: partw
 *
 * Created on February 28, 2021, 2:15 PM
 */

#ifndef CONFIG_H
#define CONFIG_H
#include <xc.h>

#define _XTAL_FREQ 1000000

// PIC18F4331 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = IRCIO // Oscillator Selection bits (Internal oscillator block, port function on RA6 and port function on RA7)
#pragma config FCMEN = ON // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON // Internal External Oscillator Switchover bit (Internal External Switchover mode enabled)

// CONFIG2L
#pragma config PWRTEN = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bits (Brown-out Reset disabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDTEN = OFF // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDPS = 32768 // Watchdog Timer Postscale Select bits (1:32768)
#pragma config WINEN = OFF // Watchdog Timer Window Enable bit (WDT window disabled)

// CONFIG3L
#pragma config PWMPIN = OFF // PWM output pins Reset state control (PWM outputs disabled upon Reset (default))
#pragma config LPOL = HIGH // Low-Side Transistors Polarity (PWM0, 2, 4 and 6 are active-high)
#pragma config HPOL = HIGH // High-Side Transistors Polarity (PWM1, 3, 5 and 7 are active-high)
#pragma config T1OSCMX = ON // Timer1 Oscillator MUX (Low-power Timer1 operation when microcontroller is in Sleep mode)

// CONFIG3H
#pragma config FLTAMX = RC1 // FLTA MUX bit (FLTA input is multiplexed with RC1)
#pragma config SSPMX = RC7 // SSP I/O MUX bit (SCK/SCL clocks and SDA/SDI data are multiplexed with RC5 and RC4, respectively. SDO output is multiplexed with RC7.)
#pragma config PWM4MX = RB5 // PWM4 MUX bit (PWM4 output is multiplexed with RB5)
#pragma config EXCLKMX = RC3 // TMR0/T5CKI External clock MUX bit (TMR0/T5CKI external clock input is multiplexed with RC3)
#pragma config MCLRE = ON // MCLR Pin Enable bit (Enabled)

// CONFIG4L
#pragma config STVREN = ON // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF // Low-Voltage ICSP Enable bit (Low-voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = OFF // Code Protection bit (Block 0 (000200-000FFFh) not code-protected)
#pragma config CP1 = OFF // Code Protection bit (Block 1 (001000-001FFF) not code-protected)

// CONFIG5H
#pragma config CPB = OFF // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF // Write Protection bit (Block 0 (000200-000FFFh) not write-protected)
#pragma config WRT1 = OFF // Write Protection bit (Block 1 (001000-001FFF) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF // Table Read Protection bit (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF // Table Read Protection bit (Block 1 (001000-001FFF) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#endif / XC_HEADER_TEMPLATE_H /

#include <stdint.h>

uint16_t count = 1000; //counter for timer
uint16_t flag = 0; //flag that tells the program timer is done

void __interrupt() high_isr(void);
void __interrupt(low_priority) low_isr(void);

void main(void){
    
    OSCCON=0x73; //osciallator setup to 4MHz
         
    TRISBbits.RB0=0;        //Set RB0 as output
    TRISBbits.RB1=0;        //Set RB1 as output
    TRISBbits.RB2=0;        //Set RB2 as output
    TRISBbits.RB3=0;        //Set RB3 as output
    TRISBbits.RB4=0;        //Set RB4 as output
    
    LATB = 0x00;       //all leds start off

    TMR0=0xBFA; //clear timer0
    T0CON=0x82; //sets bits for T0CON register 10000010

    RCONbits.IPEN = 1;      //Enable priority levels

    INTCONbits.TMR0IE = 1; //enable timer0
    INTCON2bits.TMR0IP = 1;
    INTCONbits.GIEH = 1;    //Enable high interrupts
    INTCONbits.GIEL = 1;    //Enable low interrupts
   
    while(1){ //loop to keep program running until interrupts occur
        if(flag == 1){
        LATBbits.LB0 = ~LATBbits.LB0;       //toggle leds
        LATBbits.LB1 = ~LATBbits.LB1;
        LATBbits.LB2 = ~LATBbits.LB2;
        LATBbits.LB3 = ~LATBbits.LB3;
        LATBbits.LB4 = ~LATBbits.LB4;
        count =1000; //resets count value that controls timing
        flag=0; //resets flag
        }
    } 
   
}

void __interrupt() high_isr(void){ 
    if(TMR0IF){
        if(count > 0){ //if counting
           count--;    
        }
        else{ //when done set flag and reset timer flag
        flag=1;    //this should happen every second to verify timing
        TMR0=0x0BFA;
        TMR0IF=0;
        }
    }
}
    
    
void __interrupt(low_priority) low_isr(void){ //not used
    INTCONbits.GIEH = 0;
    INTCONbits.GIEH = 1;
}
