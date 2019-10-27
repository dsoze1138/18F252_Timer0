/*
 * Possible solution for homework in post:
 * https://www.microchip.com/forums/FindPost/1116654
 * 
 * Find full project in repository:
 * https://github.com/dsoze1138/18F252_Timer0
 * 
 * File     : main.c
 * Author   : dan1138
 * Compiler : XC8 v2.05
 * IDE      : MPLABX v5.25
 *
 * Target: PIC18F252
 *
 *                             PIC18F252
 *                   +------------:_:------------+
 *         VPP ->  1 : RE3/MCLR/VPP      PGD/RB7 : 28 <> PGD  LCD_D7
 *             ->  2 : RA0/AN0           PGC/RB6 : 27 <> PGC  LCD_D6
 *             <>  3 : RA1/AN1           PGM/RB5 : 26 <>      LCD_D5
 *             <>  4 : RA2/AN2               RB4 : 25 <>      LCD_D4
 *             <>  5 : RA3/AN3          CC22/RB3 : 24 <>      LCD_D3
 *         LED <>  6 : RA4/C1OUT             RB2 : 23 <>      LCD_D2
 *             <>  7 : RA5/AN4               RB1 : 22 <>      LCD_D1
 *         GND <>  8 : VSS                   RB0 : 21 <>      LCD_D0
 *       10MHz <>  9 : RA7/OSC1              VDD : 20 <- 5v0
 *       10MHz <> 10 : RA6/OSC2              VSS : 19 <- GND
 *   32.768KHz <> 11 : RC0/SOSCO       RX/DT/RC7 : 18 <>      LCD_E
 *   32.768KHz <> 12 : RC1/SOSCI       TX/CK/RC6 : 17 <>      LCD_RS
 *             <> 13 : RC2/CCP1     SPI_MOSI/RC5 : 16 <>      LCD_RW
 *             <> 14 : RC3/SPI_CLK  SPI_MISO/RC4 : 15 <>
 *                   +---------------------------:
 *                              DIP-28
 *
 *   LCD Module        PIC
 *   MC21605C6W-SPR  PIN GPIO
 *   [ 1]GND         [19]GND
 *   [ 2]PWR         [20]PWR
 *   [ 3]CONTRAST    [ 8]GND
 *   [ 4]LCD_RS      [ 5]RC6
 *   [ 5]LCD_RW      [ 6]RC5
 *   [ 6]LCD_E       [ 7]RC7
 *   [ 7]LCD_D0      [15]RB0
 *   [ 8]LCD_D1      [16]RB1
 *   [ 9]LCD_D2      [17]RB2
 *   [10]LCD_D3      [18]RB3
 *   [11]LCD_D4      [15]RB4
 *   [12]LCD_D5      [16]RB5
 *   [13]LCD_D6      [17]RB6
 *   [14]LCD_D7      [18]RB7
 *
 * Description:
 *  Initial a PIC18F252 with an external 10MHz crystal for a 40MHz system clock.
 *  Setup a TIMER0 interrupt to assert every 1.6384 milliseconds.
 *  Assign pins to be used for an 8-bit parallel interface to an HD44780 LCD controller.
 *
 */
#pragma config OSC = HSPLL      /* HS oscillator with PLL enabled/Clock frequency = (4 x FOSC) */
#pragma config OSCS = OFF       /* Oscillator system clock switch option is disabled (main oscillator is source) */
#pragma config PWRT = OFF       /* Power-up Timer Enable bit (PWRT disabled) */
#pragma config BOR = OFF        /* Brown-out Reset Enable bit (Brown-out Reset disabled) */
#pragma config BORV = 20        /* Brown-out Reset Voltage bits (VBOR set to 2.0V) */
#pragma config WDT = OFF        /* Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit)) */
#pragma config WDTPS = 128      /* Watchdog Timer Postscale Select bits (1:128) */
#pragma config CCP2MUX = OFF    /* CCP2 Mux bit (CCP2 input/output is multiplexed with RB3) */
#pragma config STVR = ON        /* Stack Full/Underflow Reset Enable bit (Stack Full/Underflow will cause RESET) */
#pragma config LVP = OFF        /* Low Voltage ICSP Enable bit (Low Voltage ICSP disabled) */
#pragma config CPB = OFF        /* Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code protected) */
#pragma config CPD = OFF        /* Data EEPROM Code Protection bit (Data EEPROM not code protected) */
#pragma config WRTC = OFF       /* Configuration Register Write Protection  */
#pragma config WRTB = OFF       /* Boot Block Write Protection  */
#pragma config WRTD = OFF       /* Data EEPROM Write Protection */
#pragma config EBTRB = OFF      /* Boot Block Table Read Protection */
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
/*
 *
 */
#include <xc.h>
#include <stddef.h>
#include <stdint.h>
/*
 * System oscillator frequency
 */
#define _XTAL_FREQ (40000000ul)
/*
 * Initialize this PIC
 */
void PIC_Init(void) {
    INTCON = 0;     /* disable interrupts */
    INTCON2 = 0xF5;
    INTCON3 = 0xC0;
    PIE1 = 0;
    PIE2 = 0;

    RCONbits.IPEN = 0;  /* use legacy interrupt model */

    ADCON1 = 0x0F;      /* configure all ADC inputs for digital I/O */

    LATA   = 0x00;
    TRISA  = 0xEF;      /* RA4 output for LED */
    LATB   = 0x00;
    TRISB  = 0x00;      /* RB0-7 outputs to HD44780 controller */
    LATC   = 0x00;
    TRISC  = 0x1F;      /* RC5,RC6,RC7 outputs to HD44780 controller */
}
/*
 * Setup TIMER0 to assert an interrupt every 16384 instruction cycles
 */
 uint8_t gTIMER0_TickCount;
void TIMER0_Init(void)
{
    INTCONbits.TMR0IE = 0;
    gTIMER0_TickCount = 0;
    T0CON = 0b11000101;     /* TMR0 clock edge low to high, TMR0 clock = FCY, TMR0 prescale 1:64        */
    TMR0 = 0;               /* TIMER0 will assert the overflow flag every 256*64 (16384)                */
    INTCONbits.TMR0IF = 0;  /* instruction cycles, with a 40MHz oscillator this is 1.6384 milliseconds. */
    INTCONbits.TMR0IE = 1;
}
/*
 * Interrupt handlers
 */
void __interrupt(high_priority) ISR_Handler(void)
{
    /* Handle system tick */
    if (INTCONbits.TMR0IE)
    {
        if(INTCONbits.TMR0IF)
        {
            INTCONbits.TMR0IF = 0;
            gTIMER0_TickCount++;
        }
    }
}
/*
 * Application
 */
void main(void)
{
    uint8_t  T0_TickSample;
    uint8_t  T0_TickDelta;

    PIC_Init();
    TIMER0_Init();

    T0_TickSample = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE  = 1;
    /*
     * Application process loop
     */
    for(;;)
    {
        T0_TickDelta = gTIMER0_TickCount - T0_TickSample;
        if(T0_TickDelta >= 10)
        {
            T0_TickSample = T0_TickSample + T0_TickDelta;
            /* at least 10 TIMER0 (1.6384ms) interrupts occurred */
            LATAbits.LATA4 ^= 1; /* Toggle LED on RA4 */
        }
    }
}
