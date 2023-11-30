/* 
 * File:   lightPulse.c
 * Author: williampyke
 *
 * Created on November 23, 2023, 1:06 PM
 */
#include <pic18f4620.h>

// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#define _XTAL_FREQ 8000000

unsigned int state;
int count = 0, redON = 0, redOFF = 1, irON = 0, irOFF = 1, waitON = 0, waitOFF = 1, arrayvals = 0, adcout = 0, channel = 1;
char arrsel;
int HRadc[2] ; //store adc readings from pulse
int16_t HRVal[150];
int HRadcL = 0, HRadcH = 1;
int SPadc [2] ; //store adc readings from pulse
int16_t SPVal[150];
int SPadcL = 0, SPadcH = 1;

void USARTsetup(void) // setup BLE module
{
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    RCSTAbits.SPEN = 1; //Serial Port Enabled
    TXSTAbits.BRGH = 1; //Sets High Baud Rate mode
    TXSTAbits.SYNC = 0; //Sets EUSART to Asynchronous mode
    TXSTAbits.TXEN = 1; //Transmit Enabled
    TXSTAbits.TX9 = 0;  //Sets 8-bit transmission
    SPBRGH = 0x00;          
    SPBRG = 0b00110011;     //51 decimal
    BAUDCON = 0b00000000;
}


void csvOutput()
{
    int size = sizeof(HRarray) / sizeof(HRarray[0]);
    for (int i = 0; i < size; ++i) 
    {
      if(HRarray[i] != 0) 
      {
        printf("%d", i+1);
        printf(",");
        printf("%d", HRarray[i]);
        printf(",");
        printf("%d", SPO2array[i]);
        printf("\n");
      }
      else
      {
        break;
      }
    }
}

void realtimeSend()
{
    compare();
    MemMang();
    printf("\rHR: %d SPO2: %d       ",HRvalue, SPO2value);
}

void putch(char c)
{
    while(!TXIF)
        continue;
    TXREG = c;
}

void REDon(void)
{
    LATDbits.LATD2 = 1;
} 
  
void REDoff(void)
{
    LATDbits.LATD2 = 0;
}

void IRon(void)
{
    LATDbits.LATD3 = 1;
} 
  
void IRoff(void)
{
    LATDbits.LATD3 = 0;
}

void adcselect(int channel)
{
    if(channel == 1)    //AN0 is RED LED channel
    {
        ADCON0bits.CHS3 = 0;
        ADCON0bits.CHS2 = 0;
        ADCON0bits.CHS1 = 0;
        ADCON0bits.CHS0 = 0;
    }
    else if(channel == 2)   //AN1 is IR LED channel
    {
        ADCON0bits.CHS3 = 0;
        ADCON0bits.CHS2 = 0;
        ADCON0bits.CHS1 = 0;
        ADCON0bits.CHS0 = 1;        
    }
}
void setupLEDs(void)
{
    TRISDbits.TRISD2 = 0;
    LATDbits.LATD2 = 0;
    TRISDbits.TRISD3 = 0;
    LATDbits.LATD3 = 0;
}

void timer0setup(void)
{
    T0CON = 0b010000001;    //set as 8-bit timer with prescaler of 4 and off by default
    INTCONbits.GIE = 1;
    INTCONbits.TMR0IE = 1;
}

void AD_setup()
{
    TRISAbits.TRISA0 = 1; //Configured as an inputs to allow analog input
    TRISAbits.TRISA1 = 1;
    ADCON0 = 0b00000001; //Analog AN0 Select bits, AD idle, AD On bit
    ADCON1 = 0b00000100; //VREF- source is VSS, VREF+ source is VDD, AN0 and AN1 set as Analog inputs
    ADCON2 = 0b10101101; // Left justified, 12 TAD, and FOSC/2
}
void ADC_read(int channel, int I)
{
    ADCON0bits.GO = 1 ; //Start conversion of ADC
    while(ADCON0bits.GO);
    if(channel == 1)
    {
        HRadc[0] = ADRESL;
        HRadc[1] = ADRESH;
    }
    else if (channel == 2)
    {
        SPadc[0] = ADRESL;
        SPadc[1] = ADRESH;
    }
    ADCconvert(channel,I);
}

void compare()
{
    if((SPO2 < 70)||(HR < 30)||(HR > 200))
    {
        soundPlay();
    }
}

void ADCconvert(int channel, int I)
{
    if(channel == 1)
    {
    HRVal[I] = (HRadc[1] << 8 ) | HRadc [0];
    //printf ("HR ADC output: %d\n\n", HRVal[num]); 
    }
    else if (channel == 2)
    {
    SPVal[I] = (SPadc[1] << 8 ) | SPadc [0];
    //printf ("SP ADC output: %d\n\n", SPVal[num]); 
    }
}

void __interrupt(low_priority) tmrint(void)
{
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        INTCONbits.TMR0IF = 0;
        count++;
    }
    return;
}

void lightPulse(void)
{
    T0CONbits.TMR0ON = 1;
    for (int i =0; i < 150; i++)
    {
        IRoff();
        REDon();
        //printf("RED ON");
        adcselect(1);
        ADC_read(1);
        while(count < 2);   //runs until count = 2
        REDoff();
        //printf("RED OFF");           
        IRon();
        //printf("IR ON");
        adcselect(2);
        ADC_read(2);
        while(count < 4);   //runs until count = 4
        //printf("IR OFF");
        count = 0;
    }  
    T0CONbits.TMR0ON = 0;
}

void main(void) 
{
    timer0setup();
    setupLEDs();
    USARTsetup();
    AD_setup();
    while(1)
    {
        
    }
    
return;
}	

