/* 
 * File:   bleLink.c
 * Author: williampyke
 *
 * Created on November 23, 2023, 1:06 PM
 */
#include <p18F4620.h>

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

#include <stdio.h>
#include <stdlib.h>

/*
 * 
 */
//const portTICK_PERIOD_MS = 1;                       //since tick rate = 1000Hz, 1 tick per ms
//const TickType_t xDelay = 500 / portTICK_PERIOD_MS; //set delay to 500ms

void USARTsetup(void) // setup BLE module
{
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    RCSTAbits.SPEN = 1; //Serial Port Enabled
    RCSTAbits.CREN = 1; //Continuous Receive Enabled
    RCSTAbits.RX9 = 0;  //Sets 8-bit reception
    TXSTAbits.BRGH = 1; //Sets High Baud Rate mode
    TXSTAbits.SYNC = 0; //Sets EUSART to Asynchronous mode
    TXSTAbits.TXEN = 1; //Transmit Enabled
    TXSTAbits.TX9 = 0;  //Sets 8-bit transmission
    PIE1bits.TXIE = 1;  //Enables tx interrupt
    PIE1bits.RCIE = 1;  //Enables rx interrupt
    SPBRGH = 0x00;          
    SPBRG = 0b00110011;     //51 decimal
    BAUDCON = 0b00000000;
}

void interruptsetup(void)
{
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
} 
    return;
}

void cputime_delay(int cycles)
{
    for (int i = 0; i < cycles; i++)
    {
        Nop();  //nop from C18 manual
    }

}

void bleCharload(char byte)  
{
    TXREG = byte;
    while(!TXIF);  
    while(!TRMT);
}

void bleStringload(char* string)
{
    while(*string)
    bleCharload(*string++);
}

void bleSend()
{
  TXREG = 0b00001101;   //sends carriage return 
  //vTaskDelay(xDelay); //delays for xDelay defined at program start
  cputime_delay(1000);  //delay for 1000cycles
}

void main(void) 
{
    interruptsetup();
    USARTsetup();
    while(1) 
    {
        bleStringload("hello my pulse rate is 981");
        bleSend();        
    }
    return;
}	

