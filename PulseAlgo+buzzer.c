/* 
 * File:   PulseRate_xc8.c
 * Author: Sadie Edwards
 *
 * Created on November 27, 2023, 7:29 PM
 */
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
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic18f4620.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define _XTAL_FREQ 8000000

void ADCconvert();
double BPM ();
int HRadc [150] ; //store adc readings from red sensor
int SPadc [150] ; //store adc readings from IR sensor
int16_t HRVal[75]; //store concatenated high and low adc readings from red sensor
int16_t SPVal[75]; //store concatenated high and low adc readings from ir sensor
int HRadcL = 0, HRadcH = 1; //points to red adc value in array
int SPadcL = 0, SPadcH = 1; //points to IR adc value in array
int peakIndicesR [20]; //store the indices of the highest peaks found in the red adc array
int peakIndicesIR [20]; //store the indices of the highest peaks found in the IR adc array
int peakIndR = 0, peakIndIR = 0, peakValR = 0, peakValIR = 0; //stores the peak value found in signal at the position in the array stored
int baseline = 205; //RED peak baseline (expected points) for 3V (gain 1000/ Offset 1.8V)
int lowIndR = 0, lowIndIR = 0, lowValR = 0, lowValIR = 0; //low index number and value for red and ir 
double BPMout [], SpO2out [];
int RED[], IR[];
int tick = 0, now = 0, num = 0, arrSize1 = 0, arrSize2 = 0, freq = 0, state = 0;
//clock_t now;
// Function to simulate the digital signal status (replace this with your actual signal source)
void __interrupt (high_priority) tcInt (void)
{
    if (PIR1bits.TMR1IF && PIE1bits.TMR1IE)
    {
        PIE1bits.TMR1IE = 0;
        if ((tick - now) >= (freq/0.26) )
        {
            tick = 0;
            if (state == idle)
            {
                state = MEASURING; // changes switch state to measuring mode
            }
        }
        tick++;
    }
    
    if (!LATCbits.LATC1)
    {
        state = REPORTING; //changes switch state to reporting mode
    }
    else if (LATCbits.LATC1)
    {
        state = MEASURING; //changes to measuring mode
    }
    return;
}


void timer1Setup()
{
    T1CON = 0b10110011;
    INTCONbits.GIE = 1;
    IPR1bits.TMR1IP = 1;
    RCONbits.IPEN = 1;
}

void putch(char c) //Empty stub that puts a character to stdout peripheral when using printf() function
{
    while(!TXIF) //Send data to EUSART receive buffer once not empty
        continue;
    TXREG = c; //Parsing the data to be printed to the Read/Write Transmit Buffer register
}

void AD_AN8setup()
{
    TRISBbits.TRISB2 = 1; //Configured as an input to allow analog input
    ADCON0 = 0b00100001; //Analog AN8 Select bits, AD idle, AD On bit
    ADCON1 = 0b00000110; //VREF- source is VSS, VREF+ source is VDD, AN8 set as Analog input
    ADCON2 = 0b10101111; // Right justified, 12 TAD, and FRC
}

void ADC_AN8read()
{
    ADCON0bits.GO = 1 ; //Start conversion of ADC
    while(ADCON0bits.GO);
    HRadc[HRadcL] = ADRESL;
    HRadcL = HRadcL + 2;
    HRadc[HRadcH] = ADRESH;
    HRadcH = HRadcH + 2;
    ADCconvert();
}

void ADCconvert()
{
    static int num = 0;
    num = (num + 1) % (sizeof(HRVal) / sizeof(HRVal[0])); // ensures that if the end of the array is reached it would start back at the top
    HRVal[num] = (HRadc[HRadcH- 2] << 8 ) | HRadc[HRadcL -2];
}

void USARTsetup(void)
{
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    RCSTAbits.SPEN = 1; //Configures RX/DT and TX/CK pins as serial port pins
    RCSTA = 0b10110000; //Enables continuous receive
    TXSTAbits.BRGH = 1; // Selects High baud rate speed
    TXSTAbits.SYNC = 0; //Ensures that Asynchronous EUSART mode is selected
    TXSTAbits.TXEN = 1; //Enables Transmit
    SPBRGH = 0x00; //EUSART baud rate generator register high byte
    SPBRG = 0b00011001; //EUSART baud rate generator register low byte with SPBRG value
    BAUDCON = 0b00000000;
    INTCONbits.PEIE = 1; //Enables all low priority peripheral interrupts
    PIE1bits.RCIE = 1; //Enables the EUSART receive interrupt
    IPR1bits.RCIP = 0; //Sets the EUSART Receive interrupt priority to low
}

void dipSwitch()
{
    TRISD = 0b11110011; //Sets RD7, RD6, RD5, RD4, RD1, and RD0 as inputs
    TRISCbits.TRISC1 = 1; // Sets RC1 to an input
    TRISCbits.TRISC3 = 1; // Sets RC3 to an input
}

void PWMsetup()
{
    TRISCbits.TRISC2 = 0; //configure TRIS bits for output
    T2CON = 0b00000111; //Postscale is 4, Turns on Timer2, Prescaleer is 16
    PWM1CONbits.PRSEN = 1; //automatically restarts PWM.
}

void songPlay()
{
    PR2 = 0b00111011;
    CCPR1L = 0b00111011; //two eight MSbs of the 10-bit value for PWM duty cycle
    CCP1CON = 0b00001100;
    __delay_ms(500);
    PR2 = 0b00110100;
    CCPR1L = 0b00111011; //two eight MSbs of the 10-bit value for PWM duty cycle
    CCP1CON = 0b00111100;
    __delay_ms(500);

}

void MemMang()
{
   static int count = 0;
   if (count >= arrSize1)
   {
       count = 0;
       printf("\r");
       printf("Printing Period: %d\n", freq);
       printf ("----Memory Storage Full-----------\n");
       printf ("----Dumping values to Terminal----\n");
       for (int i = 0; i < arrSize1; i++)
       {
           printf("%d . HR: %d . SpO2: %d\n",num,BPMout [i],SpO2out [i]);
           num++;
           __delay_ms(1);           
       }
       printf("-----Resuming--------\n\n"); 
   }
   BPMout [count] = BPM(); 
   SpO2out [count] = 1;
}

void Init()
{
    freq = 0;
    //if RA0, RA1, RA2 or RA3 change size of array
    if (LATDbits.LATD7)
    {
        double BPMout [20];
        double SpO2out [20];
        printf ("Storage for 20 readings selected\n");
    }
    else if (LATDbits.LATD6)
    {
        double BPMout [30];
        double SpO2out [30];
        printf ("Storage for 30 readings selected\n");
    }
    else if (LATDbits.LATD5)
    {
        double BPMout [40];
        double SpO2out [40];
        printf ("Storage for 40 readings selected\n");
    }
    else if (LATDbits.LATD4)
    {
        double BPMout [50];
        double SpO2out [50];
        printf ("Storage for 50 readings selected\n");
    }

    //if RC1, RD0, or RD1 if on then it changes the frequency of readings for the period of measuring
    if (LATCbits.LATC3){ freq = 20;}
    else if (LATDbits.LATD0){ freq = 30;}
    else if (LATDbits.LATD1){ freq = 40; }

    timer1Setup();
    now = tick;
}

void TroughFind()
{
    //Finding Trough for Red signal
    int n = peakIndicesR [0];
    printf("Enter value: %d", n);
    
    do
    {
        n+=1;
    }while(HRVal[n] > HRVal[n+1]);
    lowIndR = n;
    printf("Exit value: %d", lowIndR);
    lowValR = HRVal[n];    
    
    //Finding Trough for IR signal
    int m = peakIndicesIR [0];
    printf("Enter value: %d", m);
    
    do
    {
        m+=1;
    }while(SPVal[m] > SPVal[m+1]);
    lowIndIR = m;
    printf("Exit value: %d", lowIndIR);
    lowValIR = SPVal[m];
}

void PeakFind()
{
    //Finding Peak for Red signal
    int i = 0;
    for(i = 0; i < sizeof(peakIndicesR)/sizeof(peakIndicesR[0]);i++)
    {
        peakIndicesR[i] = 0;
    }
    i = 0;
    int j = 0;
    for (i=0; i <=(sizeof(HRVal)/sizeof(HRVal[0])); i++)
    {
        if (HRVal[i] > baseline)
        {
            if (peakValR <= HRVal[i])
            {
                peakIndR = i; 
                peakValR = HRVal[peakIndR]; 
            }
        }
        else if (HRVal[i] < baseline && peakIndR != 0)
        {
            peakIndicesR[j] = peakIndR;
            printf("Peaks at: %d\n",peakIndicesR[j]);
            j++;
            peakIndR = 0;
            peakValR = 0;
        }
    }
    if (peakIndR != 0)
    {
      peakIndicesR[j] = peakIndR;
      printf("Peaks at: %d\n",peakIndicesR[j]);
      j++;  
    }
    
    //Finding Peak for IR signal
    i = 0;
    for(i = 0; i < sizeof(peakIndicesIR)/sizeof(peakIndicesIR[0]);i++)
    {
        peakIndicesIR[i] = 0;
    }
    i = 0;
    j = 0;
    for (i=0; i <=(sizeof(SPVal)/sizeof(SPVal[0])); i++)
    {
        if (SPVal[i] > baseline)
        {
            if (peakValIR <= SPVal[i])
            {
                peakIndIR = i; 
                peakValIR = SPVal[peakIndIR]; 
            }
        }
        else if (SPVal[i] < baseline && peakIndIR != 0)
        {
            peakIndicesIR[j] = peakIndIR;
            printf("Peaks at: %d\n",peakIndicesIR[j]);
            j++;
            peakIndIR = 0;
            peakValIR = 0;
        }
    }
    if (peakIndIR != 0)
    {
      peakIndicesIR[j] = peakIndIR;
      printf("Peaks at: %d\n",peakIndicesIR[j]);
      j++;  
    }
}

double BPM ()
{
    int count = 0,sum = 0 ;
    double period = 0, bpm = 0;
    int i = 0;
    /*calculates the difference between the indices to note when each peak occurred
     *the sampling rate is at 20ms so the index should be occurring every 20ms
     *thus multiplying the two finds one pulse
     * For accuracy, an average is taken for three pulses
     * The BPM is found by converting the frequency of the pulse for one beat and multiplying for 60 to get for a minute
    */
    for(i = 1;i<sizeof(peakIndicesIR)/sizeof(peakIndicesIR[0]);i++)
    {
        if(peakIndicesIR[i]==0)
        {
            break;
        }
        else
        {
            sum += peakIndicesIR[i]-peakIndicesIR[i-1];
            count++;
        } 
    }
    
    period = ((sum/count) * 0.02);
    bpm = 60 / period ;
    return bpm;
}

void main(void) {
    USARTsetup();
    AD_AN8setup();
    //timer0setup();
    
    printf("Starting System \n");
    for (int u=0;u<150;u++)
    {
        ADC_AN8read();
        __delay_ms(20);
        
    }
    for (int u=0;u<150;u++)
    {
        printf ("RED ADC output: %d\n",HRVal[u]);     
        printf ("IR ADC output: %d\n",SPVal[u]);
    }
    //while(1)
    
       
    PeakFind(); 
    printf("peakVal: %d\n",peakIndicesIR[0]);
    printf("peakVal: %d\n",peakIndicesIR[1]);
    printf("peakVal: %d\n",peakIndicesIR[2]);
    //printf("Checking for Trough %d\n\n", TroughFind());
    printf("Human Pulse rate: %.2f\n\n",BPM());
    return ;
}

