/*
 * File:   main.c
 * Authors: Amir Ali, Sadie Edwards and William Pyke
 *
 *
 * @brief Finite state machine implemented to manager the pulse oximetry code
 * 
 */

// PIC18F4620 Configuration Bit Settings
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


#include <xc.h>
#include <stdio.h>
#include <math.h>

#define _XTAL_FREQ 8000000
#define MEASURING 0 
#define REPORTING 1 
#define IDLE 2

int state;
int reportDONE = 0, SPO2 = 0, BPM = 0;
int count = 0, arrayvals = 0, adcout = 0, channel = 1;
int HRadc[2] ; //store adc readings from pulse
int16_t RedVal[150];
int SPadc [2] ; //store adc readings from pulse
int16_t IRVal[150];
int peakIndicesR [15]; //store the indices of the highest peaks found in the red adc array
int peakIndicesIR [15]; //store the indices of the highest peaks found in the IR adc array
int peakIndR = 0, peakIndIR = 0,peakValR = 0, peakValIR = 0; //stores the peak value found in signal at the position in the array stored
int baseline = 615; //RED peak baseline (expected points) for 3V (gain 1000/ Offset 1.8V)
int lowIndR = 0, lowIndIR = 0, lowValR = 0, lowValIR = 0; //low index number and value for red and ir 
int BPMout[50], SpO2out[50];
int tick = 0, now = 0, num = 0, arrSize = 0, freq = 0;

void putch(char c) //Empty stub that puts a character to stdout peripheral when using printf() function
{
    while(!TXIF); //Send data to EUSART receive buffer once not empty
    TXREG = c; //Parsing the data to be printed to the Read/Write Transmit Buffer register
}

void __interrupt (high_priority) PeriodInt(void)
{
    if (PIR1bits.TMR1IF && PIE1bits.TMR1IE)
    {
        PIR1bits.TMR1IF = 0;
        if (tick >= (freq/0.26) )
        {
            tick = 0;
            if (state == IDLE)
            {
                state = MEASURING; // changes switch state to measuring mode
            }
        }
        tick++;
    }
    if (LATCbits.LATC1 == 0)
    {
        state = REPORTING; //changes switch state to reporting mode
    }
    return;
}

//void __interrupt (low_priority) tmrint(void)
//{
//    if(PIR2bits.TMR3IF && PIE2bits.TMR3IE)
//    {
//        PIR2bits.TMR3IF = 0;
//        TMR3H = 0b11011001;
//        TMR3L = 0b10010100;
//        count++;
//    }
//}


void timer1Setup()
{
    T1CON = 0b10110001;
    INTCONbits.GIE = 1;
    RCONbits.IPEN = 1;
    IPR1bits.TMR1IP = 1;
    PIE1bits.TMR1IE = 1;
}

void dipSwitchSetup()
{
    TRISD = 0b11110011; //Sets RD7, RD6, RD5, RD4, RD1, and RD0 as inputs
    TRISCbits.TRISC1 = 1; // Sets RC1 to an input
    TRISCbits.TRISC3 = 1; // Sets RC3 to an input
}

void PWMsetup()
{
    TRISCbits.TRISC2 = 0; //configure TRIS bits for output
    T2CON = 0b00000111; //Postscale is 4, Turns on Timer2, Prescaler is 16
    PWM1CONbits.PRSEN = 1; //automatically restarts PWM.
}

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

void setupLEDs()
{
    TRISDbits.TRISD2 = 0;
    LATDbits.LATD2 = 0;
    TRISDbits.TRISD3 = 0;
    LATDbits.LATD3 = 0;
}

//void timer3setup(void)
//{
//    T3CON = 0b00000010;    //set as 8-bit timer with prescaler of 4 and off by default
//    PIE2bits.TMR3IE = 1;
//    IPR2bits.TMR3IP = 0;
//}

void AD_setup()
{
    TRISAbits.TRISA0 = 1; //Configured as an inputs to allow analog input
    TRISAbits.TRISA1 = 1;
    ADCON0 = 0b00000001; //Analog AN0 Select bits, AD idle, AD On bit
    ADCON1 = 0b00000100; //VREF- source is VSS, VREF+ source is VDD, AN0 and AN1 set as Analog inputs
    ADCON2 = 0b10101101; // Left justified, 12 TAD, and FOSC/2
}

void songPlay()
{
    PR2 = 0b10011100;
    CCPR1L = 0b00100111; //two eight MSbs of the 10-bit value for PWM duty cycle
    CCP1CON = 0b00001100;
    __delay_ms(500);
    PR2 = 0b01111100;
    CCPR1L = 0b10001011; //two eight MSbs of the 10-bit value for PWM duty cycle
    CCP1CON = 0b00001100;
    __delay_ms(500);
    PR2 = 0b01111100;
    CCPR1L = 0b10001011; //two eight MSbs of the 10-bit value for PWM duty cycle
    CCP1CON = 0b00001100;
    __delay_ms(500);

}

void MemMang()
{
   static int count1 = 0;
   if (count1 >= arrSize)
   {
       count1 = 0;
       printf("\r");
       printf("Printing Period: %d\n", freq);
       printf ("----Memory Storage Full-----------\n");
       printf ("----Dumping values to Terminal----\n");
       for (int i = 0; i < arrSize; i++)
       {
           printf("%d . HR: %d . SpO2: %d\n",num,BPMout [i],SpO2out [i]);
           num++;
           __delay_ms(1);           
       }
       printf("-----Resuming--------\n\n"); 
   }
   BPMout [count1] = BPM; 
   SpO2out [count1] = SPO2;
   count1++;
}

void TroughFind()
{
    //Finding Trough for Red signal
    int n = peakIndicesR [0];
    do
    {
        n+=1;
    }while(RedVal[n] > RedVal[n+1]);
    lowIndR = n;
    lowValR = RedVal[n];    
    
    //Finding Trough for IR signal
    int m = peakIndicesIR [0];
    do
    {
        m+=1;
    }while(IRVal[m] > IRVal[m+1]);
    lowIndIR = m;
    lowValIR = IRVal[m];
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
    for (i=0; i <=(sizeof(RedVal)/sizeof(RedVal[0])); i++)
    {
        if (RedVal[i] > baseline)
        {
            if (peakValR <= RedVal[i])
            {
                peakIndR = i; 
                peakValR = RedVal[peakIndR]; 
            }
        }
        else if (RedVal[i] < baseline && peakIndR != 0)
        {
            peakIndicesR[j] = peakIndR;
            //printf("Peaks at: %d\n",peakIndicesR[j]);
            j++;
            peakIndR = 0;
            peakValR = 0;
        }
    }
    if (peakIndR != 0)
    {
      peakIndicesR[j] = peakIndR;
      //printf("Peaks at: %d\n",peakIndicesR[j]);
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
    for (i=0; i <=(sizeof(IRVal)/sizeof(IRVal[0])); i++)
    {
        if (IRVal[i] > baseline)
        {
            if (peakValIR <= IRVal[i])
            {
                peakIndIR = i; 
                peakValIR = IRVal[peakIndIR]; 
            }
        }
        else if (IRVal[i] < baseline && peakIndIR != 0)
        {
            peakIndicesIR[j] = peakIndIR;
            //printf("Peaks at: %d\n",peakIndicesIR[j]);
            j++;
            peakIndIR = 0;
            peakValIR = 0;
        }
    }
    if (peakIndIR != 0)
    {
      peakIndicesIR[j] = peakIndIR;
      //printf("Peaks at: %d\n",peakIndicesIR[j]);
      j++;  
    }
}

void BPMcalc()
{
    int count = 0,sum = 0;
    double period = 0;
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
    BPM = 60 / period ;
}

void calculateSPO2()
{
    int redAC = 0, irAC = 0,redPeak = 0, irPeak = 0;
    SPO2 = 0;
    redPeak = RedVal[peakIndicesR[0]];
    irPeak = IRVal[peakIndicesIR[0]];
    redAC = redPeak - lowValR;
    irAC = irPeak - lowValIR;
    SPO2 = (log(redAC)/log(irAC)) * 100; 
}

void csvOutput()
{
    int size = sizeof(BPMout) / sizeof(BPMout[0]);
    for (int i = 0; i < size; ++i) 
    {
      if(BPMout[i] != 0) 
      {
        printf("%d", i+1);
        printf(",");
        printf("%d", BPMout[i]);
        printf(",");
        printf("%d", SpO2out[i]);
        printf("\n");
      }
      else
      {
        break;
      }
    }
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

void CompareThresh()
{
    if((SPO2 < 70)||(BPM < 30)||(BPM > 200))
    {
        songPlay();
    }
}

void ADCconvert(int channel, int I)
{
    if(channel == 1)
    {
        RedVal[I] = (HRadc[1] << 8 ) | HRadc [0];
    }
    else if (channel == 2)
    {
        IRVal[I] = (SPadc[1] << 8 ) | SPadc [0]; 
    }
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

void realtimeSend()
{
    CompareThresh();
    MemMang();
    printf("HR: %d SPO2: %d       \r",BPM, SPO2);
}


void lightPulse(void)
{
//    TMR3H = 0b11011001;
//    TMR3L = 0b10010100;
//    T3CONbits.TMR3ON = 1;
//    for (int j = 0; j < 100; j++)
//    {
//        IRoff();
//        REDon();
//        __delay_ms(10);
//        IRon();
//        REDoff();
//        __delay_ms(10);
//    }
    for (int i =0; i < 150; i++)
    {
        //IRoff();
        //REDon();
        adcselect(1);
        ADC_read(1,i);
        //while(count < 2);   //runs until count = 2
        __delay_ms(9);
        //REDoff();           
        //IRon();
        adcselect(2);
        ADC_read(2,i);
        //while(count < 4);   //runs until count = 4
        __delay_ms(9);
        //count = 0;
    }  
    //IRoff();
    //T3CONbits.TMR3ON = 0;
}


void main(void) 
{
    USARTsetup();
    dipSwitchSetup();
    setupLEDs();
    PWMsetup();
    AD_setup();
    freq = 0;
    //if RA0, RA1, RA2 or RA3 change size of array
    if (LATDbits.LATD7 == 1)
    {
        arrSize = 20;
        printf ("Storage for 20 readings selected\n");
    }
    else if (LATDbits.LATD6 == 1)
    {
        arrSize = 30;
        printf ("Storage for 30 readings selected\n");
    }
    else if (LATDbits.LATD5 == 1)
    {
        arrSize = 40;
        printf ("Storage for 40 readings selected\n");
    }
    else if (LATDbits.LATD4 == 1)
    {
        arrSize = 50;
        printf ("Storage for 50 readings selected\n");
    }
    
    //if RC1, RD0, or RD1 if on then it changes the frequency of readings for the period of measuring
    if (LATCbits.LATC3 == 1){ freq = 10;printf ("Time is 10s\n");}
    else if (LAtDbits.LATD0 == 1){ freq = 20;printf ("Time is 20s\n");}
    else if (LATDbits.LATD1 == 1){ freq = 30;printf ("Time is 30s\n");}
    timer1Setup();
    //timer3setup();
    while(1)
    {
        switch(state) 
        { 
            case MEASURING:
                lightPulse();
                PeakFind();
                TroughFind();
                BPMcalc();
                calculateSPO2();
                realtimeSend();
                state = IDLE;
            break;
            case REPORTING:
                if(reportDONE == 0)
                {
                    csvOutput();
                    reportDONE = 1;
                }     
                if (LATCbits.LATC1 == 1)
                {
                    state = MEASURING; //changes to measuring mode
                    reportDONE = 0;
                }
            break;
            case IDLE:
                while(state == IDLE);
            break;
        }
    }
    return;
}

