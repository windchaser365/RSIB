
#include "config.h"

#include <plib/timers.h>
#include <xc.h>

#pragma config STVREN = OFF //Stack Overflow/Underflow Reset 
#pragma config XINST = OFF //Extended Instruction Set
//#pragma config CCP2MX = OFF
#pragma config OSC = INTOSCPLL // INTOSC
#pragma config WDTEN = OFF  // wdt off
#pragma config IOL1WAY = OFF // works either way
#pragma config CP0 = OFF // code protect
//#pragma config BOR = OFF
//#pragma config MCLRE = OFF
//#pragma config PWRT   = OFF

#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)

void config(void)
{
	// set internal oscillator at 8 MHz
	OSCCONbits.IRCF2 = 1;
	OSCCONbits.IRCF1 = 1;
	OSCCONbits.IRCF0 = 1;
	// PLL 4x
	OSCTUNEbits.PLLEN = 1; // boost oscillator to 32MHz
    
    __delay_ms(100);
    __delay_ms(100);
    __delay_ms(100);
    __delay_ms(100);
    __delay_ms(100);

    // disable all interrupts
	INTCONbits.GIEH = 0;     // enable all high priority interrupts
    INTCONbits.GIEL = 0;     // enable all low priority interrupts
    RCONbits.IPEN = 1; // enable interrupt priorities

	// setup I/O pins
    TRISAbits.TRISA0 = DIN; // test port
    TRISAbits.TRISA1 = DIN; // AN0 SONAR level A/D converter
    TRISAbits.TRISA2 = DIN; // AN1 BATT voltage level
	TRISAbits.TRISA3 = DOUT; // heartbeat out
	TRISAbits.TRISA5 = DIN; // SS1
	TRISAbits.TRISA6 = DIN; // unused
    TRISCbits.TRISC0 = DIN; // unused
    TRISCbits.TRISC1 = DIN; // unused
    TRISCbits.TRISC2 = DIN; // BATT voltage level
	TRISCbits.TRISC3 = DIN; // SCK1
	TRISCbits.TRISC4 = DIN; // SDA1
	TRISCbits.TRISC5 = DIN; // unused
    TRISCbits.TRISC6 = DIN; // unused
    TRISCbits.TRISC7 = DIN; // unused
    
	// unused pins TODO add missing ones
	TRISBbits.TRISB0 = DIN; // INT0 (defaults to analog input!)
    TRISBbits.TRISB1 = DIN; // INT1 (defaults to analog input!)
    TRISBbits.TRISB2 = DIN; // INT2 (defaults to analog input!)
    TRISBbits.TRISB3 = DIN; // INT3 (defaults to analog input!)
    TRISBbits.TRISB4 = DIN; // canale 5
	TRISBbits.TRISB5 = DIN; // defaults to digital input
	TRISBbits.TRISB6 = DIN; // defaults to digital input
	TRISBbits.TRISB7 = DIN; // defaults to digital input
    
    ANCON0bits.PCFG0 = ANALOG; // CH0 SONAR A/D
    ANCON0bits.PCFG1 = ANALOG; // CH1 BATT A/D
    ANCON0bits.PCFG2 = ANALOG; // A/D (free)
    ANCON0bits.PCFG3 = DIGITAL; // AN3 as digital
    ANCON0bits.PCFG4 = DIGITAL; // AN4 (SS1) as digital input
    ANCON1bits.PCFG8 = DIGITAL; // 
    ANCON1bits.PCFG9 = DIGITAL; // 
    ANCON1bits.PCFG10 = DIGITAL; // 
    ANCON1bits.PCFG11 = ANALOG; // A/D (free)
    ANCON1bits.PCFG12 = DIGITAL; // 
    
    // setup A/D converter
    ADCON0bits.VCFG1 = 0; // Vss is the voltage - reference
    ADCON0bits.VCFG0 = 0; // Vdd is the voltage + reference
    ADCON1bits.ADFM = 1; // right-justified
    ADCON1bits.ADCAL = 0; // Normal A/D Converter operation
    ADCON1bits.ADCS = 5; // Fosc/16 = 2MHz => T=0.5us
    ADCON1bits.ACQT = 3; // 6T=6*0.5=3us acquisition time (safe)
    IPR1bits.ADIP = 0; // low priority on convertion completion interrupt
    PIE1bits.ADIE = 1; // enable A/D conversion completion interrupt
    ADCON0bits.ADON = 1; // A/D module enabled
    
    // configure PPS ports
    RPINR1 = 4; // map INT1 to RP4
    RPINR2 = 5; // map INT2 to RP5
    RPINR3 = 6; // map INT3 to RP6

	RCONbits.IPEN = 1; // Interrupt priority levels enabled (needed for MSSP interrupt)
	
    // now setup I2C
    SSP1CON1bits.SSPEN = 0; // master synchronous serial port (SPI) disabled (must be set at end of config)
	SSP1STATbits.CKE = 0; // disable SMBus during config 
    SSP1CON2bits.SEN = 0; // disable start condition during config
	SSP1STATbits.SMP = 1; // slew rate control enabled (high speed)
	SSP1CON2bits.GCEN = 0; // General call address 0000h disabled

	SSP1ADD = PWM_READER_I2C_BASE_ADDR; // slave base address
    SSP1CON1bits.SSPM = 0b1001; // Load SSPxMSK register at SSPxADD SFR address
    SSP1MSK = I2C_MASK; // 7-bit slave address mask
    SSP1CON1bits.SSPM = 0b0110; // I2C Slave mode, 7-bit address without Start and Stop bit interrupts enabled;	
    
	SSP1STATbits.CKE = 1; // enable SMBus
	SSP1CON1bits.SSPEN = 1; // master synchronous serial port enabled
    SSP1CON1bits.CKP = 1;  // Clock release
    SSP1CON2bits.SEN = 1; // start condition enable (automatic clock hold feature)
	
	// setup MSSP interrupt
    
    PIE1bits.SSP1IE = 0; // disable interrupt from MSSP during config
    IPR1bits.SSP1IP = 0; // MSSP Interrupt Priority set to low
    PIR1bits.SSP1IF = 0; // Clear interrupt status to avoid automatic interrupt ad "boot time"
    
    // setup port-change interrupts and edge transition interrupts for PWM readings
    INTCONbits.RBIE = 1;    // enable port change interrupt (RB4-7))
    INTCON2bits.RBPU = 1;   // disable pull-up to PORT B
    INTCON3bits.INT1IP = 1; // high priority for INT1
    INTCON3bits.INT2IP = 1; // high priority for INT2
    INTCON2bits.INT3IP = 1; // high priority for INT3
    INTCON2bits.RBIP = 1;   // high priority for RB IOC
    INTCONbits.INT0IE = 1;  // enable external interrupt 0
    INTCON3bits.INT1IE = 1; // enable external interrupt 1
    INTCON3bits.INT2IE = 1; // enable external interrupt 2
    INTCON3bits.INT3IE = 1; // enable external interrupt 3
    INTCON2bits.INTEDG0 = 1; // external interrupt 0 on rising edge
    INTCON2bits.INTEDG1 = 1; // external interrupt 1 on rising edge
    INTCON2bits.INTEDG2 = 1; // external interrupt 2 on rising edge
    INTCON2bits.INTEDG3 = 1; // external interrupt 3 on rising edge
    INTCONbits.INT0IF = 0;   // clear external INT0 flag (ready for ext interrupt)
    INTCON3bits.INT1IF = 0;  // clear external INT1 flag (ready for ext interrupt)
    INTCON3bits.INT2IF = 0;  // clear external INT2 flag (ready for ext interrupt)
    INTCON3bits.INT3IF = 0;  // clear external INT3 flag (ready for ext interrupt)
        
    // setup TIMER 0 as PWM measure reference
    T0CONbits.TMR0ON = 1; // enable TIMER 0
    T0CONbits.T08BIT = 0; // 16 bits
    T0CONbits.T0CS = 0; // internal source
    T0CONbits.T0SE = 0; // increment at low-to-high transition
    T0CONbits.PSA = 0; // prescaler assigned
    T0CONbits.T0PS = 5; // 1/64 prescaler ==> (8MHz/4=2MHz, 2MHz/64=XXXXXHz, XXXX/65535=yyyys period)
    
    // setup TIMER 1 for analog acquisition start (BATT, SONAR)
    T1CONbits.TMR1CS = 0; // timer clock source is instruction clock (Tosc/4 = 2MHz)
    T1CONbits.T1CKPS = 0; // no prescale
    T1CONbits.T1OSCEN = 0; // disable crystal oscillator circuit
    T1CONbits.T1SYNC = 1; // disable external clock sync
    T1CONbits.RD16 = 1; // 16-bit register read/write
    
    // setup Timer 1 interrupt
    PIE1bits.TMR1IE = 0; // disable timer 1 overflow interrupt during config
    IPR1bits.TMR1IP = 0; // low priority to timer 0 (for A/D acquisition)
    
    // enable peripheral interrupts
    INTCONbits.PEIE = 1;
    
    // enable all interrupts
	INTCONbits.GIEH = 1;     // enable all high priority interrupts
    INTCONbits.GIEL = 1;     // enable all low priority interrupts
    
    __delay_ms(100);
    
    PIE1bits.TMR1IE = 1; // enable timer 1 overflow interrupt for A/D acquisition    
    T1CONbits.TMR1ON = 1; // timer 1 start for PWM acquisition

    __delay_ms(100);

    PIE1bits.SSP1IE = 1; // enable interrupt from I2C
}