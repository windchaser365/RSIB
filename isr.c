#include "isr.h"
#include "config.h"
#include "types.h"
#include <xc.h> 
#include <stdint.h>

#define TMR_COUNT (TMR0L | (TMR0H << 8))

typedef enum
{
    PWM1 = 0,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6,
    _PWM_COUNT
} PwmReaderRegisterValueEnum;

typedef enum
{
    PWM1_HI_BYTE_IDX = 0,
    PWM1_LO_BYTE_IDX,
    PWM2_HI_BYTE_IDX,
    PWM2_LO_BYTE_IDX,
    PWM3_HI_BYTE_IDX,
    PWM3_LO_BYTE_IDX,
    PWM4_HI_BYTE_IDX,
    PWM4_LO_BYTE_IDX,
    PWM5_HI_BYTE_IDX,
    PWM5_LO_BYTE_IDX,
    PWM6_HI_BYTE_IDX,
    PWM6_LO_BYTE_IDX,
    _PWM_BYTE_COUNT
} PwmReaderByteIndexEnum;

typedef union
{
    UINT16 values[_PWM_COUNT];
    UINT8 bytes[2 * _PWM_COUNT];
} PwmReaderDataType;

//volatile UINT16 counter = 0u;
static volatile PwmReaderDataType pwmValues; // cioè i valori dei registri
acquisitionStatusType acquisitionStatus;

void interrupt high_priority highISR (void)           // interrupt function 
{   
    static UINT16 pwm1RisingEdgeCounter, pwm1FallingEdgeCounter;
    static UINT16 pwm2RisingEdgeCounter, pwm2FallingEdgeCounter;
    static UINT16 pwm3RisingEdgeCounter, pwm3FallingEdgeCounter;
    static UINT16 pwm4RisingEdgeCounter, pwm4FallingEdgeCounter;
    static UINT16 pwm5RisingEdgeCounter, pwm5FallingEdgeCounter;
    static UINT16 pwm6RisingEdgeCounter, pwm6FallingEdgeCounter;
    
    // IOC changed pin detection
    static UINT8 rbPrev = 0u;  // previous RB (pin 4-7) pin values
    static UINT8 rbCurr;       // current RB (pin 4-7) pin values
    static UINT8 rbChanged;    // changed RB (pin 4-7) pins

    if (INTCONbits.INT0IE && INTCONbits.INT0IF) 
    {   
        if (INTCON2bits.INTEDG0) // INT0 at rising edge => rising edge hit
        {
            pwm1RisingEdgeCounter = TMR_COUNT;
            INTCON2bits.INTEDG0 = 0;          // next interrupt on falling edge
        }
        else
        {
            pwm1FallingEdgeCounter = TMR_COUNT;
            
            if (pwm1FallingEdgeCounter >= pwm1RisingEdgeCounter)
            {
                pwmValues.values[PWM1] = pwm1FallingEdgeCounter - pwm1RisingEdgeCounter;
            }
            else
            {
                pwmValues.values[PWM1] = UINT8_MAX - pwm1RisingEdgeCounter + pwm1FallingEdgeCounter;
            }

            INTCON2bits.INTEDG0 = 1;          // next interrupt on rising edge
        }
        
        INTCONbits.INT0IF = 0;            // clear the interrupt flag 
    }
    else if (INTCON3bits.INT1IE && INTCON3bits.INT1IF) 
    {   
        if (INTCON2bits.INTEDG1) // INT1 at rising edge => rising edge hit
        {
            pwm2RisingEdgeCounter = TMR_COUNT;
            INTCON2bits.INTEDG1 = 0;          // next interrupt on falling edge
        }
        else
        {
            pwm2FallingEdgeCounter = TMR_COUNT;
            
            if (pwm2FallingEdgeCounter >= pwm2RisingEdgeCounter)
            {
                pwmValues.values[PWM2] = pwm2FallingEdgeCounter - pwm2RisingEdgeCounter;
            }
            else
            {
                pwmValues.values[PWM2] = UINT8_MAX - pwm2RisingEdgeCounter + pwm2FallingEdgeCounter;
            }

            INTCON2bits.INTEDG1 = 1;          // next interrupt on rising edge
        }
        
        INTCON3bits.INT1IF = 0;            // clear the interrupt flag 
    }
    else if (INTCON3bits.INT2IE && INTCON3bits.INT2IF) 
    {   
        if (INTCON2bits.INTEDG2) // INT2 at rising edge => rising edge hit
        {
            pwm3RisingEdgeCounter = TMR_COUNT;
            INTCON2bits.INTEDG2 = 0;          // next interrupt on falling edge
        }
        else
        {
            pwm3FallingEdgeCounter = TMR_COUNT;
            
            if (pwm3FallingEdgeCounter >= pwm3RisingEdgeCounter)
            {
                pwmValues.values[PWM3] = pwm3FallingEdgeCounter - pwm3RisingEdgeCounter;
            }
            else
            {
                pwmValues.values[PWM3] = UINT8_MAX - pwm3RisingEdgeCounter + pwm3FallingEdgeCounter;
            }

            INTCON2bits.INTEDG2 = 1;          // next interrupt on rising edge
        }
        
        INTCON3bits.INT2IF = 0;            // clear the interrupt flag 
    }
    else if (INTCON3bits.INT3IE && INTCON3bits.INT3IF) 
    {   
        if (INTCON2bits.INTEDG3) // INT3 at rising edge => rising edge hit
        {
            pwm4RisingEdgeCounter = TMR_COUNT;
            INTCON2bits.INTEDG3 = 0;          // next interrupt on falling edge
        }
        else
        {
            pwm4FallingEdgeCounter = TMR_COUNT;
            
            if (pwm4FallingEdgeCounter >= pwm4RisingEdgeCounter)
            {
                pwmValues.values[PWM4] = pwm4FallingEdgeCounter - pwm4RisingEdgeCounter;
            }
            else
            {
                pwmValues.values[PWM4] = UINT8_MAX - pwm4RisingEdgeCounter + pwm4FallingEdgeCounter;
            }

            INTCON2bits.INTEDG3 = 1;          // next interrupt on rising edge
        }
        
        INTCON3bits.INT3IF = 0;            // clear the interrupt flag 
    }   
    if (INTCONbits.RBIF)
    {
        rbCurr = PORTB & 0xF0;  // take only pin 4-7 into account
        rbChanged = rbCurr ^ rbPrev;
        
        if (rbChanged & 0x40) // if pin RB6 changed
        {
            if (PORTBbits.RB6) { // rising edge PB4 hit
                pwm5RisingEdgeCounter = TMR_COUNT;
            } 
            else 
            {
                pwm5FallingEdgeCounter = TMR_COUNT;
                
                if (pwm5FallingEdgeCounter >= pwm5RisingEdgeCounter)
                {
                    pwmValues.values[PWM5] = pwm5FallingEdgeCounter - pwm5RisingEdgeCounter;
                }
                else
                {
                    pwmValues.values[PWM5] = UINT8_MAX - pwm5RisingEdgeCounter + pwm5FallingEdgeCounter;
                }
            }
        }
        else if (rbChanged & 0x80) // if pin RB7 changed
        {
            if (PORTBbits.RB7) { // rising edge PB5 hit
                pwm6RisingEdgeCounter = TMR_COUNT;
            } 
            else 
            {
                pwm6FallingEdgeCounter = TMR_COUNT;
                
                if (pwm6FallingEdgeCounter >= pwm6RisingEdgeCounter)
                {
                    pwmValues.values[PWM6] = pwm6FallingEdgeCounter - pwm6RisingEdgeCounter;
                }
                else
                {
                    pwmValues.values[PWM6] = UINT8_MAX - pwm6RisingEdgeCounter + pwm6FallingEdgeCounter;
                }
            }
        }
        
        rbPrev = rbCurr;
        INTCONbits.RBIF = 0;
    }   
}

void interrupt low_priority lowISR (void)        // interrupt function 
{
    UINT8 incomingByte; 
    static UINT8 requestedI2cAddress; // 
    static UINT8 curr_i2c_offset = 0; // indirizzo corrente autoincrementante per lettura
    static UINT8 battLevel[2]; // low byte, high byte
    static UINT8 sonarLevel[2]; // low byte, high byte
    
    /**
     * @brief A/D acquisition COUNTER (incremented at every acquisition)
     *
     * at every 10 SONAR acquisitions, 1 A/D acquisition is performed
     */
    static UINT8 adAcquisitionCounter = 0; // contains the following info:

    if (PIE1bits.SSP1IE && PIR1bits.SSP1IF)
    {
        incomingByte = SSP1BUF; // read buffer (automatically cleared)

        if (!SSP1STATbits.D_NOT_A) 
        {
            if (SSPSTATbits.R_NOT_W) 
            {                
                SSPCON1bits.WCOL = 0; // reset collision

                requestedI2cAddress = (incomingByte >> 1); // last bit for R/W

                if (requestedI2cAddress == PWM_READER_I2C_ADDR)                    
                {
                    curr_i2c_offset = 0;
                    SSP1BUF = pwmValues.bytes[curr_i2c_offset++];
                }
                else if (requestedI2cAddress == BATT_READER_I2C_ADDR)                     
                {
                    curr_i2c_offset = 0;
                    SSP1BUF = battLevel[curr_i2c_offset++];
                }
                else if (requestedI2cAddress == SONAR_READER_I2C_ADDR)                     
                {
                    curr_i2c_offset = 0;
                    SSP1BUF = sonarLevel[curr_i2c_offset++];
                }
            }
        } 
        else // richiesto dato (dopo l'indirizzo)
        {
            if (SSPSTATbits.R_NOT_W) 
            {
                SSPCON1bits.WCOL = 0;
                
                if (requestedI2cAddress == PWM_READER_I2C_ADDR)                    
                {
                    SSP1BUF = pwmValues.bytes[curr_i2c_offset++];
                    curr_i2c_offset %= _PWM_BYTE_COUNT;
                }
                else if (requestedI2cAddress == BATT_READER_I2C_ADDR)                     
                {
                    SSP1BUF = battLevel[curr_i2c_offset++];
                    curr_i2c_offset %= 2;
                }
                else if (requestedI2cAddress == SONAR_READER_I2C_ADDR)                     
                {
                    SSP1BUF = sonarLevel[curr_i2c_offset++];
                    curr_i2c_offset %= 2;
                }
            }   
        }
                        
        PIR1bits.SSP1IF = 0;   // ready for next interrupt
        SSP1CON1bits.CKP = 1;  // Clock release
    }
    else if (PIE1bits.TMR1IE && PIR1bits.TMR1IF)
    {
        adAcquisitionCounter++;
        
        // in any acquisition is in progress, exit
        if (acquisitionStatus.value != 0u) {
            PIR1bits.TMR1IF = 0;
            return;
        }
            
        if ((adAcquisitionCounter % 10) == 0) 
        {
            // start A/D conversion for battery level if done
            if (!ADCON0bits.NOT_DONE) {
                ADCON0bits.CHS = 1; 
                ADCON0bits.GO = 1;               
                acquisitionStatus.battAcquisitionInProgress = 1u;
            } 
        } 
        else
        {
            // start A/D conversion for sonar if done
            if (!ADCON0bits.NOT_DONE) {
                ADCON0bits.CHS = 0; 
                ADCON0bits.GO = 1;
                acquisitionStatus.sonarAcquisitionInProgress = 1u;
            } 
        }                    
        
        PIR1bits.TMR1IF = 0;
    }
    else if (PIE1bits.ADIE && PIR1bits.ADIF)
    {       
        if (acquisitionStatus.battAcquisitionInProgress)
        {
            battLevel[1] = ADRESH;
            battLevel[0] = ADRESL;
            acquisitionStatus.battAcquisitionInProgress = 0u;
        }
        else if (acquisitionStatus.sonarAcquisitionInProgress)
        {
            sonarLevel[1] = ADRESH;
            sonarLevel[0] = ADRESL;
            acquisitionStatus.sonarAcquisitionInProgress = 0u;
        }
        
        PIR1bits.ADIF = 0;
    }
}