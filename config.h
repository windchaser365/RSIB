#ifndef CONFIG_H
#define	CONFIG_H

// derived device enabling (do not change!!)
#define _TIMER_0_ON 	0
#define _TIMER_1_ON 	0

// I/O pins (latch)
#define TEST_OUT_PIN LATAbits.LATA1
#define ALIVE_SIGNAL LATAbits.LATA0

// port mapping
#define DIN 		1
#define DOUT 		0

#define ANALOG      0
#define DIGITAL     1

#define I2C_MASK                    0b11111001
#define PWM_READER_I2C_BASE_ADDR    0b00010000
#define PWM_READER_I2C_ADDR         0b0001000
#define BATT_READER_I2C_ADDR        0b0001001
#define SONAR_READER_I2C_ADDR       0b0001010

#define _XTAL_FREQ                  32000000UL

typedef union
{
    struct
    {
        unsigned   sonarAcquisitionInProgress:1;
        unsigned   battAcquisitionInProgress:1;
    };
    struct
    {
        unsigned   value:8;
    };
} acquisitionStatusType;

// config functions
void config(void);

#endif	/* CONFIG_H */

