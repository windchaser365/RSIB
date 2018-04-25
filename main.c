/*
 * File:   main.c
 * Author: giuseppe
 *
 * Created on 22 gennaio 2017, 10.07
 */

#include <xc.h>
#include "config.h"
#include "types.h"

extern acquisitionStatusType acquisitionStatus;

void main(void) 
{
    acquisitionStatus.value = 0u;
    
	config();       

    while(1) 
    {        
        __delay_us(100);
        LATAbits.LATA3 = !PORTAbits.RA3;
	}        
}

