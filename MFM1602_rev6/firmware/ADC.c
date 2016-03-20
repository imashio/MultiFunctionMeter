//
//  ADC.c
//  MFM_rev6
//
//  Created by imashio on 12/3/14.
//
//

#include <avr/io.h>

#include "ADC.h"

void ADC_init(){
    
    ADMUX   = 0b00100000;
    ADCSRA  = 0b11000111;
    ADCSRB  = 0b00000000;
    
}