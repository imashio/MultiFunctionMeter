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
    //
    
    ADCSRA  = 0b11100111;
    // bit 7    ADEN (ADC enable)
    // bit 6    ADSC (ADC start conversion)
    //              this bit will set 0 after conversion
    //              must set 1 to AD conversion
    // bit 5    ADATE (ADC Auto Trigger enable)
    // bit 4    ADIF (ADC Interrput flag)
    // bit 3    ADIF (ADC Interrput enable)
    // bit 2:0  ADPS (ADC Prescaler Select Bits)
    
    ADCSRB  = 0b00000000;
    // bit 2:0  ADTS (ADC Auto Trigger Source)
    //          000 continuous
    
}