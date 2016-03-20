//
//  LEDarray.c
//  MFM_rev3
//
//  Created by imashio on 11/6/14.
//
//

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "ledarray_595_softspi.h"
#include "LEDarray.h"

// LED Array
#define RPM_LOW         3500
#define RPM_HIGH        7000
#define RPM_FLASH       7200
#define Narray          8

#define LEDarrayDuty_L  8
#define LEDarrayDuty_H  64

#define FLASH           25

// LED Array Variables Declaration
unsigned int            LED_scale;
unsigned int            rpm_disp;

unsigned int            flash = 0xff;
unsigned int            flash_cnt = 0;

void  LEDarray(unsigned int rpm){
    
    if( (rpm > rpm_disp+250) || (rpm < rpm_disp-250)){
        rpm_disp = rpm;
    }

    if( rpm >= RPM_FLASH ){     // Flash-sequence
        OCR2A   = LEDarrayDuty_H;   // Compare value for LED bar duty
        if( flash_cnt == FLASH){
            flash_cnt = 0;
            flash = ~flash;
        }else{
            flash_cnt++;
        }
    }else{                      // Normal-sequence
        flash = 0xff;
        OCR2A   = LEDarrayDuty_L;   // Compare value for LED bar duty
    }

    
    if( rpm < RPM_LOW ){
        LED_scale = 0;
    }else if( rpm > RPM_HIGH ){
        LED_scale = Narray;
    }else{
        LED_scale = (unsigned int)(Narray*(float)( rpm_disp - RPM_LOW )/(float)( RPM_HIGH - RPM_LOW ));
    }

    send_bits_595_LED( ( ~( 0xff << (int)LED_scale ) ) & flash );
        
    
}