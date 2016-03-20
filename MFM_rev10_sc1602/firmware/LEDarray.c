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
#define RPM_LOW         4000
#define RPM_HIGH        7000
#define RPM_FLASH       7000
#define Narray          8
#define Nstep           1

#define LEDarrayDuty_L  8   // NORMAL
#define LEDarrayDuty_H  64  // MAX for FLASH

#define FLASH_SPEED     25

// LED Array Variables Declaration
unsigned int            LED_scale;
unsigned int            rpm_disp = 0;

unsigned int            flash = 0xff;
unsigned int            flash_cnt = 0;

unsigned int            RPM_DIFF;

unsigned int LEDarray_init(){
//void  LEDarray_init(){
    RPM_DIFF = (float)( RPM_HIGH - RPM_LOW )/(float)(Narray/Nstep);
    return RPM_DIFF;
}

void  LEDarray(unsigned int rpm){
    
    if( ( rpm > rpm_disp + 250 ) || ( rpm < ( rpm_disp - 250 ) ) ){
        rpm_disp = rpm;
    }

    if( rpm >= RPM_FLASH ){     // Flash-sequence
        OCR2A   = LEDarrayDuty_H;   // Compare value for LED bar duty
        if( flash_cnt == FLASH_SPEED){
            flash_cnt = 0;
            flash = ~flash;
        }else{
            flash_cnt++;
        }
    }else{                      // Normal-sequence
        flash = 0xff;
        OCR2A   = LEDarrayDuty_L;   // Compare value for LED bar duty
    }

    
    if( rpm_disp < RPM_LOW ){
        LED_scale = 0;
    }else if( rpm_disp == RPM_LOW ){
        LED_scale = Nstep;
    }else if( rpm_disp > RPM_HIGH ){
        LED_scale = Narray;
    }else{
        LED_scale = (unsigned int)((Narray)*(float)( rpm_disp - RPM_LOW )/(float)( RPM_HIGH - RPM_LOW ));
    }

    send_bits_595_LED( ( ~( 0xff << (int)LED_scale ) ) & flash );
        
    
}