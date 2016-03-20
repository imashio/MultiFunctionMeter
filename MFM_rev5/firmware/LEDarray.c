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
#define RPM_LOW         10
#define RPM_HIGH        9000
#define RPM_FLASH       10000


// LED Array Variables Declaration
int                 LED_scale;

void  LEDarray(unsigned int rpm){

    if( rpm < RPM_LOW ){
        LED_scale = 0;
    }else if( rpm > RPM_HIGH ){
        LED_scale = 8;
    }else{
        LED_scale = 8.0*( rpm - RPM_LOW )/( RPM_HIGH - RPM_LOW );
    }
    send_bits_595_LED(~( 0xff << (int)LED_scale ));
    
}