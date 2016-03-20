//
//  tictoc.c
//  MFM_rev4
//
//  Created by imashio on 11/15/14.
//
//

#include <avr/io.h>
#include <util/delay.h>
#include "tictoc.h"

// This function uses TCNT1
// Must enable 16-bit timer

double              unit_time_us;
unsigned long int   start_cnt;
unsigned long int   end_cnt;

void tictoc_init(unsigned int fosc_MHz, unsigned int Ndiv){
    
    unit_time_us = Ndiv / ( 1.0*fosc_MHz );
    
}

void tic(){
    
    start_cnt = TCNT1;
    
}

unsigned long int toc(){
    
    unsigned long int time_us;
    
    end_cnt = TCNT1;
    if( start_cnt < end_cnt ){
        time_us = ( ( 0x10000 + end_cnt ) - start_cnt ) * unit_time_us;
    }else{
        time_us = ( end_cnt - start_cnt ) * unit_time_us;
    }
    return time_us;
    
}

