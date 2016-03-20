//
//  TachoMeter.c
//  MFM_rev5
//
//  Created by imashio on 11/17/14.
//
//

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "TachoMeter.h"

// Tacho Meter Parameter
// Number of pulse, Npulse [pulse/rpm]
#define Npulse          2
// Median Filter Length for Tacho Meter, must be same Nmed in "TachoMeter.h"
#define Nmed            8
// Tracking Range
#define RPM_DIFF        250
// Maximum RPM
#define RPM_MAX         10000
// Count Number Lower Limit (Ignore count less than 'TCNT_LIM')
#define TCNT_LIM        100 // comment 小さくするとおかしい。回転数表示が小さくなる。
// Merging CNT
#define NOISE_CNT       0

unsigned long int   FOSC;
unsigned long int   Ndiv;


double              freq;
double              freq_cur;
unsigned long int   rpm;
unsigned long int   meas_array[Nmed];
unsigned long int   proc_array[Nmed];
unsigned int        tacho_n = 0;
unsigned long int   cnt;

// for debug
unsigned long int   cnt_min;
unsigned long int   cnt_max;

ISR(INT0_vect){
    if( TCNT1 > TCNT_LIM ){
        meas_array[tacho_n] = TCNT1;
        if( tacho_n == Nmed-1 ) tacho_n=0;
        else                    tacho_n++;
        TCNT1 = 0;
    }
}

// 16-bit Timer overflow
ISR(TIMER1_OVF_vect){
    meas_array[tacho_n] = 0xffffffff;
    if( tacho_n == Nmed-1 ) tacho_n=0;
    else                    tacho_n++;
    TCNT1 = 0;
}

// Bubble Sort (min->max)
void BubbleSort(void){
    unsigned int tmp;
    unsigned int min;
    unsigned int min_index = 0;
    unsigned int n,m;
    
    for(m=0;m<Nmed;m++){
        min = 0xffffffff;
        for(n=m;n<Nmed;n++){
            if( proc_array[n] < min ){
                min = proc_array[n];
                min_index = n;
            }
        }
        tmp                     = proc_array[m];
        proc_array[m]           = proc_array[min_index];
        proc_array[min_index]   = tmp;
    }
}

void TachoMeter_init(unsigned long int FOSC_in,unsigned long int Ndiv_in){
    
    FOSC = FOSC_in;
    Ndiv = Ndiv_in;
    
}


unsigned long int TachoMeter(void){
    
    unsigned int n;
    
    // Tacho Meter
    //Median Filter
    for(n=0;n<Nmed;n++){
        proc_array[n] = meas_array[n];
    }
    BubbleSort();
    
    // Calculate frequency
    if(meas_array[tacho_n] >= 0xffff){          // Too Long Gap Pulse
        freq = 0;
    }else{                                      // Last Pulse
        freq = FOSC / Ndiv / meas_array[tacho_n];
    }
    
    // Decide measured frequency is valid or invalid
    
    if( ( freq - freq_cur ) < RPM_DIFF/60.0 ){  // Tracking
        freq_cur = freq;
    }else{                                      // Force Track
        cnt_min = proc_array[0];
        cnt_max = proc_array[Nmed-1];
        cnt = proc_array[Nmed>>1]; // 最新カウント値をセット
        freq_cur = (float)(FOSC / Ndiv) / (float)cnt;
    }
    
    // Calcurate RPM
    rpm = (unsigned long int)( 60.0 * freq_cur / (float)Npulse );
    
    return rpm;
    
}
