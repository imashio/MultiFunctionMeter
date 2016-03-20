//
//  FuelPumpDriver.c
//  MFM_rev6
//
//  Created by imashio on 11/26/14.
//
//

#include "FuelPumpDriver.h"

static const unsigned int INIT = 0x9a;
static const unsigned int RPM_IDLE = 800;
static const unsigned int RPM_MAX = 3250;
static const float FP_LIMIT = 2.55;

unsigned int watch_dog_cnt = 0;
unsigned int watch_dog_thr = 8;

unsigned int adjust = 0;

unsigned int FuelPumpDriver(unsigned long int rpm, float fuel_press, float boost, unsigned int mode){
    unsigned long int   FPD_COMP;
    unsigned long int   rpm_diff;
    
    if( ( mode == 1 ) | ( boost > -0.1 )){
        FPD_COMP = 0xff;
    }else{
                
        if( rpm < RPM_MAX ){
            if( rpm > RPM_IDLE ){
                rpm_diff = rpm - RPM_IDLE;
            }else{
                rpm_diff = 0;
            }
            FPD_COMP = (unsigned int)((float)rpm_diff / (float)( RPM_MAX - RPM_IDLE ) * (float)(0xff-INIT) ) + INIT + adjust;
        }else{
            FPD_COMP = 0xff;
        }
        
        // truncate
        if( FPD_COMP > 0xff ){
            FPD_COMP = 0xff;
        }
    }
    
    return (unsigned int)FPD_COMP;
}

