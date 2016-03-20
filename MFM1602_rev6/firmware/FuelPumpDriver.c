//
//  FuelPumpDriver.c
//  MFM_rev6
//
//  Created by imashio on 11/26/14.
//
//

#include "FuelPumpDriver.h"

unsigned int FuelPumpDriver(unsigned long int rpm, float fuel_press, float boost){
    unsigned long int    FPD_COMP;
    
    if( rpm < 3000 ){
        FPD_COMP = rpm / 3000 * 0xff;
    }else{
        FPD_COMP = 0xff;
    }
    
    // truncate
    if( FPD_COMP < 0x7f ){
        FPD_COMP = 0x7f;
    }else if( FPD_COMP > 0xff ){
        FPD_COMP = 0xff;
    }
    
    return (unsigned int)FPD_COMP;
}

