#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd_595_softspi.h"


#define         BarMeter_N          8
#define         BarMeter_MIN        0.0
#define         BarMeter_MAX        8000

float    BarMeter_INTG = (BarMeter_MAX-BarMeter_MIN)/BarMeter_N;

void BarMeter_init(void){
    unsigned int code;
    unsigned int addr;
    
    for(code=0;code<6;code++){
        for(addr=0;addr<8;addr++){
            lcd_set_CGRAMaddr(code,addr);
            if( (addr > 1) & (addr < 6) )
                lcd_set_char(~(0x1f>>code));
            else if( (addr == 6) | (addr == 1) ) lcd_set_char(0x00);
            else                                 lcd_set_char(0x10);
        }
    }
}

void BarMeter_disp(unsigned int data){
    
    unsigned int    conv_data;
    
    unsigned char   INTG;
    unsigned char   FRAC;
    unsigned int    n;
    
    if( data > BarMeter_MAX ){
        data = BarMeter_MAX;
    }else if( data < BarMeter_MIN ){
        data = BarMeter_MIN;
    }
    
    conv_data = data - BarMeter_MIN;
    INTG = (unsigned int)(conv_data/BarMeter_INTG);
    FRAC = (unsigned int)((conv_data - BarMeter_INTG*INTG)*1.0/BarMeter_INTG * 5);

    for(n=0;n<INTG;n++){
        lcd_set_char(0x5);
    }
    if( INTG != BarMeter_N ) lcd_set_char(FRAC);
    for(n=INTG+1;n<BarMeter_N;n++){
        lcd_set_char(0x0);
    }

}

