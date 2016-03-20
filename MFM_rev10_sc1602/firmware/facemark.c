//
//  facemark.c
//  MFM_rev5
//
//  Created by imashio on 11/17/14.
//
//

#include "facemark.h"
#include "lcd_595_softspi.h"

// Set Facemark character
void FaceMark_init(void){
    unsigned int code;
    unsigned int addr;
    
    //code 0-5 is used in "BarMeter"
    // "ω"
    code = 6;
    addr = 0;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00000000);
    addr = 1;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00000000);
    addr = 2;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00000000);
    addr = 3;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00000000);
    addr = 4;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00010001);
    addr = 5;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00010101);
    addr = 6;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00010101);
    addr = 7;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00001010);
    
    // "Д"
    code = 7;
    addr = 0;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0x00);
    addr = 1;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0x00);
    addr = 2;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00011100);
    addr = 3;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00010100);
    addr = 4;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00010100);
    addr = 5;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00010010);
    addr = 6;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00011111);
    addr = 7;
    lcd_set_CGRAMaddr(code,addr);
    lcd_set_char(0b00010001);
    
}

void shobon(void){
    lcd_set_char('(');
    lcd_set_char(0x27);
    lcd_set_char(0xa5);
    lcd_set_char(0x06); // "ω"
    lcd_set_char(0xa5);
    lcd_set_char(0x60);
    lcd_set_char(')');
}

void shakin(void){
    lcd_set_char('(');
    lcd_set_char(0x60);
    lcd_set_char(0xa5);
    lcd_set_char(0x06); // "ω"
    lcd_set_char(0xa5);
    lcd_set_char(0x27);
    lcd_set_char(')');
}

void kuwa(void){
    lcd_set_char('(');
    lcd_set_char(0xdf);
    lcd_set_char(0x07); // "Д"
    lcd_set_char(0xdf);
    lcd_set_char(')');
}
