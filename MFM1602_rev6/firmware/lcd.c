// LCD Control Functions
// Target : ATMEGA88 (ATMEGA168)
// Clock  : 1MHz (Fuse : CKDIV8 = 0, Fullswing crystal)
//

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd.h"

// I/O
//  PB0 : OUT : Vo
//  PB1 : OUT : RS
//  PB2 : OUT : R/W
//  PB3 : OUT : E
//  PC0 : OUT : D4
//  PC1 : OUT : D5
//  PC2 : OUT : D6
//  PC3 : OUT : D7
//

// Registor & BIT Configration
#define LCD_PORT      PORTC	//LCD DATA PORT Registor
#define LCD_DDR       DDRC	// LCD DATA PORT DDR registor
#define LCD_PORT_E_RS PORTB	//LCD E & RS PORT Registor
#define LCD_DDR_E_RS  DDRB // 
#define LCD_Vo		  0x01	// Contrast adujust (1:Lowest, 0: Highest) 
							// PB0
#define LCD_RW		  0x04	// Read / Write
							// PB2
#define LCD_E		  0x08	// Enable
							// PB3
#define LCD_RS		  0x02	// Register Select
							// PB1 

// fuction for sending control bitpattern
void lcd_set_4bit(unsigned char bitpattern){
//	cli();
	LCD_PORT		= bitpattern; // set sending data
	LCD_PORT_E_RS	^= LCD_E; // set E "H"
	_delay_us(20);
	LCD_PORT_E_RS	^= LCD_E; // set E "L"
	_delay_us(20);	
//	sei();
}

// Initializing LCD
void lcd_init(void)
{
	// set output register
	LCD_DDR      = _BV(3)|_BV(2)|_BV(1)|_BV(0);
	LCD_DDR_E_RS = _BV(3)|_BV(2)|_BV(1)|_BV(0);
	// E, R/W, RS, Vo

	// set instruction input mode
	LCD_PORT_E_RS	= 0x00;

	// initialize data port
	LCD_PORT		= 0x00;

	// wait more than 15ms
	_delay_ms(20);

	// set 8-bit mode
	lcd_set_4bit(0x03);
	_delay_ms(5);
	lcd_set_4bit(0x03);
	_delay_us(110);
	lcd_set_4bit(0x03);
	_delay_us(50);
	
	// set 4-bit mode
	lcd_set_4bit(0x02);
	_delay_us(50);

	// function setting
	lcd_set_4bit(0x02);
	lcd_set_4bit(0x08);
	_delay_ms(1);

	// set display off, cursol off and blink off
	lcd_set_4bit(0x00);
	lcd_set_4bit(0x0c);
	_delay_ms(1);

	// set entry mode (cursol direction : right, disable shift)
	lcd_set_4bit(0x00);
	lcd_set_4bit(0x03);
	_delay_ms(1);

	// clear display
	lcd_set_4bit(0x00);
	lcd_set_4bit(0x01);
	_delay_ms(2);

	// clear cursol position
	lcd_set_4bit(0x00);
	lcd_set_4bit(0x02);
	_delay_ms(2);

	// set data input mode
	LCD_PORT_E_RS	^= LCD_RS;
}

// LCD character allocation
void lcd_locate(unsigned char row, unsigned char col)
{
//	cli();
	// set instruction input mode
	LCD_PORT_E_RS	^= LCD_RS;
	lcd_set_char(0x80 | col | (0x40 * row) );
	// set data input mode
	LCD_PORT_E_RS	^= LCD_RS;
//	sei();
}

// LCD character (1 byte) sending
void lcd_set_char(unsigned char c)
{
//	cli();
	lcd_set_4bit(c >> 4); // Upper Bit
	lcd_set_4bit(0x0f & c); // Lower Bit
	_delay_ms(1);
//	sei();
}

// LCD String Sending
void lcd_set_str(unsigned char *s)
{
//	cli();
	while (*s != 0) {
		lcd_set_char(*s);
		s++;
	} 
//	sei();
}

// Display numeric on LCD display
void lcd_set_numeric(float num, unsigned int Nint, unsigned int Nfrac, unsigned int sign){
    unsigned long int DivFactor = 1;
    unsigned int n, m;
    unsigned int d[10];
    unsigned int valid = 0;
    
    for(n=0;n<Nfrac;n++){
        num = num * 10;
    }
    
    // Detect sign
    if( sign == 1){
        if( num < 0 ){
            lcd_set_char('-');
            num = -num;
        }else{
            lcd_set_char('+');
        }
    }
    // end of
				
    for(n=0;n<=Nint+Nfrac-1;n++){
        
        if( ( n== Nint ) && ( Nfrac!=0 )){
            lcd_set_char('.');
        }
        
        DivFactor = 1;
        for(m=n+1;m<Nint+Nfrac;m++){
            DivFactor = DivFactor * 10;
        }
        
        d[n] = (unsigned int)( num / DivFactor );
        if( (valid == 0) && ( d[n]!=0 || n==Nint-1) ) valid = 1;
        num = num - d[n]*DivFactor;
        
        if( (valid == 1)|(n == (Nint+Nfrac-1)) ) lcd_set_char(0x30 + d[n]);
        else if( valid == 0 )                   lcd_set_char(0x20);
        else                                    lcd_set_char(0xff);
    }
}

// clear display
void lcd_clear(){
//	cli();
	LCD_PORT_E_RS	^= LCD_RS;
	lcd_set_4bit(0x00);
	lcd_set_4bit(0x01);
	_delay_ms(2);
	LCD_PORT_E_RS	^= LCD_RS;
	lcd_locate(0,0);
//	sei();
}


// Set CGRAM address
void lcd_set_CGRAMaddr(unsigned char code,unsigned char addr)
{
    // set instruction input mode
    LCD_PORT_E_RS	^= LCD_RS;
    lcd_set_char(0x40 | code<<3 | addr );
    // set data input mode
    LCD_PORT_E_RS	^= LCD_RS;
}
