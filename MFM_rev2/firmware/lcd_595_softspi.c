// LCD Control Functions
// Target : ATMEGA88 (ATMEGA168)
// Clock  : 1MHz (Fuse : CKDIV8 = 0, Fullswing crystal)
//

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd_595_softspi.h"

// SPI port Dfinition
#define DDR_SPI         DDRB
#define PORT_SPI        PORTB
#define DD_MOSI         PB0
#define DD_SCK          PB1
#define DD_RCK          PB2

unsigned char E;
unsigned char RS;
unsigned char DATA;

void SoftSPI_Init(void){
    /*MOSI,SCK=出力、他は入力に設定 */
    DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_RCK);
}

void SoftSPI_TX(unsigned char cData){
    unsigned int n;
    for(n=0;n<8;n++){
        PORT_SPI &= ~(1<<DD_SCK);               // Set SCK "L"
        _delay_us(1);
        PORT_SPI &= ~(1<<DD_MOSI);              // Initialize Data Port
        PORT_SPI |= (0x01&(cData>>(7-n)))<<DD_MOSI; // Set Data bit
        _delay_us(1);
        PORT_SPI |= (1<<DD_SCK);                // Set SCK "H"
        _delay_us(1);
    }
}

void send_bits_595(unsigned char RS, unsigned char E, unsigned char DATA){
    unsigned char bits;
    bits = 0x3f & ( (RS<<5)|(E<<4)|DATA );
    SoftSPI_TX(bits);
    _delay_us(10);
    PORT_SPI &= ~(1<<DD_RCK);   // Set RCK "L"
    PORT_SPI |=  (1<<DD_RCK);   // Set RCK "H"
    PORT_SPI &= ~(1<<DD_RCK);   // Set RCK "L"
}

// fuction for sending control bitpattern
void lcd_set_4bit(unsigned char bitpattern){
	DATA    =   0x0f & bitpattern; // set sending data
	E       =   0;
    send_bits_595(RS,E,DATA);
//	_delay_us(20);
	E       =   1;
    send_bits_595(RS,E,DATA);
//	_delay_us(20);
	E       =   0;
    send_bits_595(RS,E,DATA);
//	_delay_us(20);
}

// Initializing LCD
void lcd_init(void){

	// set enable "0"
    E       = 0;
    // set instruction input mode
	RS      = 0;
	// initialize data port
	DATA	= 0x00;
    send_bits_595(RS,E,DATA);
    
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

	// set display on, cursol off and blink off
	lcd_set_4bit(0x00);
	lcd_set_4bit(0x0c);
	_delay_ms(1);

	// set entry mode (cursol direction : right, disable shift)
	lcd_set_4bit(0x00);
	lcd_set_4bit(0x06);
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
    RS      =   1;
}

// LCD character allocation
void lcd_locate(unsigned char row, unsigned char col)
{
	// set instruction input mode
    RS      =   0;
//	lcd_set_char(0x80 | col | (0x40 * row) );
	lcd_set_char(0x80 | ( col + 20*(unsigned int)(row/2) ) | ( 0x40 * (row%2) ) );
	// set data input mode
    RS      =   1;
}

// LCD character (1 byte) sending
void lcd_set_char(unsigned char c)
{
	lcd_set_4bit(c >> 4); // Upper Bit
	lcd_set_4bit(0x0f & c); // Lower Bit
	_delay_us(50);
}

// LCD String Sending
void lcd_set_str(unsigned char *s)
{
	while (*s != 0) {
		lcd_set_char(*s);
		s++;
	} 
}

// Display numeric on LCD display
void lcd_set_numeric(unsigned int num, unsigned int Nint, unsigned int Nfrac){
    unsigned long int DivFactor = 1;
	unsigned int n, m;
	unsigned int d[10];
    unsigned int valid = 0;
    for(n=0;n<Nfrac;n++){
        num = num * 10;
    }
    for(n=0;n<=Nint+Nfrac-1;n++){
        
        DivFactor = 1;
        for(m=n+1;m<Nint+Nfrac;m++){
            DivFactor = DivFactor * 10;
        }
        
        d[n] = (unsigned int)( num / DivFactor );
        if( (valid == 0) & (d[n]!=0) ) valid = 1;
        num = num - d[n]*DivFactor;
        
        if( (valid == 1)|(n == (Nint+Nfrac-1)) ) lcd_set_char(0x30 + d[n]);
        else if( valid == 0 )                   lcd_set_char(0x20);
        else                                    lcd_set_char(0xff);
    }
}

// Clear LCD
void lcd_clear(){
	// clear display
    RS      =   0;
	lcd_set_4bit(0x00);
	lcd_set_4bit(0x01);
	_delay_ms(1);
    RS      =   1;
	lcd_locate(0,0);
}

// Set CGRAM address
void lcd_set_CGRAMaddr(unsigned char code,unsigned char addr)
{
	// set instruction input mode
    RS      =   0;
    lcd_set_char(0x40 | code<<3 | addr );
	// set data input mode
    RS      =   1;
}

