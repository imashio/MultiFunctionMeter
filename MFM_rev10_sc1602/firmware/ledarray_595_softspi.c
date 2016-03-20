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
#define DD_MOSI         PB3
#define DD_SCK          PB4
#define DD_RCK          PB5

unsigned char E;
unsigned char RS;
unsigned char DATA;

void SoftSPI_LED_Init(void){
    /*MOSI,SCK=出力、他は入力に設定 */
    DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_RCK);
}

void SoftSPI_LED_TX(unsigned char cData){
    unsigned int n;
    for(n=0;n<8;n++){
        PORT_SPI &= ~(1<<DD_SCK);               // Set SCK "L"
        PORT_SPI &= ~(1<<DD_MOSI);              // Initialize Data Port
        PORT_SPI |= (0x01&(cData>>(7-n)))<<DD_MOSI; // Set Data bit
        PORT_SPI |= (1<<DD_SCK);                // Set SCK "H"
    }
}

void send_bits_595_LED(unsigned char DATA){
    unsigned char bits;
    bits = DATA;
    SoftSPI_LED_TX(bits);
    PORT_SPI &= ~(1<<DD_RCK);   // Set RCK "L"
    PORT_SPI |=  (1<<DD_RCK);   // Set RCK "H"
    PORT_SPI &= ~(1<<DD_RCK);   // Set RCK "L"
}

