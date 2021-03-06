// USART transmission Functions

#include <avr/io.h>
#include <avr/delay.h>
#include "usart.h"
#define USART_STATUS UCSR0A

/*
-- You should define variables in your source code --
#define FOSC 8000000 // Clock Speed 
#define BAUD   19200 
#define UBRR FOSC/16/BAUD-1 
*/

// USART initialize
void USARTinit(unsigned int ubrr){
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	unsigned int Mode= 0b00;	// USART mode select
								//   '00' : Asynchronous
								//   '01' : Synchronous
	unsigned int Pari= 0b10;	// Parity mode select
								//   '00': No parity
								//   '10': Even parity
								//   '11': Odd parity
	unsigned int STPB= 0b0;		// Stop bit length
								//   '0' : 1-bit
								//   '1' : 2-bit
	unsigned int CLKE= 0b0;		// Clock edge for sampling
								//  *This bit must be set '0' in Asynchronous mode 
								//   '0' : Negative 
								//   '1' : Positive
	unsigned int RCIE= 0b1;		// Receive complete interuput enable
	unsigned int TCIE= 0b0;		// Transmit complete interuput enable
	unsigned int TRIE= 0b0;		// Transmit Data register empty interrupt enable
	unsigned int RE  = 0b1;		// Receive enable
	unsigned int TE  = 0b0;		// Transmit enable
	unsigned int Nb  = 0b011;	// Number of bit per packet
								//   '000' : 5bit
								//   '001' : 6bit
								//   '010' : 7bit
								//   '011' : 8bit
	unsigned int U2X = 0;		// Over clocking operation
	UCSR0B = (RCIE<<7)|(TCIE<<6)|(TRIE<<5)|(RE<<4)|(TE<<3)|((Nb>>2)<<2);
	UCSR0C = (Mode<<6)|(Pari<<4)|(STPB<<3)|(Nb<<1)|(CLKE);
	UCSR0A = U2X<<1;
}

// USART transmit
void USART_transmit(unsigned char data){
	while(!(USART_STATUS & (1<<UDRE0)));	// Wait until tx buffer is empty
	UDR0 = data;
}


// USART receive without wait & Error rejection
/*
unsigned char USART_receive(void){
	return UDR0;
}
*/

// USART receive w/o. Time-out
unsigned char USART_receive(void){
	unsigned char error_data;
	while(!(USART_STATUS & (1<<RXC0)));	// Wait receive process
	if ( !( ( USART_STATUS & (1<< FE0))
	      | ( USART_STATUS & (1<<UPE0)) ) ){	// detect framing & parity error
		return UDR0;
	}else{
		error_data = UDR0;
		return 0xff;
	}
}

// USART receive w. Time-out
unsigned char USART_receive_wTO(void){
	unsigned char error_data;
	unsigned int n = 0;
	unsigned int m = 0;
	
	for(n=0;n<1280;n++){
		if( (USART_STATUS & (1<<RXC0)) ){
			if ( !( ( USART_STATUS & (1<< FE0))
				   | ( USART_STATUS & (1<<UPE0)) ) ){	// detect framing & parity error
				return UDR0;
			}else{
				error_data = UDR0;
				return 0xff;
			}
		}
	}
	return 0xff;
}
