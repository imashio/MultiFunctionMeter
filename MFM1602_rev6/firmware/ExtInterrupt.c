/*
 *  ExtInterrupt.c
 *  DefiLinkTap_rev2b
 *
 *  Created by Ryuta on 10/11/28.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "ExtInterrupt.h"

void ExtInterrupt_init(void) {

   // INT0, INT1 Pin change interrupt enable
	EIMSK = 0b00000001;
	// bit 7-2 : not valid ,  bit 1 : INT1 ,  bit 0 : INT0 
	
   // INT0, INT1 Pin change interrupt mode setting
	EICRA = 0b00000011;
	// bit 7-4 : not valid ,  bit 3-2 : INT1 ,  bit 1-0 : INT0 
	//		00 ... Low
	//		01 ... Logic changing
	//		10 ... Negative edge
	//		11 ... Positive edge


	// Pin group mask
	PCICR = 0b00000000;
	// bit 0 : PCINT  0 -  7
	// bit 1 : PCINT  8 - 14
	// bit 2 : PCINT 16 - 23

	// Pin mask
	// PCMSK2 is valid, when bit 2 of PCICR is setted 
	// PCINT23, PCINT22, PCINT21, PCINT20, PCINT19, PCINT18, PCINT17, PCINT16
	PCMSK2 = 0b00000000;
	// PCMSK1 is valid, when bit 1 of PCICR is setted 
	//  ----- , PCINT14, PCINT13, PCINT12, PCINT11, PCINT10,  PCINT9,  PCINT8
	PCMSK1 = 0b00000000;
	// PCMSK0 is valid, when bit 0 of PCICR is setted 
	//  PCINT7,  PCINT6,  PCINT5,  PCINT4,  PCINT3,  PCINT2,  PCINT1,  PCINT0
	PCMSK0 = 0b00000000;
}