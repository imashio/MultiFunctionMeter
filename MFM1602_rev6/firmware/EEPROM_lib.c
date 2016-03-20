/*
 *  EEPROM.c
 *  
 *
 *  Created by Imashow on 8/8/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "EEPROM_lib.h"

void EEPROMinit(void){
	// EEPROM initialize
	EEARH = 0x00;	// Upper EEPROM address (Valid only LSB 1-bit)
	EEARL = 0x00;	// Lower EEPROM address
	EEDR  = 0x00;	// EEPROM read & write data
	EECR  = 0b00000000;	// EEPROM control register
	//			|||||+ EERE  : EEPROM read enable
	//			||||+  EEPE  : EEPROM program enable
	//			|||+   EEMPE : EEPROM master program enable
	//			||+    EERIE : EEPROM ready interupt enable
	//			++     EEPM  : EEPROM program mode
}

void EEPROMwrite(unsigned int EEPROMaddress, unsigned char EEPROMdata){
	while(EECR & (1<<EEPE))			// wait annother EEPROM programming
	while(SPMCSR & (1<<SELFPRGEN))	// wait self programing
	cli();							// disable intruput
	EEARL = EEPROMaddress;
	EEDR  = EEPROMdata;
	EECR = _BV(EEMPE);				// set master program enable
	EECR = _BV(EEPE);				// set program enable
	sei();
}

unsigned char EEPROMread(unsigned int EEPROMaddress){
	while(EECR & (1<<EEPE))			// wait annother EEPROM programming
	while(SPMCSR & (1<<SELFPRGEN))	// wait self programing
	cli();							// disable intruput
	EEARL = EEPROMaddress;
	EECR = _BV(EERE);				// set read enable
	sei();
	return EEDR;
}

