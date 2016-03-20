/*
 *  EEPROM_lib.h
 *  TempMeter_2ch_LCD_rev1
 *
 *  Created by Imashow on 8/8/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

void EEPROMinit(void);

void EEPROMwrite(unsigned int EEPROMaddress, unsigned char EEPROMdata);

unsigned char EEPROMread(unsigned int EEPROMaddress);
