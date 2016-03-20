//
//             Multi Function Meter
//                  Revision 3
//
//    Display data of Defi Link Unit 2 daisy chain
//   "Measure Defi Link Unit 2 data interval freqency"
//
//      Display : SC2004
//

#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "lcd_595_softspi.h"
#include "ledarray_595_softspi.h"
#include "ExtInterrupt.h"
#include "usart.h"
#include "tictoc.h"
#include "TachoMeter.h"
#include "facemark.h"

// MCU clock speed (for USART & Tacho)
#define FOSC			16000000 // Clock Speed

// USART transmission speed definition
#define BAUD			19200
#define UBRR			FOSC/16/BAUD-1

// Number of Display data
#define Ndata           4

// Wait time interval
#define	WAIT			150 // unit : ms

// LCD width
#define LCD_W           20

// Display character width of Defi Link Tap "name + data"
#define DISP_W          11
//#define DISP_W          7

// LED bar duty ( 0:0% - 255:100% )
#define LEDarrayDuty    32

uint8_t     chg_index   = Ndata;
uint16_t    chg_count   = 0xffff;

//
uint8_t   RxID[] = {
        0x01,	// Turbo
        0x02,	// Tacho
        0x03,	// Oil pres.
        0x04,	// Fuel pres.
        0x05,	// Ext. Temp.
        0x07,	// Oil Temp.
        0x0f	// Water Temp.
};

// Measure Tarfet ID Declarations
uint8_t     t_id[Ndata];
void set_initial_t_id(void){
    t_id[0] = 6;
    t_id[1] = 5;
    t_id[2] = 3;
    t_id[3] = 0;
}

uint8_t*    RxName[7];
uint8_t*    RxName_short[7];
uint8_t*    RxName_long[7];

uint8_t     RxNameLength[7];

uint8_t     lcd_update  = 1;		// if bit is "1" then Re-draw LCD

unsigned char	data[Ndata][4];     // Recive byte data of packet for processing
//	data[*][0] : Control
//	data[*][1] : Angle Data (MSB)
//	data[*][2] : Angle Data
//	data[*][3] : Angle Data (LSB)

// Tacho Meter Variables Declaration
unsigned long int   rpm;

// Variables for display update timer
uint16_t            timer0_cnt;
uint16_t            timer0_cnt_last = 0;
uint16_t            delay_cnt;


// Bad ISR interrput detector for debugging
ISR(BADISR_vect){
    cli();    // これ以上割り込ませない
    lcd_locate(0,0);
    lcd_set_str("BADISR ERROR");
    while(1);  // ここで止めておく
}


// 8-bit timer intialazation for PWM FuelPump Driver
unsigned long int   Ndiv0;
void timer0_init(void) {
    TCCR0A  = 0b11000001;	// Timer/Counter1 Control Register A
    TCCR0B  = 0b00000100;
    TIMSK0  = 0b00000101;   // ovfl interrupt is enabled for delay timer
    TCNT0   = 0x00;         // Initialize 8-bit counter bit
    OCR0A   = 0x80;         // Compare value for Fuel Pump driver PWM
    OCR0B   = LEDarrayDuty; // Compare value for LED bar duty

    switch( TCCR0B & 0b00000111 ){
        case 0b001 :
            Ndiv0 = 1;
            break;
        case 0b010 :
            Ndiv0 = 8;
            break;
        case 0b011 :
            Ndiv0 = 64;
            break;
        case 0b100 :
            Ndiv0 = 256;
            break;
        case 0b101 :
            Ndiv0 = 1024;
            break;
        default :
            Ndiv0 = 1;
            break;
    }
    
}

// 16-bit Counter intialazation for Tacho Meter
unsigned long int   Ndiv1;
void timer1_init(void) {
	TCCR1A  = 0b00000000;	// Timer/Counter1 Control Register A
	TCCR1B  = 0b00000011;
	TCCR1C  = 0b00000000;
	TIMSK1  = 0b00000001;
	TCNT1   = 0x0000;         // Initialize 16-bit counter bit
	OCR1A   = 0xffff;         // 16-bit register for compare

    switch( TCCR1B & 0b00000111 ){
        case 0b001 :
            Ndiv1 = 1;
            break;
        case 0b010 :
            Ndiv1 = 8;
            break;
        case 0b011 :
            Ndiv1 = 64;
            break;
        case 0b100 :
            Ndiv1 = 256;
            break;
        case 0b101 :
            Ndiv1 = 1024;
            break;
        default :
            Ndiv1 = 1;
            break;
    }
    
}

// Count character length
int StrLength(const char *s){
    int n = 0;
    while (*s++ != '\0')
        n++;
    return (n);
}

// Display
void DisplayItemInfo(void){
	int k=0;
	for(k=0;k<Ndata;k++){
		lcd_locate(k,0);
		lcd_set_str((unsigned char*)RxName[t_id[k]]);
	}
	
}

ISR(USART_RX_vect){
    unsigned int    ID;
    unsigned int    n, m;

    ID = USART_receive_woTO(); // Synchronize & Detect receiver ID
    for(n=0;n<Ndata;n++){
        if ( ID == RxID[t_id[n]] ){
            for(m=0;m<4;m++){
                data[n][m] = USART_receive_wTO();
            }
            
            // measure interval time, debug
            if( ID == RxID[t_id[1]] ){
                unsigned long int time_us;
                time_us = toc();
//                lcd_locate(2,14);
//                lcd_set_numeric(time_us/1000.0,3,1,0);
                tic();
            }
            
            break;
        }
    }
}

// 8-bit Timer overflow
ISR(TIMER0_OVF_vect){
    // increment counter for display update
    timer0_cnt++;
    
    // LED array : display
    LEDarray((unsigned int)rpm);
}


ISR(TIMER0_COMPB_vect){
    // LED array : off
    send_bits_595_LED(0x00);
}

int main(void)
{

    // for 74HC595 port setting for LCD
    SoftSPI_Init();
    
    // for 74HC595 port setting for LED array
    SoftSPI_LED_Init();
    
    // Initialize LCD
	lcd_init();
	
    // PWM & delay counter init
    timer0_init();
    
    // PWM putput port definition
    DDRD |= (1<<PD6);
    
    // TachoMeter counter init
    timer1_init();

	// USART initialize
	USARTinit(UBRR);
	
	// Ext. Interupt setting
	ExtInterrupt_init();
    
    // TicToc initialize
    tictoc_init(FOSC, Ndiv1);
    
    // Tacho Meter Initialize
    TachoMeter_init(FOSC,Ndiv1);
    
    // Bar-Meter Initialize
    BarMeter_init();
    
    // Facemark character Initialize
    FaceMark_init();
    
    // Set Initial Target IDs
    set_initial_t_id();
    
    delay_cnt = (unsigned long int)( ( WAIT*1.0 ) * ( 1.0*FOSC/Ndiv0 ) / 1000 / 256 );
    
	// Declarations
	unsigned char*   opening_message0 = "Multi-Function Meter";
    unsigned char*   opening_message1 = " w/ FuelPump Driver ";
    unsigned char*   opening_message2 = "   Firmware Rev.5   ";
    
	uint8_t         n, m;					// 'for' loop variables
    
    uint8_t         FPDcomp = 0xff;
    
	uint8_t         index = 0;				// LCD displaying data index
	
	uint16_t		maxv = 2352;			// maximum decimal angle data value from 'Defi Link Unit II'
    
	uint8_t         id;						// ID index for processing
    
	uint8_t         valid_packet[Ndata];	// Validtity indicator
	
	uint8_t			low4bits[4];			// Extracted lower 4 bits from byte data
	uint16_t        dec_ang;				// Angle data (decimal)
	float           dec_nrm;				// Angle data (decimal)
	float			value[Ndata];			// Decoded value
    uint16_t        mult_factor[3];		// Multiplying factor for hexadecimal to decimal decoding
    
	uint8_t         digits_int[5];			// Digits integer data
	unsigned char	digits_char[5];			// Digits character data for display
    
	float           div_factor;				// Dividing factor for integer
	uint8_t         digits_valid;			// Indicate digits in integer are valid or invalid
    
    // value = eq_grad * dec_nrm + eq_intercept
    // Gradient-term of decoding equation
	uint16_t eq_grad[] = {
        3,	// Turbo
        9000,	// Tacho
        10,	// Oil pres.
        6,	// Fuel pres.
        900,	// Ext. Temp.
        100,	// Oil Temp.
        100	// Water Temp.
    };
    
    // Intercept-term of decoding equation
    int16_t eq_intercept[] = {
        -1,	// Turbo
        0,	// Tacho
        0,	// Oil pres.
        0,	// Fuel pres.
        200,	// Ext. Temp.
        50,	// Oil Temp.
        20	// Water Temp.
    };
    
    // Definition of number of significant figure
	uint8_t   Nsig[] = {				// Number of significant figures
        3,	// Turbo
        4,	// Tacho
        3,	// Oil pres.
        3,	// Fuel pres.
        4,	// Ext. Temp.
        3,	// Oil Temp.
        3	// Water Temp.
    };
    
    // Deifinition of number of integer figure
	uint8_t	Nint[] = {				// Number of integr digits
        1,	// Turbo
        4,	// Tacho
        2,	// Oil pres.
        1,	// Fuel pres.
        4,	// Ext. Temp.
        3,	// Oil Temp.
        3	// Water Temp.
    };
    
    uint8_t	SIGN[] = {				// Show +/-, enable showing is '1'
        1,	// Turbo
        0,	// Tacho
        0,	// Oil pres.
        0,	// Fuel pres.
        0,	// Ext. Temp.
        0,	// Oil Temp.
        0	// Water Temp.
    };
    
	uint8_t	Nspace[7];				// Number of space between character and digits
    float   Resolution[7];


    RxName[0]		=   "BOOST";
    RxName[1]		=   "TACHO";
    RxName[2]		=   "OIL.P";
    RxName[3]		=   "FUEL.P";
    RxName[4]		=   "EXT.T";
    RxName[5]		=   "OIL.T";
    RxName[6]		=   "WATER.T";

/*
	RxName[0]		=   "Boost";
	RxName[1]		=   "Tacho";
	RxName[2]		=   "Oil.P";
	RxName[3]		=   "Fuel.P";
	RxName[4]		=   "ExTmp";
	RxName[5]		=   "Oil.T";
	RxName[6]		=   "Water.T";
*/
    
/*
	RxName[0]	=   "BS";
	RxName[1]	=   "TC";
	RxName[2]	=   "OP";
	RxName[3]	=   "FP";
	RxName[4]	=   "ET";
	RxName[5]	=   "OT";
	RxName[6]	=   "WT";
*/
    
/*
	RxName[0]	=   "Boost";
	RxName[1]	=   "Tacho";
	RxName[2]	=   "Oil press";
	RxName[3]	=   "Fuel press";
	RxName[4]	=   "Ext. Temp.";
	RxName[5]	=   "Oil Temp.";
	RxName[6]	=   "Water Temp.";
*/
    
    // Definition of Resolution for processing and number of space for display
    for(n=0;n<7;n++){
        Resolution[n] = 1;
        for(m=0;m<Nsig[n]-Nint[n];m++){
            Resolution[n] = Resolution[n] / 10;
        }
        RxNameLength[n] = StrLength(RxName[n]);
        Nspace[n]       = DISP_W - RxNameLength[n] - ( Nsig[n] + (Nsig[n]!=Nint[n]) + SIGN[n] );
    }
    
    
	mult_factor[0] = 1;
	mult_factor[1] = 16;
	mult_factor[2] = 256;
    
    
    // opening @ LED array
    for(n=0;n<=8;n++){
        send_bits_595_LED(0x01 << n);
        _delay_ms(60);
    }

    // opening @ LCD
	_delay_ms(50);
	lcd_locate(1,0);
	for(n=0;n<LCD_W;n++){
		lcd_set_char(opening_message0[n]);
		_delay_ms(20);
	}
    lcd_locate(2,0);
    for(n=0;n<LCD_W;n++){
        lcd_set_char(opening_message1[n]);
        _delay_ms(20);
    }
    lcd_locate(3,0);
    for(n=0;n<LCD_W;n++){
        lcd_set_char(opening_message2[n]);
        _delay_ms(20);
    }
    
    // opening @ LED array
    for(n=0;n<=8;n++){
        send_bits_595_LED(~( 0xff << n ));
        _delay_ms(30);
    }
    _delay_ms(250);
    for(n=0;n<=8;n++){
        send_bits_595_LED( 0xff >> n );
        _delay_ms(30);
    }
    _delay_ms(50);
    for(n=0;n<2;n++){
        send_bits_595_LED(0xff);
        _delay_ms(75);
        send_bits_595_LED(0x00);
        _delay_ms(75);
    }
    
    // Clear Opening
	for(m=0;m<4;m++){
		lcd_locate(m,0);
		for(n=0;n<LCD_W;n++){
			lcd_set_char(0x20);
			_delay_ms(15);
		}
	}
    
    
    // Enable Interrupt
    sei();

    
	// Main function start this
	while(1){

        if(lcd_update){
            DisplayItemInfo();
            lcd_locate((chg_index&0x03)>>1,8-(1-chg_index%2));
            lcd_update = 0;
		}

        
        ////// Measure Sequence //////
        
        //// Defi Link Tap ////
		for ( index = 0; index < Ndata; index++ ){
			
			// Rx data read
			id = t_id[index];
            
			// Judge data validity
			for( n = 1; n < 4; n++ ){
                if( ( ( (data[index][n] >= '0') & (data[index][n] <= '9') )
                     |( (data[index][n] >= 'A') & (data[index][n] <= 'F') ) ) ){
					valid_packet[index] = 1;
				}else{
				 	valid_packet[index] = 0;
		 			break;
				}
			}
			// end of judge
            
			if ( valid_packet[index] == 1 ) {
				// Change char to angle-dec
                dec_ang = 0;
				for( n = 1; n < 4; n++){ // data[0] is neglected because of it is control data
					if  ( (data[index][n] & 0xf0) == 0x30 ){
						low4bits[n] = (unsigned int)(data[index][n] & 0x0f);
					}else if ( (data[index][n] & 0xf0) == 0x40 ){
						low4bits[n] = (unsigned int)(data[index][n] & 0x0f) + 9;
					}else{
						break;
					}
					dec_ang = dec_ang + low4bits[n] * mult_factor[3-n];
				}
				// end of Change char to angle-dec
                
                // Change angle-dec to normlized-dec
                dec_nrm = (float)dec_ang / maxv;
                // end of Change angle-dec to normlized-dec
                
				// Change dec to ISO
				value[index] = dec_nrm * eq_grad[id] + eq_intercept[id];
                // end of change
				
            }
		}
        
        rpm = TachoMeter();
        
        // Fuel Pump Driver
        if( rpm > 5000 ) FPDcomp = 0xff;
        else FPDcomp = rpm / 5000.0 * 0xff + 0x48;
        OCR0A = FPDcomp;

        
        
        ////// Display sequence //////
        if(    ( ( 0xffff - timer0_cnt_last > delay_cnt ) && ( timer0_cnt - timer0_cnt_last > delay_cnt ) )
            || ( ( 0xffff - timer0_cnt_last < delay_cnt ) && ( timer0_cnt + (0xffff - timer0_cnt_last) > delay_cnt ) ) ){
            timer0_cnt_last = timer0_cnt;
//            timer0_cnt = 0; //update 2014/11/19

            
            //// Defi Link Tap ////
            for ( index = 0; index < Ndata; index++ ){
                
                // Rx data read
                id = t_id[index];

                // clear value area of LCD
                lcd_locate(index,RxNameLength[id]);
                for (n=0;n<=(DISP_W-RxNameLength[id])-1;n++) {
                    lcd_set_char(' ');
                }
                // end of clear value
                
                // pad blank area of LCD
                lcd_locate(index,RxNameLength[id]);
                for (n=0;n<Nspace[id];n++){
                    lcd_set_char(' ');
                }
                // end of pad blank area of LCD
                
                if ( valid_packet[index] == 1 ) {
                    lcd_set_numeric(value[index],Nint[id],Nsig[id]-Nint[id],SIGN[id]);
                }else if( valid_packet[index] == 0 ){
                    for(n=0;n<(Nsig[id]!=Nint[id])+SIGN[id];n++){
                        lcd_set_char(' ');
                    }
                    for(n=0;n<Nsig[id];n++){
                        lcd_set_char('*');
                    }
                }

            }
            
            
            // Display Facemark
            lcd_locate(2,13);
            if((unsigned int)rpm < 3000){
                shobon();
            }else if((unsigned int)rpm < 5000){
                shakin();
            }else{
                kuwa();
                lcd_set_str("  ");
            }

    /*
            // Display FuelPump Duty
            lcd_locate(2,12);
            lcd_set_str("DUTY ");
            lcd_set_numeric((unsigned int)(FPDcomp*1.0/0xff*100),3,0,0);
    */
            
    /*
            // Display Bar Meter FuelPump Duty
            lcd_locate(3,12);
            BarMeter_disp((unsigned int)(FPDcomp*1.0/0xff*100));
    */
            
            // カウントデータデバグ用
    /*
            lcd_locate(1,0);
            for(n=0;n<3;n++){
                lcd_set_numeric(proc_array[n+0],4,0,0);
            }
            lcd_locate(2,0);
            for(n=0;n<3;n++){
                lcd_set_numeric(proc_array[n+3],4,0,0);
            }
            lcd_locate(3,0);
            for(n=0;n<3;n++){
                lcd_set_numeric(proc_array[n+6],4,0,0);
            }
    */
        
            // Update Indicator
            lcd_locate(2,12);
            lcd_set_char(0xff);
            
        }else{

            // Clear Update Indicator
            lcd_locate(2,12);
            lcd_set_char(' ');
            
        }
        
        //// Real-Time Update items
        
        // Display RPM
        lcd_locate(0,12);
        lcd_set_numeric((unsigned int)rpm,5,0,0);
        lcd_set_str("RPM");
        
        // Display Bar Meter RPM
        lcd_locate(3,12);
        BarMeter_disp((unsigned int)rpm);
        
        
	}

    return 0;
}
