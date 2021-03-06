//
//             Multi Function Meter
//                  Revision 1
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
#include "ExtInterrupt.h"
#include "usart.h"

// USART transmission speed definition
#define FOSC			8000000 // Clock Speed
#define BAUD			19200
#define UBRR			FOSC/16/BAUD-1

// Number of Display data
#define Ndata           4

// Wait time interval
#define	WAIT			10 // unit : ms

// LCD width
#define LCD_W           20

// Display character width of Defi Link Tap "name + data"
#define DISP_W          11

// Median Filter for Tacho Meter
#define Nmed            9

uint8_t     chg_index   = Ndata;
uint16_t    chg_count   = 0xffff;

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
void set_initial_t_id(){
    t_id[0] = 3;
    t_id[1] = 0;
    t_id[2] = 6;
    t_id[3] = 5;
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
unsigned long int   cnt;
double              freq;
unsigned long int   minv = 0xffff; // debug
unsigned long int   maxv = 0x0000; // debug
unsigned long int   rpm;
unsigned long int   meas_array[Nmed];
unsigned long int   proc_array[Nmed];
unsigned int        tacho_n = 0;

// 16-bit Counter intialazation
void timer1_init() {
	TCCR1A  = 0b00000000;	// Timer/Counter1 Control Register A
	TCCR1B  = 0b00000011;
	TCCR1C  = 0b00000000;
	TIMSK1  = 0b00000001;
	TCNT1   = 0x0000;         // Initialize 16-bit counter bit
	OCR1A   = 0xffff;         // 16-bit register for compare
}

// 8-bit PWM timer intialazation
void PWM_init() {
	TCCR0A  = 0b10000001;	// Timer/Counter1 Control Register A
	TCCR0B  = 0b00000101;
	TIMSK0  = 0b00000000;
	TCNT0   = 0x00;         // Initialize 8-bit counter bit
	OCR0A   = 0x80;         // 8-bit register for compare
}

// Count character length
int StrLength(const char *s){
    int n = 0;
    while (*s++ != '\0')
        n++;
    return (n);
}

// Bubble Sort (min->max)
void BubbleSort(){
	unsigned int tmp;
	unsigned int min;
	unsigned int min_index;
	unsigned int n,m;
	
	for(m=0;m<Nmed;m++){
		min = 0xffffffff;
		for(n=m;n<Nmed;n++){
			if( proc_array[n] < min ){
				min = proc_array[n];
				min_index = n;
			}
		}
		tmp                     = proc_array[m];
		proc_array[m]           = proc_array[min_index];
		proc_array[min_index]   = tmp;
	}
}

// Display
void DisplayItemInfo(void){
	int k;
	lcd_clear();
	for(k=0;k<Ndata;k++){
//		lcd_locate((unsigned int)((LCD_W/2*k)/LCD_W),(LCD_W/2)*(k%2));
		lcd_locate(k,0);
		lcd_set_str(RxName[t_id[k]]);
	}
	
}

ISR(USART_RX_vect){
    unsigned int    ID;
    unsigned int    n, m;

    ID = USART_receive(); // Synchronize & Detect receiver ID
    // Get 3-bit sngle data
    for(n=0;n<Ndata;n++){
        if ( ID == RxID[t_id[n]] ){
            for(m=0;m<4;m++) data[n][m] = USART_receive();
            break;
        }
    }
}

ISR(INT0_vect){
    meas_array[tacho_n] = TCNT1;
    if( tacho_n == Nmed-1 ) tacho_n=0;
    else                    tacho_n++;
    TCNT1 = 0;
}

// 16-bit Timer overflow
ISR(TIMER1_OVF_vect){
    meas_array[tacho_n] = 0xffffffff;
    if( tacho_n == Nmed-1 ) tacho_n=0;
    else                    tacho_n++;
    TCNT1 = 0;
}

// Bad ISR interrput detector for debugging
ISR(BADISR_vect){
    cli();    // これ以上割り込ませない
    lcd_locate(0,0);
    lcd_set_str("BADISR ERROR");
    while(1);  // ここで止めておく
}

int main(void)
{

    // for 74HC595 port setting
    SoftSPI_Init();
    
    // Initialize LCD
	lcd_init();
	

    // Interval Measure
    timer1_init();

	// USART initialize
	USARTinit(UBRR);
	
	// Ext. Interupt setting
	ExtInterrupt_init();
    
    // Bar-Meter Initialize
    BarMeter_init();
    // PWM putput port definition
    DDRD |= (1<<PD6);
    
    // PWM counter init
    PWM_init();
    
    set_initial_t_id();

	// Declarations
	unsigned char*   opening_message0 = "       M.F.M.       ";
    unsigned char*   opening_message1 = "     Revision 2     ";
    
	uint8_t         n, m;					// 'for' loop variables
    
    uint8_t        FPDcomp = 0xff;
    
	uint8_t         index = 0;				// LCD displaying data index
	
	uint16_t		maxv = 2352;			// maximum decimal angle data value from 'Defi Link Unit II'
    
	uint8_t         id;						// ID index for processing
    
	uint8_t         valid_packet;			// Validtity indicator
	
	uint8_t			low4bits[4];			// Extracted lower 4 bits from byte data
	uint16_t        dec_ang;				// Angle data (decimal)
	float           dec_nrm;				// Angle data (decimal)
	float			value;					// Decoded value
	uint8_t         value_sign;				// Sign of rounded value
	uint8_t         digits_int[5];			// Digits integer data
	unsigned char	digits_char[5];			// Digits character data for display
    
	uint16_t         mult_factor[3];			// Multiplying factor for hexadecimal to decimal decoding
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
    
	uint8_t	Nspace[7];				// Number of space between character and digits
    float   Resolution[7];
    
	RxName[0]		=   "Boost";
	RxName[1]		=   "Tacho";
	RxName[2]		=   "Oil.P";
	RxName[3]		=   "Fuel.P";
	RxName[4]		=   "ExTmp";
	RxName[5]		=   "Oil.T";
	RxName[6]		=   "Water.T";
    
	RxName_short[0]	=   "BS";
	RxName_short[1]	=   "TC";
	RxName_short[2]	=   "OP";
	RxName_short[3]	=   "FP";
	RxName_short[4]	=   "ET";
	RxName_short[5]	=   "OT";
	RxName_short[6]	=   "WT";
    
	RxName_long[0]	=   "Boost";
	RxName_long[1]	=   "Tacho";
	RxName_long[2]	=   "Oil press";
	RxName_long[3]	=   "Fuel press";
	RxName_long[4]	=   "Ext. Temp.";
	RxName_long[5]	=   "Oil Temp.";
	RxName_long[6]	=   "Water Temp.";
    
    // Definition of Resolution for processing and number of space for display
    for(n=0;n<7;n++){
        Resolution[n] = 1;
        for(m=0;m<Nsig[n]-Nint[n];m++){
            Resolution[n] = Resolution[n] / 10;
        }
        RxNameLength[n] = StrLength(RxName[n]);
        Nspace[n]       = DISP_W - RxNameLength[n] - ( Nsig[n] + (Nsig[n]!=Nint[n]) );
    }
    
	mult_factor[0] = 1;
	mult_factor[1] = 16;
	mult_factor[2] = 256;
    for(m=0;m<3;m++){
        for(n=0;n<=m;n++){
            
        }
    }

    // opening
	_delay_ms(50);
	lcd_locate(1,0);
	for(n=0;n<16;n++){
		lcd_set_char(opening_message0[n]);
		_delay_ms(20);
	}
	lcd_locate(2,0);
	for(n=0;n<16;n++){
		lcd_set_char(opening_message1[n]);
		_delay_ms(20);
	}
    
    // Clear Opening
	_delay_ms(300);
	for(m=0;m<4;m++){
		lcd_locate(m,0);
		for(n=0;n<16;n++){
			lcd_set_char(0x20);
			_delay_ms(20);
		}
	}
    
    sei();

	// Main function start this
	while(1){

        if(lcd_update){
			DisplayItemInfo();
            lcd_locate((chg_index&0x03)>>1,8-(1-chg_index%2));
			lcd_update = 0;
		}

        
        // Defi Link Tap
		for ( index = 0; index < Ndata; index++ ){
			
			// Rx data read
			id = t_id[index];
            
			// Judge data validity
			for( n = 1; n < 4; n++ ){
				if( ( ( (data[index][n] >= '0') & (data[index][n] <= '9') )
                     |( (data[index][n] >= 'A') & (data[index][n] <= 'F') ) ) ){
					valid_packet = 1;
				}else{
					valid_packet = 0;
					break;
				}
			}
			// end of judge
            
            // clear value area of LCD
            //				lcd_locate((uint8_t)((LCD_W/2*index)/LCD_W),(LCD_W/2)*(index%2)+RxNameLength[id]);
            lcd_locate(index,RxNameLength[id]);
            for (n=0;n<=(DISP_W-RxNameLength[id])-1;n++) {
                lcd_set_char(' ');
            }
            // end of clear value
            
            // pad blank area of LCD
            lcd_locate(index,RxNameLength[id]);
            for (n=0;n<Nspace[id]-1;n++){
                lcd_set_char(' ');
            }
            // end of pad blank area of LCD
			if ( valid_packet == 1 ) {
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
				value = dec_nrm * eq_grad[id] + eq_intercept[id];
                // end of change
				
				// Detect sign
                if( value < 0 ){
					value_sign = 1;
					value = -value;
				}else{
					value_sign = 0;
				}
				// end of
				
                lcd_set_numeric(value,Nint[id],Nsig[id]-Nint[id]);
/*
                // Divide from integer to each digits
				for (n=0;n<=Nsig[id]-1;n++) {
					div_factor = Resolution[id];
					for (m=0;m<Nsig[id]-1-n;m++) {
						div_factor = div_factor * 10;
					}
					digits_int[n] = (uint8_t)(value / div_factor);
					digits_char[n] = (uint8_t)digits_int[n] + 0x30;
					value = value - div_factor * digits_int[n];
				}
				// end of

                // Display value to LCD
				if( !value_sign )	lcd_set_char(' ');
				else				lcd_set_char('-');
				digits_valid = 0;
				for (n=0;n<=Nsig[id]-1;n++) {
					if( n == Nint[id] ) lcd_set_char('.');
					if( !digits_valid && (digits_char[n]==0x30) && ( n+1 < Nint[id])){
						lcd_set_char(' ');
					}else{
						lcd_set_char(digits_char[n]);
						digits_valid = 1;
					}
				}
				// end of
 */
				
				_delay_ms((unsigned int)(WAIT));
                
			}else if( valid_packet == 0 ){
                for (n=0;n<=Nsig[id]-1;n++){
                    lcd_set_char('-');
                }
            }
		}
        
        // Tacho Meter
        for(n=0;n<Nmed;n++){
            proc_array[n] = meas_array[n];
        }
        BubbleSort();

/*
        for(n=0;n<Nmed;n++){ //debug
            lcd_locate((int)(n/3)+1,6*(n%3)); // debug
            lcd_set_numeric((unsigned int)proc_array[n],5); // debug
        }
*/
        if(proc_array[Nmed-1] >= 0xffff){
            freq = 0;
        }else{
            cnt = proc_array[Nmed>>1];
            freq = 8000000.0 / 64.0 / cnt;
            //     ^^^^^^^^^   ^^^^
            //       fosc      Ndiv
        }
        
        rpm = (unsigned long int)( 60.0 * freq );

        // Fuel Pump Driver
//        if( rpm > 3200 ) FPDcomzp = 0xff;
//        else FPDcomp = rpm / 3200.0 * 0xff + 0x48;
//        OCR0A = FPDcomp;
        OCR0A = 0x01;
/*
        // Display Freq
        lcd_locate(1,0); // debug
        lcd_set_numeric((unsigned int)freq,4); // debug
        lcd_set_str("Hz"); // debug
*/
        // Display RPM
        lcd_locate(0,12);
        lcd_set_numeric((unsigned int)rpm,5,0);
        lcd_set_str("rpm");

/*
        lcd_locate(0,0); // debug
        lcd_set_numeric((unsigned int)cnt,7); // debug
        
        lcd_locate(2,12);
        lcd_set_numeric((unsigned int)FPDcomp,8);
 */
        lcd_locate(3,12);
        BarMeter_disp((unsigned int)rpm);
        
//        _delay_ms(10);

	}
 
    return 0;
}
