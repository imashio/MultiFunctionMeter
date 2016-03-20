// Liquid Crystal Display Header File

void SoftSPI_Init(void);
void SoftSPI_TX(unsigned char);
void send_bits_595(unsigned char, unsigned char, unsigned char);

void lcd_init(void);
void lcd_locate(unsigned char, unsigned char);

void lcd_set_4bit(unsigned char);
void lcd_set_char(unsigned char);
void lcd_set_str(unsigned char *);
void lcd_set_numeric(unsigned int, unsigned int);
void lcd_clear(void);
void lcd_set_CGRAMaddr(unsigned char,unsigned char);

void BarMeter_disp(unsigned int);
void BarMeter_init(void);