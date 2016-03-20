// Liquid Crystal Display Header File

void lcd_init(void);
void lcd_locate(unsigned char, unsigned char);

void lcd_set_4bit(unsigned char);
void lcd_set_char(unsigned char);
void lcd_set_str(unsigned char *);
void lcd_set_numeric(float, unsigned int, unsigned int, unsigned int);
void lcd_clear(void);
void lcd_set_CGRAMaddr(unsigned char,unsigned char);