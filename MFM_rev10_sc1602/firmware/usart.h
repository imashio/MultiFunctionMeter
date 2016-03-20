// USART initialize
void USARTinit(unsigned int ubrr);

// USART transmit
void USART_transmit(unsigned char data);
void USART_transmit_str(unsigned char *s);
void USART_transmit_numeric(float num, unsigned int Nint, unsigned int Nfrac, unsigned int sign);


// USART receive
unsigned char USART_receive(void);
unsigned char USART_receive_wTO(void);