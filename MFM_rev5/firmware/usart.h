// USART initialize
void USARTinit(unsigned int ubrr);

// USART transmit
void USART_transmit(unsigned char data);

// USART receive
unsigned char USART_receive(void);
unsigned char USART_receive_woTO(void);
unsigned char USART_receive_wTO(void);