#include <MSP430.h>
#include <stdint.h>
#include <stdbool.h>
bool fg,fg1,cmfg;
void extract_MSB_byte(uint16_t);
void sendTomaster(uint8_t);
uint16_t RCV_Data = 0x0000, MSB_byte= 0x0000, LSB_byte = 0x0000;
uint16_t MSB_byte_temp = 0x0000;
uint8_t SP_CMD = 0xA5;
int x=0;
void main (void)
 { 
	 WDTCTL = WDTPW + WDTHOLD;
	 if(CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xFF)
	 {
		 while(1);
	 }
	 BCSCTL1 = CALBC1_1MHZ;
	 DCOCTL = CALDCO_1MHZ;
	 ///*7 Segment Display*///
	 P7DIR |= 0xFF;
	 P8DIR |= 0x0F;
	 P1DIR |= BIT0;
	 ///*7 Segment Display*///
	 
	 ///*Timer Configuration Start*///
	 TACCR0 = 1500; //UPPER LEIMIT 
	 TACTL = MC_1 | ID_0 | TASSEL_2 | TACLR; // SET UP AND TIMER START A
	 __enable_interrupt ();
	 ///*Timer Configuration End*///
	 
	 ///*UART Configrations Start*///
	 P3SEL = 0x30;
	 UCA0CTL1 |= UCSSEL_2;
	 UCA0BR0 = 8;
	 UCA0BR1 = 0;
	 UCA0MCTL = UCBRS2 | UCBRS0;
	 UCA0CTL1 &= ~UCSWRST;
	 IE2 |= UCA0RXIE | UCA0TXIE;
	 ///*UART Configrations End*///
	 __bis_SR_register(CPUOFF + GIE);
	 return 0;
 }
  #pragma vector=USCIAB0RX_VECTOR
 __interrupt void USCI0RX_ISR(void)
 {
	 while(!(IFG2 & UCA0RXIFG));
	 MSB_byte_temp = UCA0RXBUF;
	 extract_MSB_byte(MSB_byte_temp);
	 while(!(IFG2 & UCA0RXIFG));
	 LSB_byte = UCA0RXBUF;
	 P8OUT = MSB_byte_temp;
	 P7OUT = LSB_byte;
	 RCV_Data = ((MSB_byte | LSB_byte) & 0x0FFF);
	 if( RCV_Data < 616)
	 {
		 if( fg == true )
		 {
			 sendTomaster(SP_CMD);
			 fg = false;
			 cmfg = true;
		 }
	 }
	 if( RCV_Data >= 616)
	 {
		 fg = true;
		 if( cmfg == true){
		 sendTomaster(0);
			 cmfg = false;
		 }
	 }
	 if( RCV_Data == 1859)
	 {
		 if(fg1 == true)
		 {
			 //P1OUT |= BIT0;
			 TACCTL0 = CCIE; //ENABLE INTERRUPT ON COMPARE 0
			 fg1 = false;
			 
		 }
	 }
	 if( (RCV_Data < 1859))
	 {
		 fg1 = true;
		 P1OUT &= ~BIT0;
	     TACTL = MC_1 | ID_0 | TASSEL_2 | TACLR; // SET UP AND TIMER START A
		 TACCTL0 = ~CCIE; //ENABLE INTERRUPT ON COMPARE 0
	     __enable_interrupt ();
	 }
 }
 #pragma vector=USCIAB0TX_VECTOR
 __interrupt void USCI0TX_ISR(void)
 { 
	 while(!(IFG2 & UCA0TXIFG));
 }
 void extract_MSB_byte(uint16_t byte)
 {
	 MSB_byte = ((MSB_byte_temp<<8) & 0x0F00);
 }
 void sendTomaster(uint8_t cmd)
 {
	 UCA0TXBUF = cmd; 
 }
 #pragma vector = TIMERA0_VECTOR
 __interrupt void TA0_ISR(void)
 {
	 P1OUT ^= BIT0;
	 if(TACCR0 == 1500)
	 {
		 TACCR0 = 500;
	 }
	 else
	 {
		 TACCR0 = 1500;
	 }
 }