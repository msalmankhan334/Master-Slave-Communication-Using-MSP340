#include <MSP430.h>
#include <stdint.h>
void arrang_data(uint16_t);
void sendToslave(uint16_t);
uint16_t memResult = 0x0000, MSB_byte= 0x0000, LSB_byte = 0x0000;
uint8_t sp = 0x00,sp_cmd = 0xA5;
int x;
void main (void)
 { 
	 WDTCTL = WDTPW + WDTHOLD;
	 if(CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xFF)
	 {
		 while(1);
	 }
	 BCSCTL1 = CALBC1_1MHZ;
	 DCOCTL = CALDCO_1MHZ;
	 ////*LED BLINKING8///
	 P1DIR |= BIT0;
	 ////*LED BLINKING8///
	 ///*Timer Configuration Start*///
	 TACCR0 = 20000; //UPPER LEIMIT 
	 TACTL = MC_1 | ID_0 | TASSEL_2 | TACLR; // SET UP AND TIMER START A
	 //__enable_interrupt ();
	 ///*Timer Configuration End*///
	 ///*ADC Configuration Start**///
	 P6SEL |= BIT0;
	 ADC12CTL0 = SHT0_2 | ADC12ON | ENC;
	 ADC12CTL1 = SHP;
	 ADC12IE = BIT0;
	 ///*ADC Configuration End**///
	 
	 ///*UART Configrations Start*///
	 P3SEL = 0x30;
	 UCA0CTL1 |= UCSSEL_2;
	 UCA0BR0 = 8;
	 UCA0BR1 = 0;
	 UCA0MCTL = UCBRS2 | UCBRS0;
	 UCA0CTL1 &= ~UCSWRST;
	 IE2 |= UCA0RXIE | UCA0TXIE;
	 ///*UART Configrations End*/// 
	 while(1)
	 {
		 int i;
		 for(i = 5000; i>0; i--);
		 ADC12CTL0 |= (ADC12SC);
		  __bis_SR_register(CPUOFF + GIE);
	 }
 }   
#pragma vector = ADC12_VECTOR
 __interrupt void  adc12_isr(void)
 {
	 while(ADC12CTL0 & ADC12BUSY);
	 memResult = ADC12MEM0;
	 arrang_data(memResult);
	 sendToslave(MSB_byte);
	 __delay_cycles(100);
	 sendToslave(LSB_byte);
	 __delay_cycles(100);
	 __bic_SR_register_on_exit(CPUOFF);
 }
 #pragma vector=USCIAB0RX_VECTOR
 __interrupt void USCI0RX_ISR(void)
 {
	 __delay_cycles(10000);
	 while(!(IFG2 & UCA0RXIFG));
	 sp = UCA0RXBUF;
	 if(sp == sp_cmd)
	 {
		 TACCTL0 = CCIE;
	 }
	 else
	 {
		 P1OUT &= ~BIT0;
		 TACCTL0 = ~CCIE; //ENABLE INTERRUPT ON COMPARE 0
	 }
 }
 #pragma vector=USCIAB0TX_VECTOR
 __interrupt void USCI0TX_ISR(void)
 {
	 while(!(IFG2 & UCA0TXIFG));
	 asm("nop");
 }
 void arrang_data(uint16_t data)
 {
	 asm("nop");
	 MSB_byte = ((data>>8) & 0x000F);
	 LSB_byte = (data & 0x00FF);
 }
 void sendToslave(uint16_t chunk)
 {
	 UCA0TXBUF = chunk;
 }
#pragma vector = TIMERA0_VECTOR
 __interrupt void TA0_ISR(void)
 {
	 x++;
	 if( x == 10)
	 {
		 P1OUT ^= BIT0;
		 x =0;
	 }
 }