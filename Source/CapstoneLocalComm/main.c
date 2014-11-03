#include <msp430.h> 

/*
 * main.c
 */

#define BUTTON_0 BIT0 // P2
#define BUTTON_1 BIT1 // P2
#define LED_1 BIT5 // P1
#define LED_2 BIT5 // P2
#define READ BIT4 // P2
#define WRITE BIT3 //P2
#define WRITE_EN BIT4 //P1

#define INIT_COMM_0 1
#define INIT_COMM_1 0

int id = 1;
int write_1[] = {INIT_COMM_0, INIT_COMM_1, 0, 1};
int write_2[] = {INIT_COMM_0, INIT_COMM_1, 1, 0};
int* write;
int writeCount = 4;

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    _BIS_SR(GIE);

    TACCR0 = 0x1000;			// 2^12 ticks of the ACLK/8 should be one second
    TACCTL0 = CCIE | CM_0;		// capture/compare interrupt enable; CM0 means no capture
    TACTL = TASSEL_1 | ID_3 | TACLR | MC_1;		// Timer A source is ACLK/8, it runs in up mode, start by clearing it
	
    P2REN |= BUTTON_0 | BUTTON_1 | READ;
    P2OUT |= BUTTON_0 | BUTTON_1 | READ;
    P2DIR &= ~(BUTTON_0 | BUTTON_1 | READ);

    P1DIR |= LED_1 | WRITE_EN;
    P2DIR |= LED_2 | WRITE;

    while(1) {
    	if(P2OUT & BUTTON_0 & (writeCount == 4)) {
    		write = write_1;
    		writeCount = 0;
    		P1OUT &= ~WRITE_EN;
    	}
    	if(P2OUT & BUTTON_1 & (writeCount == 4)) {
    		write = write_2;
    		writeCount = 0;
    		P1OUT &= ~WRITE_EN;
    	}
    	if(writeCount == 4) {
    		P1OUT |= WRITE_EN;
    	}
    }

	return 0;
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TimerA0_routine (void) {
	// Handle the interrupt
	doRead();
	doWrite();
}

int initRead1 = 0;
int initRead2 = 0;
int valSet = 0;
int valRead = 0;
void doRead() {
	if(initRead1 && initRead2) {
		if(P2IN & READ) {
			valRead = 1;
		}
		else {
			valRead = 0;
		}
		valSet++;
	}
	else if(initRead1) {
		if(P2IN & READ) {
			initRead1 = 0;
		}
		else {
			initRead2 = 1;
		}
		valSet = 0;
	}
	else {
		if(P2IN & READ) {
			initRead1 = 1;
		}
		valSet = 0;
	}
}

void handleRead() {
	if((valSet == 1) && valRead) {
		P1OUT |= LED_1;
	}
	else {
		P1OUT &= ~LED_1;
	}

	if((valSet == 2) && valRead) {
		P2OUT |= LED_2;
	}
	else {
		P2OUT &= ~LED_2;
	}
}

void doWrite() {
	if(writeCount < 4) {
		if(write[writeCount]) {
			P2OUT |= WRITE;
		}
		else {
			P2OUT &= ~WRITE;
		}
		writeCount++;
	}
}
