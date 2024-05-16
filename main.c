// Design Project - PID Temperature Controller

/*
A temperature controller program that interfaces with a GUI interface built in
MATLAB over UART to adjust the temperature of a Peltier module to a setpoint.
It uses PID to reach the setpoint quickly and relies on the L298N H-Bridge IC
for driving the module, with the code running on an MSP430G2553 Microcontroller
from the TI Launchpad Development Kit.
*/

#include <msp430.h>
#include <string.h>

// Constant definitions
#define RXD (BIT1)
#define TXD (BIT2)
#define ADC (BIT4)
#define PWM_1 (BIT6)
#define PWM_2 (BIT7)
#define vPOINTS 30

/**
 * main.c
 */

unsigned volatile char tecVoltage[vPOINTS];	   // Array of voltage points read from thermistor as 8-bit value
unsigned volatile char bufferVoltage[vPOINTS]; // Buffer array of voltage points read from thermistor as 8-bit value
signed volatile char PIDOutput;				   // 8-bit signed output from PID algorithm implemented in MATLAB

unsigned int resetTime = 1; // Pulse period in milliseconds

/*
Description: MSP430 Hardware Initialization Subroutine
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void initMSP(void)
{
	WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

	// Set clock to 16 MHz
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
}

/*
Description: ADC10 Module Initialization Subroutine
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void initADC(void)
{
	ADC10CTL1 |= INCH_4 | CONSEQ_2;							 // Use CONSEQ_2 (Mode 2 - Repeat single channel) and set ADC10 on P1.4
	ADC10AE0 |= ADC;										 // Enable ADC10 on P1.4
	ADC10CTL0 |= ADC10SHT_0 | MSC | ADC10ON | ADC10SC | ENC; // Select 4 ADC10CLK cycles sample-and-hold time, multiple sample and conversion (MSC), turn on ADC10, start and enable conversion
}

/*
Description: UART Serial Interface Initialization Subroutine
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void initUART(void)
{
	// initialize the USCI to RXD on P1.1 & TXD on P1.2
	P1SEL |= RXD | TXD;
	P1SEL2 |= RXD | TXD;

	// Set communication data rate to 115200 baud
	UCA0BR1 = 0;
	UCA0BR0 = 138;

	UCA0CTL1 |= UCSSEL_3; // Set clock to SMCLK
	UCA0CTL1 &= ~UCSWRST; // Release UART RESET

	IE2 |= UCA0RXIE; // Set UCA0RXIE high to enable interrupt request of UCA0RXIFG
}

/*
Description: PWM Timer Initialization Subroutine
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void initPWM(void)
{
	P1DIR |= PWM_1; // P1.6 is output (Heating)
	P2DIR |= PWM_1; // P2.6 is output (Cooling)
	P1OUT = 0;		// Set all outputs to 0
	P2OUT = 0;		// Set all outputs to 0

	TACCTL1 |= OUTMOD_3; // Set PWM mode to set/reset

	TACTL |= TASSEL_2 | ID_2 | MC_1; // Set Timer A to SMCLK, scale factor 2, count up to TACCR0
	TACCR0 = resetTime * 4000 - 1;	 // Configured for 16 MHz & scale factor 2
}

/*
Description: Get 8-bit Data over UART
Inputs:      (unsigned char)UCA0RXBUF
Outputs:     void
Parameters:  void
Returns:     (unsigned char)data
*/
unsigned char getData(void)
{
	signed char data = UCA0RXBUF; // Fetch data as 8-bit char

	return data;
}

/*
Description: Send 30 8-bit Datapoints over UART
Inputs:      (unsigned int[])tecVoltage
Outputs:     (unsigned char)UCA0TXBUF
Parameters:  void
Returns:     void
*/
void sendData(void)
{
	volatile unsigned int i; // Initialize counter for loop

	for (i = 0; i < vPOINTS; i++)
	{
		while (!(IFG2 & UCA0TXIFG))
			; // Wait if the transmitter has not completed the last transmission

		UCA0TXBUF = tecVoltage[i]; // Send each of last 30 datapoints as 8-bit char
	}
}

/*
Description: Sample ADC and Return 30 8-bit Voltage Values
Inputs:      (unsigned int)ADC10MEM
Outputs:     void
Parameters:  void
Returns:     (unsigned int[])bufferVoltage
*/
void sample(void)
{
	volatile unsigned int i; // Initialize counter for loop

	for (i = 0; i < vPOINTS; i++)
	{
		bufferVoltage[i] = (float)ADC10MEM / 1023.0 * 255.0 + 0.5; // Sample ADC 30 times, convert from 10-bit to 8-bit, round float to int, and add to buffer array
	}
}

/*
Description: Update timers to adjust PWM duty cycle and pin to maintain set temperature.
Inputs:      void
Outputs:     (float)TACCR1
Parameters:  (signed char)inputPWM
Returns:     void
*/
void updatePWM(signed char inputPWM)
{
	float dutyCycle = abs(inputPWM) / 127.0;	 // Duty cycle to write to PWM pins
	float setTime = resetTime * (1 - dutyCycle); // Time low in milliseconds

	if (inputPWM > 0) // Heating mode
	{
		// Set registers to enable PWM for P1.6
		P1SEL |= PWM_1;
		P1SEL2 &= ~PWM_1;

		// Clear registers to disable PWM for P2.6
		P2SEL &= ~(PWM_1 | PWM_2);
		P2SEL2 &= ~(PWM_1 | PWM_2);
	}
	else // Cooling mode
	{
		// Set registers to enable PWM for P2.6
		P2SEL |= PWM_1;
		P2SEL &= ~PWM_2;
		P2SEL2 &= ~(PWM_1 | PWM_2);

		// Clear registers to disable PWM for P1.6
		P1SEL &= ~PWM_1;
		P1SEL2 &= ~PWM_1;
	}

	TACCR1 = setTime * 4000 - 1; // Configured for 16 MHz & scale factor 2
}

#pragma vector = USCIAB0RX_VECTOR // Set pragma vector to serial data recieved vector

/*
Description: Serial Data Recieved Interrupt Subroutine
Inputs:      (unsigned char)dutyCycle
Outputs:     (unsigned char)tecVoltage & (unsigned char)dutyCycle
Parameters:  void
Returns:     void
*/
__interrupt void serialInterrupt(void)
{
	PIDOutput = getData(); // Fetch PWM duty cycle data from recieve buffer
	sendData();			   // Send last voltage reading datset over UART
	updatePWM(PIDOutput);  // Update PWM duty cycle on timers & pins to heat/cool
}

/*
Description: Main Subroutine
Inputs:      void
Outputs:     void
Parameters:  void
Returns:     void
*/
void main(void)
{
	initMSP();	// Calling the initMSP() function
	initUART(); // Calling the initUART() function
	initADC();	// Calling the initADC() function
	initPWM();	// Calling the initPWM() function

	while (!(IFG2 & UCA0RXIFG))
		;
	getData(); // Wait to receive a one-time transmission from MATLAB to begin

	__bis_SR_register(GIE); // Enable interrupts

	while (1)
	{
		sample();									  // Sample data continuously
		memcpy(&tecVoltage, &bufferVoltage, vPOINTS); // When done collecting, empty buffer into tecVoltage array to be sent to MATLAB
	}
}
