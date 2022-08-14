/*
 * MiniProject2.c
 *
 *  Created on: Sep 15, 2021
 *      Author: Kirollos Medhat
 */


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define Number_of_seconds_in_minutes 60 //defining number of interrupts at which minutes variable is counted
#define Number_of_minutes_in_hours 60 //defining number of interrupts at which hours variable is counted
unsigned char seconds =0; //defining variables which will be showed on the segments
unsigned char minutes =0;
unsigned char hours =0;

void TIMER1_CTC_Init(void){
	TCCR1B =  (1<<WGM12) | (1<<CS10) | (1<<CS12) | (1<<FOC1B);
	/* Configure timer 1  channel B control register
	 * 1. Non PWM mode FOC1B=1
	 * 2. CTC Mode WGM12=1 & WGM11=0 & WGM10=0
	 * 3. clock prescaler 1024 = CPU clock CS10=1 CS11=0 CS12=1
	 */
	OCR1A = 975; //Configure the timer 1 channel b to flag interrupt every approx 1 s
	TCNT1 = 0; //reseting the counter to count from 0
	TIMSK|=(1<<OCIE1A); //enabling the timer 1 compare mode

}
void Reset_Timer_init(void){
	MCUCR |= (1<<ISC01);
	GICR |= (1<<INT0);
	DDRD &= ~(1<<PD2);
	PORTD |= (1<<PD2);
	/* Configure INT0 for reset
	 * 1.Setting ISC01 = 1 , ISC00 = 0 for falling edge INT0.
	 * 2. Enabling external int0 module GICR setting INT0 =1 and other bits with 0
	 * 3. enabling the button as input by clearing its bit and enabling the internal pull up resistor by setting its bit by 1
	 */
}
void Pause_interrupt_init(void){
	MCUCR |= (1<<ISC10) | (1<<ISC11);
	GICR |= (1<<INT1);
	DDRD &= ~(1<<PD3);
	/* Configure INT1 for pause
	 * 1.Setting ISC01 = 1 , ISC00 = 1 for rising edge INT1 .
	 * 2. Enabling external for INT1 general module GICR setting INT1 =1 and other bits with 0
	 * 3. enabling the button PD3 as input by clearing its bit
	 */
}
void Resume_interrupt_init(void){
	MCUCR |= (1<<ISC01);
	GICR |= (1<< INT2);
	DDRB &= ~(1<<PB2);
	PORTB |= (1<<PB2);
	/* Configure INT2 for resume
	 * 1.Setting ISC01 = 1 , ISC00 = 0 for falling edge INT2 .
	 * 2. Enabling external for INT2 general module GICR setting INT2 =1 and other bits with 0
	 * 3. enabling the button PD3 as input by clearing its bit and enabling internal pull up resistor for PB2
	 */
}
//No need for interrupt nesting as every button for every task does a different task which doesn't interfere the current task
ISR(TIMER1_COMPA_vect){
	//marking interrupt increment variable seconds to create 1 second at every interrupt
	//making variables for minutes and hours by the needed number of seconds in minute and needed varibale minutes in hours
	seconds++;
	if(seconds == Number_of_seconds_in_minutes){
		seconds =0;
		minutes++;
	}
	if(minutes == Number_of_minutes_in_hours){
		minutes =0;
		seconds =0;
		hours++;
	}
	if (hours == 12){
		seconds = 0;
		minutes = 0;
		hours = 0 ;
	}
}
ISR(INT0_vect){ //reseting the interrupts and clock for reseting the timer
	TCNT1 = 0;
	seconds = 0;
	minutes = 0;
	hours = 0 ;
}
ISR(INT1_vect){ //Disabling the clock for pausing the timer from incrementing
	TCCR1B &= ~(1<<CS10) &~(1<<CS12);
}
ISR(INT2_vect){ //Enabling the clock with a compare mode to resume
	TCCR1B =  (1<<WGM12) | (1<<CS10) | (1<<CS12) | (1<<FOC1B);
}

int main(void){
	SREG |= (1<<7); //enabling the global interrupt in main as we need it always on
	DDRA &= ~0x3F; //Choosing the first 6 pins of port A as inputs
	DDRC |= 0x0F; //making the first 4 pins of portc C as outputs for segments decoder
	PORTA |= 0x3F; //initial value for PORTA pins with 0
	PORTC &= ~0x0F; //initial value 0 for segments
	TIMER1_CTC_Init(); //calling the initializations for timer and interrupts
	Reset_Timer_init();
	Pause_interrupt_init();
	Resume_interrupt_init();
	while(1){
		PORTA = (1<<PA5);
		PORTC = (PORTC & 0xF0) |((seconds) % 10);
		_delay_ms(5);
		PORTA = (1<<PA4);
		PORTC = (PORTC & 0xF0) |((seconds) / 10);
		_delay_ms(5);
		PORTA = (1<<PA3);
		PORTC = (PORTC & 0xF0) |((minutes) % 10);
		_delay_ms(5);
		PORTA = (1<<PA2);
		PORTC = (PORTC & 0xF0) |((minutes) / 10);
		_delay_ms(5);
		PORTA = (1<<PA1);
		PORTC = (PORTC & 0xF0) |((hours) % 10);
		_delay_ms(5);
		PORTA = (1<<PA0);
		PORTC = (PORTC & 0xF0) |((hours) / 10);
		_delay_ms(5);
		/*using technique that gets a 2 digit number and divides
 it by 10 to take the reminder in the first part segment
  for the digit and the tens part is floored as it is integer
  to the second part of segments to show the 2 digit number on two parts */

		/*Using multiplexing delay 5 ms at which all the portc is on with all the leds
 but every port has a value to write which enables by setting the pin in portA
 with delay that cannot be seen with eyes
		 */
	}
	return 0;
}
