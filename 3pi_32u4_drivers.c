/*
 * 3pi_32u4_drivers.c
 *
 * Created: 1/16/2023 1:15:55 PM
 *  Author: Douglas Summerville
 * Updated: Meg Jain, 3/11/2024
 */
/**********************************************************************************
 *Start of Robot 3pi AVR library provided to students
 *
 */



#include "3pi_32u4_drivers.h"
#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>

#define F_CPU 16000000

#include <util/delay.h>
#include <avr/interrupt.h>

/*
 * All push buttons are shared with other functions.  To use them, the
 * other function needs to be temporarily disconnected, the button set up,
 * a small delay to allow the input to charge, then look at the button.
 * the pin is restored to its previous state afterwards
 */

_Bool button_a_is_up()
{
	uint8_t save_ddr=DDRB;
	uint8_t save_port=PORTB;
	_Bool retval;
	DDRB &= ~(1<<3);
	PORTB |= (1<<3); //pull-up
	_delay_us(5);
	retval = PINB & (1<<3);
	PORTB = save_port;
	DDRB = save_ddr;
	return retval;
}

_Bool button_a_is_down()
{
	return !button_a_is_up();
}


_Bool button_b_is_up()
{
	uint8_t save_ddr=DDRD;
	uint8_t save_port=PORTD;
	_Bool retval;
	DDRD &= ~(1<<5);
	PORTD |= (1<<5); //pull-up
	_delay_us(5);
	retval = PIND & (1<<5);
	PORTD = save_port;
	DDRD = save_ddr;
	return retval;
}

_Bool button_b_is_down()
{
	return !button_b_is_up();
}


_Bool button_c_is_up()
{
	uint8_t save_ddr=DDRB;
	uint8_t save_port=PORTB;
	_Bool retval;
	DDRB &= ~(1<<0);
	PORTB |= (1<<0); //pull-up
	_delay_us(5);
	retval = PINB & (1<<0);
	PORTB = save_port;
	DDRB = save_ddr;
	return retval;
}

_Bool button_c_is_down()
{
	return !button_c_is_up();
}

/*starts timer 1 for PWM on the motors;
 *students need to DDR PB1,2,5,6
 *students set duty cucle in OC1A for right nd OC1B for left
 *duty cycle is up to 255 but using 0-100 slows robot
 */
static void configure_motor_timer()
{
	ICR1=256; //TOP=256; students use 0-100 to slow motors
	TCCR1A=0xA2; //fast pwm
	TCCR1B=0x1A; //prescale=8 ~8kHz
}

/* TIMER4 IRQ 1 us
 */
static uint32_t stopwatch_time;
static uint8_t stopwatch_units;
static uint8_t line_sensor_counter;
uint8_t line_sensor[5];
uint8_t next_line_sensor[5];
static void configure_timer4()
{
	OCR4C = 9;
	TIMSK4 = (1<<2);
	TCCR4B = 0x05;
}
ISR(TIMER4_OVF_vect)
{
	stopwatch_units++;
	if( stopwatch_units >= 100){
		stopwatch_time++;
		stopwatch_units=0;
	}
	if( line_sensor_counter == 0){
		for( uint8_t i=0; i<5; i++){
			line_sensor[i]=next_line_sensor[i];
			next_line_sensor[i]=0;
		}
		DDRD|=(1<<6);
		PORTD|=(1<<6);
		DDRF |= 0xb2;
		PORTF|=0xb2;
	}
	else if(line_sensor_counter == 1){
		DDRD&=~(1<<6);
		PORTD&=~(1<<6);
		DDRF &=~0xb2;
		PORTF&=~0xb2;
	}
	else{
			if( (PIND & (1<<6 )) )
				next_line_sensor[4]=line_sensor_counter;
			if( (PINF & (1<<7 )) )
				next_line_sensor[3]=line_sensor_counter;
			if( (PINF & (1<<5 )) )
				next_line_sensor[2]=line_sensor_counter;
			if( (PINF & (1<<4 )) )
				next_line_sensor[1]=line_sensor_counter;
			if( (PINF & (1<<1 )) )
				next_line_sensor[0]=line_sensor_counter;
	}
	line_sensor_counter++;
}
void start_stopwatch()
{
	stopwatch_time=0;
	stopwatch_units=0;
}
uint32_t read_stopwatch(){
	return stopwatch_time;
}


static uint32_t left_quad_cntr, right_quad_cntr;
static void configure_quadrature_counters()
{
	EICRB=0x10; //IRQ on both edges
	EIMSK=0x40; //Enable INT6 IRQ for right encoder
    PCICR = (1 << PCIE0);
    PCMSK0 = (1 << PCINT4);
    PCIFR = (1 << PCIF0);
}
uint32_t get_right_quadrature_counter()
{
	return right_quad_cntr;
}
uint32_t get_left_quadrature_counter()
{
	return left_quad_cntr;
}

ISR(INT6_vect)
{
	right_quad_cntr++;
}
ISR(PCINT0_vect)
{
	if( PCMSK0 & (1<<4))
		left_quad_cntr++;
}

#define DB0_PORT B
#define DB0_LOC 3
#define DB1_PORT B
#define DB1_LOC 0
#define DB2_PORT C
#define DB2_LOC 7
#define DB3_PORT D
#define DB3_LOC 5
#define RS_PORT D
#define RS_LOC 2
#define E_PORT D
#define E_LOC 3
#define BF_LOC 7

#define CONCATENATE(A,B) A##B
#define PORT( letter) CONCATENATE(PORT,letter)
#define DDR( letter) CONCATENATE(DDR,letter)
#define PIN(letter) CONCATENATE(PIN,letter)

//static void set_data_bus_dir_to_output();
static void set_data_bus_dir_to_input();
static void output_nibble_to_data_bus(uint8_t data);
static void pulse_E();
static void nibble_out(uint8_t data);
static void send(unsigned char data, unsigned char rs);
static void send_command(unsigned char cmd);
static void send_data(unsigned char data);

static inline void config_DB0_as_output(){ DDR(DB0_PORT) |= (1<<DB0_LOC); }
static inline void config_DB1_as_output(){ DDR(DB1_PORT) |= (1<<DB1_LOC); }
static inline void config_DB2_as_output(){ DDR(DB2_PORT) |= (1<<DB2_LOC); }
static inline void config_DB3_as_output(){ DDR(DB3_PORT) |= (1<<DB3_LOC); }

static inline void config_RS_as_output() { DDR(RS_PORT)  |= (1<<RS_LOC); }
//static inline void config_RW_as_output() { DDR(RW_PORT)  |= (1<<RW_LOC); }
static inline void config_E_as_output()  { DDR(E_PORT)  |= (1<<E_LOC); }

static inline void config_DB0_as_input(){ DDR(DB0_PORT) &= ~(1<<DB0_LOC); }
static inline void config_DB1_as_input(){ DDR(DB1_PORT) &= ~(1<<DB1_LOC); }
static inline void config_DB2_as_input(){ DDR(DB2_PORT) &= ~(1<<DB2_LOC); }
static inline void config_DB3_as_input(){ DDR(DB3_PORT) &= ~(1<<DB3_LOC); }

static inline void set_DB0_bit(){ PORT(DB0_PORT) |= (1<<DB0_LOC); }
static inline void set_DB1_bit(){ PORT(DB1_PORT) |= (1<<DB1_LOC); }
static inline void set_DB2_bit(){ PORT(DB2_PORT) |= (1<<DB2_LOC); }
static inline void set_DB3_bit(){ PORT(DB3_PORT) |= (1<<DB3_LOC); }

static inline void clear_DB0_bit(){ PORT(DB0_PORT) &= ~(1<<DB0_LOC); }
static inline void clear_DB1_bit(){ PORT(DB1_PORT) &= ~(1<<DB1_LOC); }
static inline void clear_DB2_bit(){ PORT(DB2_PORT) &= ~(1<<DB2_LOC); }
static inline void clear_DB3_bit(){ PORT(DB3_PORT) &= ~(1<<DB3_LOC); }

static inline bool get_DB0_bit(){ return PIN(DB0_PORT)&(1<<DB0_LOC); }
static inline bool get_DB1_bit(){ return PIN(DB1_PORT)&(1<<DB1_LOC); }
static inline bool get_DB2_bit(){ return PIN(DB2_PORT)&(1<<DB2_LOC); }
static inline bool get_DB3_bit(){ return PIN(DB3_PORT)&(1<<DB3_LOC); }

static inline void set_RS_bit(){ PORT(RS_PORT) |= (1<<RS_LOC); }
static inline void clear_RS_bit(){ PORT(RS_PORT) &= ~(1<<RS_LOC); }
//static inline void set_RW_bit(){ PORT(RW_PORT) |= (1<<RW_LOC); }
//static inline void clear_RW_bit(){ PORT(RW_PORT) &= ~(1<<RW_LOC); }
static inline void set_E_bit(){ PORT(E_PORT) |= (1<<E_LOC); }
static inline void clear_E_bit(){ PORT(E_PORT) &= ~(1<<E_LOC); }

static void set_data_bus_dir_to_output()
{
	config_DB0_as_output();
	config_DB1_as_output();
	config_DB2_as_output();
	config_DB3_as_output();
}
//static void set_data_bus_dir_to_input()
//{
//	config_DB0_as_input();
//	config_DB1_as_input();
//	config_DB2_as_input();
//	config_DB3_as_input();
//}
static void output_nibble_to_data_bus(uint8_t data)
{
	if( data & (1<<0) )
		set_DB0_bit();
	else
		clear_DB0_bit();
	if( data & (1<<1) )
		set_DB1_bit();
	else
		clear_DB1_bit();
	if( data & (1<<2) )
		set_DB2_bit();
	else
		clear_DB2_bit();
	if( data & (1<<3) )
		set_DB3_bit();
	else
		clear_DB3_bit();
}

static uint8_t input_nibble_from_data_bus()
{
	uint8_t data=0;
	if( get_DB0_bit() )
		data |= (1<<0);
	if( get_DB1_bit() )
		data |= (1<<1);
	if( get_DB2_bit() )
		data |= (1<<2);
	if( get_DB3_bit() )
		data |= (1<<3);
	return data;
}

void initialize_LCD_driver()
{	//this is the initialization sequence recommended in datasheet
	config_RS_as_output();
//	config_RW_as_output();
	clear_RS_bit();
//	clear_RW_bit();
	clear_E_bit();
	config_E_as_output();
	set_data_bus_dir_to_output();
	_delay_ms(20);
	nibble_out(0x3);
	_delay_ms(6);
	nibble_out(0x3);
	_delay_ms(2);
	nibble_out(0x3);
	_delay_ms(2);
	nibble_out(0x2);
	_delay_ms(0x2);
	nibble_out(0x2);
	nibble_out(0x8);
	nibble_out(0x0);
	nibble_out(0x8);
	nibble_out(0x0);
	nibble_out(0x1);
	nibble_out(0x0);
	nibble_out(0x6);

}

static void pulse_E()
{
	_delay_us(1);
	set_E_bit();
	_delay_us(5);
	clear_E_bit();
}

static void nibble_out(uint8_t data)
{
	output_nibble_to_data_bus(data);
	pulse_E();
}

static void send(unsigned char data, unsigned char rs)
{
	uint8_t save_bus=input_nibble_from_data_bus();
	_delay_ms( 4);
	if( rs )
		set_RS_bit();
	else
		clear_RS_bit();
//	set_data_bus_dir_to_output();
	nibble_out(data >> 4);
	nibble_out(data & 0x0F);
	output_nibble_to_data_bus(save_bus);
}
static void send_command(unsigned char cmd)
{
	send(cmd,0);
}
static void send_data(unsigned char data)
{
	send(data,1);
}
void LCD_set_cursor(uint8_t row, uint8_t col)
{
	send_command(  col | 0x80 | (row<<6));
}
void LCD_simple_command( uint8_t command)
{
	send_command(command);
}
void LCD_putchar( char c)
{
	send_data((unsigned char)c);
}

void configure_3pi()
{
	cli();
	configure_motor_timer();  //timer 1	prepared for motor pwm
	configure_timer4();
	//configure_proximity_led_timer();
	configure_quadrature_counters();
	initialize_LCD_driver();
	sei();
}
