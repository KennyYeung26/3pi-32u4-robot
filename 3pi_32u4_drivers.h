/*
 * 3pi_32u4_drivers.h
 *
 * Created: 1/16/2023 1:16:18 PM
 *  Author: Doug Summerville
 * Updated: Meg Jain, 3/11/2024
 */


#ifndef AVR_DRIVERS_H_
#define AVR_DRIVERS_H_

#include <stdbool.h>
#include <stdint.h>

void configure_3pi();
extern uint8_t line_sensor[5];
_Bool button_a_is_up();
_Bool button_a_is_down();
_Bool button_b_is_up();
_Bool button_b_is_down();
_Bool button_c_is_up();
_Bool button_c_is_down();

void start_stopwatch();
uint32_t read_stopwatch();
extern uint8_t line_sensor[5];

void LCD_goto_col_row(uint8_t x, uint8_t y);
void LCD_simple_command( uint8_t command);
void LCD_putchar( char c);

uint32_t get_right_quadrature_counter();
uint32_t get_left_quadrature_counter();

void LCD_set_cursor(uint8_t row, uint8_t col);
void LCD_simple_command( uint8_t command);
void LCD_putchar( char c);


#endif /* AVR_DRIVERS_H_ */
