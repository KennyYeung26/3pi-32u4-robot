
//Spring 2024, EECE 287 Sophomore Design, Meghana Jain
//Project # 1
//Group # 17
//Team Member 1: Anthony Chen
//Team Member 2: Kenny Yeung


#include <avr/io.h>

#define F_CPU 16000000 //must be before delay

#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "3pi_32u4_drivers.h"

//define PWM parameters
#define PWM_TOP 100 //maximum value of 100, indicating 100%

//define button locations
#define LEFT_MOTOR_PWD_LOCATION 6
#define RIGHT_MOTOR_PWD_LOCATION 5
#define LEFT_MOTOR_DIRECTION 2
#define RIGHT_MOTOR_DIRECTION 1

//define sensor locations
#define RIGHT_OUTER_SENSOR_LOCATION 6
#define RIGHT_INNER_SENSOR_LOCATION 7
#define MIDDLE_SENSOR_LOCATION 5
#define LEFT_INNER_SENSOR_LOCATION 4
#define LEFT_OUTER_LOCATION 1
#define IR_EMITTER_LOCATION 7

void right_wheel_forward()
{
	PORTB &= ~(1<<RIGHT_MOTOR_DIRECTION);
	PORTB |= (1<<RIGHT_MOTOR_PWD_LOCATION);
}

void right_wheel_back()
{
	PORTB |= (1<<RIGHT_MOTOR_DIRECTION);
	PORTB |= (1<<RIGHT_MOTOR_PWD_LOCATION);
}

void left_wheel_forward()
{
	PORTB &= ~(1<<LEFT_MOTOR_DIRECTION);
	PORTB |= (1<<LEFT_MOTOR_PWD_LOCATION);
}

void left_wheel_back()
{
	PORTB |= (1<<LEFT_MOTOR_DIRECTION);
	PORTB |= (1<<LEFT_MOTOR_PWD_LOCATION);
}

void turn_off_left_wheel()
{
	PORTB &= ~(1<<LEFT_MOTOR_PWD_LOCATION);
}

void turn_off_right_wheel()
{
	PORTB &= ~(1<<RIGHT_MOTOR_PWD_LOCATION);
}

void turn_off_both_wheels()
{
	turn_off_left_wheel();
	turn_off_right_wheel();
}

void both_wheels_backward(unsigned int pwm_counter, unsigned int right_duty_cycle, unsigned int left_duty_cycle)
{
	//Turn on both wheels
	if (pwm_counter < right_duty_cycle) {
		right_wheel_back();
	}
	else {
		turn_off_right_wheel();
	}
	
	if (pwm_counter < left_duty_cycle) {
		left_wheel_back();
	}
	else {
		turn_off_left_wheel();
	}
	
}

void spin_right(unsigned int pwm_counter, unsigned int right_duty_cycle, unsigned int left_duty_cycle)
{
	//Turn on both wheels
	if (pwm_counter < right_duty_cycle) {
		right_wheel_back();
	}
	else {
		turn_off_right_wheel();
	}
	
	if (pwm_counter < left_duty_cycle) {
		left_wheel_forward();
	}
	else {
		turn_off_left_wheel();
	}
}

void spin_left(unsigned int pwm_counter, unsigned int right_duty_cycle, unsigned int left_duty_cycle)
{
	//Turn on both wheels
	if (pwm_counter < right_duty_cycle) {
		right_wheel_forward();
	}
	else {
		turn_off_right_wheel();
	}
	
	if (pwm_counter < left_duty_cycle) {
		left_wheel_back();
	}
	else {
		turn_off_left_wheel();
	}
}

void both_wheels_forward(unsigned int pwm_counter, unsigned int right_duty_cycle, unsigned int left_duty_cycle)
{
	//Turn on both wheels
	if (pwm_counter < right_duty_cycle) {
		right_wheel_forward();
	}
	else {
		turn_off_right_wheel();
	}
	
	if (pwm_counter < left_duty_cycle) {
		left_wheel_forward();
	}
	else {
		turn_off_left_wheel();
	}
	
}

void move_set_distance(uint32_t*right_count_distance, int new_movement_amount, unsigned int*bomb_check, unsigned int pwm_counter, unsigned int right_duty_cycle, unsigned int left_duty_cycle)
{
	if (get_right_quadrature_counter() <  new_movement_amount + *right_count_distance ) {
		both_wheels_forward(pwm_counter, right_duty_cycle, left_duty_cycle);
	}
	else {
		turn_off_both_wheels();
		*right_count_distance = get_right_quadrature_counter();
		*bomb_check = 0;
	}
}

void move_back_set_distance(uint32_t*right_count_distance, int new_movement_amount, unsigned int*bomb_defuse, unsigned int pwm_counter, unsigned int right_duty_cycle, unsigned int left_duty_cycle)
{
	if (get_right_quadrature_counter() <  new_movement_amount + *right_count_distance ) {
		both_wheels_backward(pwm_counter, right_duty_cycle, left_duty_cycle);
	}
	else {
		turn_off_both_wheels();
		*right_count_distance = get_right_quadrature_counter();
		*bomb_defuse = 1;
	}
}

void turn_right_set_distance(uint32_t*right_count_distance, int new_movement_amount, int*turn_complete, unsigned int pwm_counter, unsigned int right_duty_cycle, unsigned int left_duty_cycle)
{
	if (get_right_quadrature_counter() <  new_movement_amount  + *right_count_distance ) {
		spin_right(pwm_counter, right_duty_cycle, left_duty_cycle);
	}
	else {
		turn_off_both_wheels();
		*right_count_distance = get_right_quadrature_counter();
		*turn_complete = 1;
	}
}


void turn_left_set_distance(uint32_t*left_count_distance, int new_movement_amount, int*turn_complete, unsigned int pwm_counter, unsigned int right_duty_cycle, unsigned int left_duty_cycle)
{
	if (get_left_quadrature_counter() <  new_movement_amount  + *left_count_distance ) {
		spin_left(pwm_counter, right_duty_cycle, left_duty_cycle);
	}
	else {
		turn_off_both_wheels();
		*left_count_distance = get_right_quadrature_counter();
		*turn_complete = 1;
	}
}

void IR_emitter()
{
	DDRB |= (1<<IR_EMITTER_LOCATION);
	PORTB |= (1<<IR_EMITTER_LOCATION);
}


void turn_on_right_outer_sensor()
{
	DDRD &= ~(1<<RIGHT_OUTER_SENSOR_LOCATION);
	PORTD |= (1<<RIGHT_OUTER_SENSOR_LOCATION);
}

void turn_on_right_inner_sensor()
{
	DDRF &= ~(1<<RIGHT_INNER_SENSOR_LOCATION);
	PORTF |= (1<<RIGHT_INNER_SENSOR_LOCATION);
}

void turn_on_middle_sensor()
{
	DDRF &= ~(1<<MIDDLE_SENSOR_LOCATION);
	PORTF |= (1<<MIDDLE_SENSOR_LOCATION);
}

void turn_on_left_inner_sensor()
{
	DDRF &= ~(1<<LEFT_INNER_SENSOR_LOCATION);
	PORTF |= (1<<LEFT_INNER_SENSOR_LOCATION);
}

void turn_on_left_outer_sensor()
{
	DDRF &= ~(1<<LEFT_OUTER_LOCATION);
	PORTF |= (1<<LEFT_OUTER_LOCATION);
}

void button_a_pulser(unsigned int*last_button_A_state, unsigned int*button_A_pressed)
{
	//Pulser for button A
	if( button_a_is_down() != *last_button_A_state) {
		//if button is pressed
		if(button_a_is_down()) {
			*button_A_pressed = 1;
		}
		//update last button state
		*last_button_A_state = (button_a_is_down());
	}
	else {
		*button_A_pressed = 0;
	}
}

void button_b_pulser(unsigned int*last_button_B_state, unsigned int*button_B_pressed)
{
	//Pulser for button B
	if( button_b_is_down() != *last_button_B_state) {
		//if button is pressed
		if(button_b_is_down()) {
			*button_B_pressed = 1;
		}
		//update last button state
		*last_button_B_state = (button_b_is_down());
	}
	else {
		*button_B_pressed = 0;
	}
}

void button_c_pulser(unsigned int*last_button_C_state, unsigned int*button_C_pressed)
{
	//Pulser for button C
	if( button_c_is_down() != *last_button_C_state){
		if(button_c_is_down()){    //if button is pressed
			*button_C_pressed = 1;
		}
		//update last button state
		*last_button_C_state = (button_c_is_down());
	}
	else{
		*button_C_pressed = 0;
	}
}

void update_LCD(int row, int col, int count)
{
	int tens = 0;
	int ones = 0;
	
	LCD_set_cursor(row, col);
	tens = 48 + (count/10);
	LCD_putchar( tens );
	ones = 48 + (count%10);
	LCD_putchar( ones );
}

void set_up_LCD_bombs(int*bomb_count)
{
	unsigned int last_button_A_state = button_a_is_down();
	unsigned int button_A_pressed = 0;
	unsigned int last_button_B_state = button_b_is_down();
	unsigned int button_B_pressed = 0;
	unsigned int last_button_C_state = button_c_is_down();
	unsigned int button_C_pressed = 0;

	
	LCD_set_cursor(0,0);
	LCD_putchar('B');
	LCD_putchar('O');
	LCD_putchar('M');
	LCD_putchar('B');
	LCD_putchar('S');
	LCD_putchar(':');
	
	LCD_set_cursor(1,0);
	LCD_putchar('F');
	LCD_putchar('O');
	LCD_putchar('U');
	LCD_putchar('N');
	LCD_putchar('D');
	LCD_putchar(':');
	
	while (button_C_pressed != 1)
	{
		button_a_pulser( &last_button_A_state, &button_A_pressed);
		button_b_pulser( &last_button_B_state, &button_B_pressed);
		button_c_pulser( &last_button_C_state, &button_C_pressed);
	
		if (button_A_pressed == 1)
		{
			(*bomb_count)--;
		
			if (*bomb_count < 0) {
				*bomb_count = 25;
			}
			
			update_LCD(0, 6, (*bomb_count));		
		
		}
	
	
		if (button_B_pressed == 1)
		{
			(*bomb_count)++;
		
			if (*bomb_count > 25) 
			{
				*bomb_count = 0;
			}
	
			update_LCD(0, 6, (*bomb_count));
			
		}
	}
	
	_delay_ms(300);
}

// keep robot on the path when going straight
void stay_on_path(unsigned int*left_duty_cycle, unsigned int*right_duty_cycle)
{
		if (line_sensor[1] < line_sensor[3]) {
			*left_duty_cycle = 12;
			*right_duty_cycle = 13;
		}
		else if (line_sensor[3] < line_sensor[1]) {
			*left_duty_cycle = 13;
			*right_duty_cycle = 12;
		}
		else if (line_sensor[3] == line_sensor[1]) {
			*left_duty_cycle = 13;
			*right_duty_cycle = 13;
		}
	
}

int detect_right()
{
	int light_value = 0;
	
	if (line_sensor[0] < 110)
	{
		light_value = 1;
	}
	return light_value;

}

int detect_left()
{
	int light_value = 0;
	
	if (line_sensor[4] < 110)
	{
		light_value = 1;
	}
	return light_value;

}

//Make pulser for right edge
void left_edge(unsigned int*last_left_state, unsigned int*left_state)
{
	 //Pulser for button A
	 if( detect_left() != *last_left_state)
	 {
		 //if bomb is detected
		 if(detect_left() == 1)
		 {
			 *left_state = 1;
		 }
		 //update last button state
		 *last_left_state = (detect_left());
	 }
	 else
	 {
		 *left_state = 0;
	 }
}

void right_edge(unsigned int*last_right_state, unsigned int*right_state)
{
	//Pulser for button A
	if( detect_right() != *last_right_state)
	{
		//if bomb is detected
		if(detect_right() == 1)
		{
			*right_state = 1;
		}
		//update last button state
		*last_right_state = (detect_right());
	}
	else
	{
		*right_state = 0;
	}
}



int obstacle_found()
{
	int obstacle = 0;
	
	// detect a obstacle
	if ( line_sensor[2] < 80 )
	{
		obstacle = 1;
	}
	return obstacle;
 }
 
 void detect_bomb (unsigned int*last_bomb_state, unsigned int*bomb_state)
 {
	 //Pulser for bomb
	 if((!obstacle_found()) != *last_bomb_state)
	 {
		 //if bomb is detected
		 if((!obstacle_found()) == 1)
		 {
			 *bomb_state = 1;
		 }
		 //update last button state
		 *last_bomb_state = (!obstacle_found());
	 }
	 else
	 {
		 *bomb_state = 0;
	 }
 }

// make robot find corner
int find_corner(int bomb_count, int*bombs_found)
{
	int next_turn_direction = 0;
	int corner_found = 0;
	unsigned int bomb_state = 0;
	unsigned int last_bomb_state = 0;
	unsigned int left_state = 0;
	unsigned int last_left_state = 1;	
	unsigned int right_state = 0;
	unsigned int last_right_state = 1;
	
	unsigned int bomb_instruction = 0;
	unsigned int bomb_check = 0;
	int edge_found = 0;
	int full_turn_complete = 0;
	int turn_complete = 0;
	int turn_number = 0;
	uint32_t right_count_distance = 0;
	uint32_t left_count_distance = 0;

	int one_inch = 120;
	int right_turn = 195;
	int left_turn = 195;
	int full_turn = 960;
	unsigned int pwm_counter = 0;
	
	unsigned int left_duty_cycle = 13;
	unsigned int right_duty_cycle = 13;
	
	
	while (corner_found != 1 && ((*bombs_found) < bomb_count))
	{
		
		//PWM Counter
		pwm_counter++;
			
		if( pwm_counter >= PWM_TOP ) {
			pwm_counter = 0;
		}
		// due to some drift, the left and right duty cycles need to be modified during runs to ensure minimal interference
	
		
		
		//|| right_turn_complete == 1
		// Condition - move forward when no bomb detected or has been detected and turning
		if (bomb_check == 0 && (bomb_state == 0) &&  edge_found == 0 )
		{
			stay_on_path(&left_duty_cycle, &right_duty_cycle);
			
			// run in a straight line, 
			both_wheels_forward(pwm_counter, right_duty_cycle, left_duty_cycle);
			
			// when white to black is detected
			if (obstacle_found() == 1)
			{
				bomb_check = 1;
			}

			
		}
		else if (bomb_check == 1)
		{
			left_edge(&last_left_state, &left_state);
			right_edge(&last_right_state, &right_state);
			
			
			stay_on_path(&left_duty_cycle, &right_duty_cycle);
			
			both_wheels_forward(pwm_counter, right_duty_cycle, left_duty_cycle);
			// always check if white to black is detected in the middle 3 sensors
			detect_bomb(&last_bomb_state, &bomb_state);
							
			// Check if it encounters a
			if ((left_state == 1 || right_state == 1))
			{
				edge_found = 1;
			}

							
			if (bomb_state == 1 || edge_found == 1)
			{
				turn_off_both_wheels();
				right_count_distance = get_right_quadrature_counter();
				left_count_distance = get_left_quadrature_counter();
				bomb_check = 0;
			}
			

		}
		// Condition - bomb is detected
		else if (bomb_state == 1 )
		{
			left_duty_cycle = 13;
			right_duty_cycle = 13;
				
			if (bomb_instruction == 0)
			{
				move_back_set_distance(&right_count_distance, one_inch, &bomb_instruction,  pwm_counter, right_duty_cycle, left_duty_cycle);
			}
			
			if (bomb_instruction == 1)
			{
			// spin 360
			 turn_right_set_distance(&right_count_distance, full_turn, &full_turn_complete,  pwm_counter, right_duty_cycle, left_duty_cycle);
			}
			
			// revolution_complete would be a return from revolution()
			if (full_turn_complete == 1)
			{
				
				//update the lcd and bomb counter
				(*bombs_found)++;
				update_LCD(1, 6, *bombs_found);
				_delay_ms(1250);
				bomb_instruction = 0;
				full_turn_complete=0;
				detect_bomb(&last_bomb_state, &bomb_state);
			}
			
		}

		// Turn right when hitting the first edge
		else if (edge_found == 1)
		{
			if (right_state == 1)
			{
				 turn_right_set_distance(&right_count_distance, right_turn, &turn_complete,  pwm_counter, right_duty_cycle, left_duty_cycle);
				 if (turn_complete == 1 )
				 {
					 //after first turn, reset the values
					 edge_found = 0;
					 turn_number++;
					 turn_complete = 0;
					 
					 right_edge(&last_right_state, &right_state);
					 left_edge(&last_left_state, &left_state);
					 
					 _delay_ms(150);
				 }
				 
				 if (turn_number == 2)
				 {
					 next_turn_direction = 1;
					 corner_found = 1;
				 }
			}
			else if (left_state == 1)
			{
				turn_left_set_distance(&left_count_distance, left_turn, &turn_complete,  pwm_counter, right_duty_cycle, left_duty_cycle);
				if (turn_complete == 1)
				{
					edge_found = 0;
					turn_number++;
					turn_complete = 0;
					right_edge(&last_right_state, &right_state);
					left_edge(&last_left_state, &left_state);
					
					_delay_ms(150);
				}
				
				if (turn_number == 2)
				{
					next_turn_direction = 0;
					corner_found = 1;
				}
				
				
			}
			
		}

		
	}
	
	update_LCD(1, 6, *bombs_found);
	
	return next_turn_direction;
	
}

unsigned int snake_about (unsigned int path_direction, int bomb_count, int*bombs_found)
{
	unsigned int bomb_state = 0;
	unsigned int last_bomb_state = 0;
	unsigned int left_state = 0;
	unsigned int last_left_state = 1;
	unsigned int right_state = 0;
	unsigned int last_right_state = 1;
	unsigned int movement_type = 0;
	unsigned int switch_direction = 0;
	unsigned int routine_complete = 0;
		
	unsigned int bomb_instruction = 0;
	unsigned int bomb_check = 0;
	int edge_found = 0;
	int full_turn_complete = 0;
	int turn_complete = 0;

	uint32_t right_count_distance = 0;
	uint32_t left_count_distance = 0;

	int one_inch = 120;
	int right_turn = 195;
	int left_turn = 195;
	int full_turn = 975;
	unsigned int pwm_counter = 0;
		
	unsigned int left_duty_cycle = 13;
	unsigned int right_duty_cycle = 13;
	
	
	while (routine_complete == 0 && ((*bombs_found) < bomb_count))
	{
		//PWM Counter
		pwm_counter++;
		
		if( pwm_counter >= PWM_TOP ) {
			pwm_counter = 0;
		}
		
		if ( bomb_check == 0 && bomb_state == 0 &&  edge_found == 0 )
		{
			
			stay_on_path(&left_duty_cycle, &right_duty_cycle);
			
			// run in a straight line,
			both_wheels_forward(pwm_counter, right_duty_cycle, left_duty_cycle);
			
			// when white to black is detected
			if (obstacle_found() == 1)
			{
				bomb_check = 1;
			}
			
			// If at in intersection before second turn
			if (movement_type == 1 && (detect_left() == 0 || detect_right() == 0))
			{
				switch_direction = 1;
				bomb_check = 1;
			}
			
		}
		// Condition - check for bomb
		else if (bomb_check == 1)
		{
			left_edge(&last_left_state, &left_state);
			right_edge(&last_right_state, &right_state);
			
			
			stay_on_path(&left_duty_cycle, &right_duty_cycle);
			
			both_wheels_forward(pwm_counter, right_duty_cycle, left_duty_cycle);
			
			// Check if white to black is detected in the middle 3 sensors
			if (switch_direction == 0)
			{
				detect_bomb(&last_bomb_state, &bomb_state);
			}
			
			// Check if it encounters a
			if ((left_state == 1 || right_state == 1) )
			{
				edge_found = 1;
				if (detect_left() == 0)
				{
					left_state = 1;
				}
				if (detect_right() == 0)
				{
					right_state = 1;
				}
			}

			
			if (bomb_state == 1 || edge_found == 1)
			{
				turn_off_both_wheels();
				right_count_distance = get_right_quadrature_counter();
				left_count_distance = get_left_quadrature_counter();
				bomb_check = 0;
			}
			
		}
		// Condition - bomb is detected
		else if (bomb_state == 1 )
		{
			left_duty_cycle = 13;
			right_duty_cycle = 13;
			
			if (bomb_instruction == 0)
			{
				move_back_set_distance(&right_count_distance, one_inch, &bomb_instruction,  pwm_counter, right_duty_cycle, left_duty_cycle);
			}
			
			if (bomb_instruction == 1)
			{
				// spin 360
				turn_right_set_distance(&right_count_distance, full_turn, &full_turn_complete,  pwm_counter, right_duty_cycle, left_duty_cycle);
			}
			
			// revolution_complete would be a return from revolution()
			if (full_turn_complete == 1)
			{
				
				// update the LCD and bomb counter
				(*bombs_found)++;
				update_LCD(1, 6, *bombs_found);
				
				right_count_distance = get_right_quadrature_counter();
				left_count_distance = get_left_quadrature_counter();
				bomb_instruction = 0;
				full_turn_complete = 0;
				detect_bomb(&last_bomb_state, &bomb_state);
				_delay_ms(1250);
				
			}
			
		}
		else if (edge_found == 1)
		{
			left_duty_cycle = 13;
			right_duty_cycle = 13;
			
			if (path_direction == 0 && left_state == 1)
			{
				turn_left_set_distance(&left_count_distance, left_turn, &turn_complete,  pwm_counter, right_duty_cycle, left_duty_cycle);
				if (turn_complete == 1)
				{
					edge_found = 0;
					turn_complete = 0;
					right_edge(&last_right_state, &right_state);
					left_edge(&last_left_state, &left_state);
					
					if (switch_direction == 1)
					{
						switch_direction = 0;
						path_direction ^= (1<<0);
					}
					
					movement_type ^= (1<<0);
					
					right_count_distance = get_right_quadrature_counter();
					left_count_distance = get_left_quadrature_counter();
					
					_delay_ms(150);
				}
				
			}
			else if (path_direction == 1 && right_state == 1)
			{
				turn_right_set_distance(&right_count_distance, right_turn, &turn_complete,  pwm_counter, right_duty_cycle, left_duty_cycle);
				if (turn_complete == 1)
				{
					edge_found = 0;
					turn_complete = 0;
					right_edge(&last_right_state, &right_state);
					left_edge(&last_left_state, &left_state);
					
					if (switch_direction == 1)
					{
						switch_direction = 0;
						path_direction ^= (1<<0);
					}
					movement_type ^= (1<<0);
					
					right_count_distance = get_right_quadrature_counter();
					left_count_distance = get_left_quadrature_counter();
					
					_delay_ms(150);
				}
			}
			else
			{

				routine_complete = 1;
				update_LCD(1, 6, *bombs_found);
			}
			
		}
	
	}
	
	return (path_direction ^ (1<<0));
}

void celebration()
{
	unsigned int pwm_counter = 0;
	unsigned int left_duty_cycle = 30;
	unsigned int right_duty_cycle = 30;
	uint32_t right_count_distance = get_right_quadrature_counter();
	int full_turn_complete = 0;
	int four_full_turns = 3775;
	
	while (full_turn_complete != 1)
	{
	//PWM Counter
	pwm_counter++;
	
		if( pwm_counter >= PWM_TOP ) 
		{
			pwm_counter = 0;
		}
		// spin 360
		turn_right_set_distance(&right_count_distance, four_full_turns, &full_turn_complete,  pwm_counter, right_duty_cycle, left_duty_cycle);
	}
}

int main(void)
{
	configure_3pi();

	unsigned int screen = 0x0F;
	
	LCD_simple_command( screen );


	uint32_t right_count_distance = 0;
	uint32_t left_count_distance = 0;

	int bomb_count = 0;
	int bombs_found = 0;
	int right_turn = 200;
	int left_turn = 200;
	unsigned int pwm_counter = 0;
	
	unsigned int left_duty_cycle = 14;
	unsigned int right_duty_cycle = 14;
	unsigned int path_direction = 0;             // If path direction is 0, take left. If 1, take right.
	
	IR_emitter();
	
	turn_on_right_outer_sensor();
	turn_on_right_inner_sensor();
	turn_on_middle_sensor();
	turn_on_left_inner_sensor();
	turn_on_left_outer_sensor();

	
	set_up_LCD_bombs(&bomb_count);
	
	
	path_direction = find_corner(bomb_count, &bombs_found);
	
	path_direction = snake_about(path_direction, bomb_count, &bombs_found);
	int turn_complete = 0;

	right_count_distance = get_right_quadrature_counter();
	left_count_distance = get_left_quadrature_counter();
	
	while ((turn_complete != 1) && (bombs_found < bomb_count)){
		//PWM Counter
		pwm_counter++;
		
		if( pwm_counter >= PWM_TOP ) {
			pwm_counter = 0;
		}
		
		if (path_direction == 1)
		{
			turn_right_set_distance(&right_count_distance, right_turn, &turn_complete,  pwm_counter, right_duty_cycle, left_duty_cycle);
		}
		else if (path_direction == 0)
		{
			turn_left_set_distance(&left_count_distance, left_turn, &turn_complete,  pwm_counter, right_duty_cycle, left_duty_cycle);
		}
		
	}
		
	_delay_ms(250);
	
	path_direction = snake_about(path_direction, bomb_count, &bombs_found);
	
	celebration();
}


