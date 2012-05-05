#include "encoder.h"
#include "global.h"
#include "motor.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

// from hardware.h (should be incorporated soon)
typedef enum {
    FRONT = 0,
    LEFT = 1,
    RIGHT = 2,
    NONE = -1,
} side_t;

// Pin Change ISR
ISR(PCINT2_vect) {
    // Check to see which pin has changed by comparing the value actually on
    // the pin to the one stored in the respective variable. If there is
    // a change, update current

    // Left motor
    encoder_buff = (PIND & _BV(PIND0));
    if (encoder_buff != left_current_A) {
        left_current_A = encoder_buff;
        LEUF = true;
        EUF = true;
    }

    encoder_buff = (PIND & _BV(PIND1));
    if (encoder_buff != left_current_B) {
        left_current_B = encoder_buff;
        LEUF = true;
        EUF = true;
    }

    // Right motor
    encoder_buff = (PIND & _BV(PIND6));
    if (encoder_buff != right_current_A) {
        right_current_A = encoder_buff;
        REUF = true;
        EUF = true;
    }
    encoder_buff = (PIND & _BV(PIND5));
    if (encoder_buff != right_current_B) {
        right_current_B = encoder_buff;
        REUF = true;
        EUF = true;
    }

    update_encoder();
    encoder_debug();
}

volatile uint16_t sensor[3] = {0};
volatile uint16_t sensor_max[3] = {0};
inline float norm(side_t dir) {
    return (float) sensor[dir] / (float) sensor_max[dir];
}

void kill_motors(void) {
    motor_set_speed('r', 0);
    motor_set_speed('l', 0);
}

// Returns true if there is currently a wall in the specified direction with
// respect to the robot.
inline bool has_wall(side_t side) {
    return (norm(side) >= 0.4);
}

bool new_wall(side_t* current_wall) {
    //check for a new wall and assign
    if (!has_wall(*current_wall) || *current_wall == NONE) {
        if (*current_wall == NONE) {
            if (has_wall(RIGHT)) {
                *current_wall = RIGHT;
                return true;
            } else if (has_wall(LEFT)) {
                *current_wall = LEFT;
                return true;
            }
        } else if ((*current_wall == RIGHT) && (!has_wall(RIGHT)) &&
                    has_wall(LEFT)) {
            *current_wall = LEFT;
                return true;
        } else if ((*current_wall == LEFT) && !has_wall(LEFT) &&
                    has_wall(RIGHT)) {
            *current_wall = RIGHT;
                return true;
        } else if (!has_wall(LEFT) && !has_wall(RIGHT)) {
            //No wall on either side
            *current_wall = NONE;
                return true;
        }
    }
    return false;
}

inline void motor_speed(uint16_t left, uint16_t right) {
    OCR1B = left;
    OCR1A = right;
}

inline bool front_clear(void) {
    return PINC & _BV(PINC5);
}

// Move n squares forward.
void move_forward(int8_t n) {
    float error_prev = 0.0;
    float error = 0;
    float integral = 0;
    float derivative = 0;
    side_t current_wall;
    motor_set_direction('r', 'f');
    motor_set_direction('l', 'f');

    for (int8_t i = 0; i < n; i++) {
        right_turns = 0;
        left_turns = 0;
        while ((right_turns < 800 || left_turns < 800) && front_clear()) {
            if (new_wall(&current_wall)) {
                integral = 0;
                derivative = 0;
                error_prev = 0;
            }

            //          unused, LEFT, RIGHT
            float thresh[] = {0, 348, 307};
            float Kp[] = {0, 5, 5};
            float Ki[] = {0, 0, 0};
            float Kd[] = {0, 0, 0};
            //Reset all PID variables
            //If there is a wall to the right or the left use analog PID
            if (current_wall != NONE) {
                side_t cw = current_wall;

                error = sensor[cw] - thresh[cw];
                integral += error;
                derivative = error - error_prev;

                int16_t output = Kp[cw] * error + Ki[cw] * integral +
                                 Kd[cw] * derivative;

                if (current_wall == LEFT) {
                    ///slow down left motor
                    motor_speed(512 + output, 512 - output);
                } else {
                    //slow down right motor
                    motor_speed(512 - output, 512 + output);
                }
            } else {
                /* do something with encoders */
                kill_motors();
            }

        }
    }
    kill_motors();
}

// Rotate 90 degrees counter-clockwise n times.
void turn_left(int8_t n) {
    motor_set_direction('r', 'f');
    motor_set_direction('l', 'r');
    OCR1B = 100;
    OCR1A = 100;
    left_turns = 0;
    while (left_turns < 200) {
        OCR1B += 100;
        OCR1A += 100;
    }
    left_turns = 0;
    kill_motors();
}

// Rotate 90 degrees clockwise n times.
void turn_right(int8_t n) {
    motor_set_direction('l', 'f');
    motor_set_direction('r', 'r');
    OCR1B = 100;
    OCR1A = 100;
    right_turns = 0;
    while (right_turns < 200) {
        OCR1B += 100;
        OCR1A += 100;
    }
    right_turns = 0;
    kill_motors();
}

// ADC ISR
ISR (ADC_vect) {
    static bool first = true;
    if (first) {
        first = false;
    } else {
        if ((((ADMUX & 0x0F) == 0) || ((ADMUX & 0xF) == 1)) &&
            (ADC > CURRENT_THRESHOLD)) {
            kill_motors(); // should signal motor failure, too
        } else if ((ADMUX & 0x0F) == 4) {
            sensor[FRONT] = ADC;
            if (sensor[FRONT] > sensor_max[FRONT])
                sensor_max[FRONT] = sensor[FRONT];
        } else if ((ADMUX & 0x0F) == 7) {
            sensor[RIGHT] = ADC;
            if (sensor[RIGHT] > sensor_max[RIGHT])
                sensor_max[RIGHT] = sensor[RIGHT];
        } else if ((ADMUX & 0x0F) == 6) {
            sensor[LEFT] = ADC;
            if (sensor[LEFT] > sensor_max[LEFT])
                sensor_max[LEFT] = sensor[LEFT];
        } else {
            // should signal failure
        }

        uint8_t X = 0;
        uint8_t next_sensor[] = {1, 4, X, X, 6, X, 7, 0};
        ADMUX = (ADMUX & 0xF0) | next_sensor[ADMUX & 0x0F];
        first = true;
    }

    if (has_wall(RIGHT)) {
        PORTC |= _BV(PORTC3);
    } else {
        PORTC &= ~(_BV(PORTC3));
    }

    if (has_wall(LEFT)) {
        PORTC |= _BV(PORTC2);
    } else {
        PORTC &= ~(_BV(PORTC2));
    }

	ADCSRA |= _BV(ADSC); // start another conversion
}


int main(void) {
    enc_init();
    motor_init();

    encoder_debug_init();

    right_turns = 0;
    left_turns = 0;
    right_direction = REVERSE;
    left_direction = REVERSE;

    encoder_debug_init();

    //Enable all interrupts
    sei();

    //uint8_t buff = 0;

    DDRC &= ~(_BV(DDC5));
    while (1) {
        while (front_clear());
        while (!front_clear());
        while (front_clear()) {
            move_forward(1);
        }
    }

}
