/* Keyboard example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_keyboard.html
 * Copyright (c) 2008 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#include "usb_keyboard.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
#define P_MAX 4

struct keyset {
    uint8_t keys[6];
    uint8_t length;
    uint8_t modifier;
};


/*  LAYOUT
 *   ___   ___   ___   ___
 *  | 1 | | 2 | | 3 | | 4 |
 *  |___| |___| |___| |___|
 *
 *   ___   ___   ___   ___
 *  | 5 | | 6 | | 7 | | 8 |
 *  |___| |___| |___| |___|
 *
 *   ___   ___   ___   ___
 *  | 9 | | 10| | 11| | 12|  ___
 *  |___| |___| |___| |___| |   |
 *                          | 16|
 *   ___   ___    _______   |   |
 *  | 13| | 14|  |   15  |  |___|
 *  |___| |___|  |_______|
 *
 */


struct keyset presets[P_MAX][16] = {
    {
        {{KEY_NUM_LOCK}, 1, 0},
        {{KEYPAD_1}, 1, 0},
        {{KEYPAD_2}, 1, 0},
        {{KEYPAD_3}, 1, 0},
        {{KEYPAD_SLASH}, 1, 0},
        {{KEYPAD_4}, 1, 0},
        {{KEYPAD_5}, 1, 0},
        {{KEYPAD_6}, 1, 0},
        {{KEYPAD_ASTERIX}, 1, 0},
        {{KEYPAD_7}, 1, 0},
        {{KEYPAD_8}, 1, 0},
        {{KEYPAD_9}, 1, 0},
        {{KEYPAD_MINUS}, 1, 0},
        {{KEYPAD_PLUS}, 1, 0},
        {{KEYPAD_0}, 1, 0},
        {{KEYPAD_ENTER}, 1, 0},
    },
    {
        {{KEY_A}, 1, KEY_LEFT_SHIFT},
        {{KEY_B}, 1, KEY_LEFT_SHIFT},
        {{KEY_C}, 1, KEY_LEFT_SHIFT},
        {{KEY_D}, 1, KEY_LEFT_SHIFT},
        {{KEY_E}, 1, KEY_LEFT_SHIFT},
        {{KEY_F}, 1, KEY_LEFT_SHIFT},
        {{KEY_G}, 1, KEY_LEFT_SHIFT},
        {{KEY_H}, 1, KEY_LEFT_SHIFT},
        {{KEY_LESS}, 1, 0},
        {{KEY_LESS}, 1, KEY_LEFT_SHIFT},
        {{KEY_LESS}, 1, KEY_RIGHT_ALT},
        {{KEY_K}, 1, KEY_LEFT_SHIFT},
        {{KEY_L}, 1, KEY_LEFT_SHIFT},
        {{KEY_N}, 1, KEY_LEFT_SHIFT},
        {{KEY_O}, 1, KEY_LEFT_SHIFT},
        {{KEY_P}, 1, KEY_LEFT_SHIFT},
    },
    {
        {{KEY_A}, 1, KEY_LEFT_SHIFT},
        {{KEY_B}, 1, KEY_LEFT_SHIFT},
        {{KEY_C}, 1, KEY_LEFT_SHIFT},
        {{KEY_D}, 1, KEY_LEFT_SHIFT},
        {{KEY_E}, 1, KEY_LEFT_SHIFT},
        {{KEY_F}, 1, KEY_LEFT_SHIFT},
        {{KEY_G}, 1, KEY_LEFT_SHIFT},
        {{KEY_H}, 1, KEY_LEFT_SHIFT},
        {{KEY_LESS}, 1, 0},
        {{KEY_LESS}, 1, KEY_LEFT_SHIFT},
        {{KEY_LESS}, 1, KEY_RIGHT_ALT},
        {{KEY_K}, 1, KEY_LEFT_SHIFT},
        {{KEY_L}, 1, KEY_LEFT_SHIFT},
        {{KEY_N}, 1, KEY_LEFT_SHIFT},
        {{KEY_O}, 1, KEY_LEFT_SHIFT},
        {{KEY_P}, 1, KEY_LEFT_SHIFT},
    },
    {
        {{KEY_A}, 1, KEY_LEFT_SHIFT},
        {{KEY_B}, 1, KEY_LEFT_SHIFT},
        {{KEY_C}, 1, KEY_LEFT_SHIFT},
        {{KEY_D}, 1, KEY_LEFT_SHIFT},
        {{KEY_E}, 1, KEY_LEFT_SHIFT},
        {{KEY_F}, 1, KEY_LEFT_SHIFT},
        {{KEY_G}, 1, KEY_LEFT_SHIFT},
        {{KEY_H}, 1, KEY_LEFT_SHIFT},
        {{KEY_LESS}, 1, 0},
        {{KEY_LESS}, 1, KEY_LEFT_SHIFT},
        {{KEY_LESS}, 1, KEY_RIGHT_ALT},
        {{KEY_K}, 1, KEY_LEFT_SHIFT},
        {{KEY_L}, 1, KEY_LEFT_SHIFT},
        {{KEY_N}, 1, KEY_LEFT_SHIFT},
        {{KEY_O}, 1, KEY_LEFT_SHIFT},
        {{KEYPAD_PLUS}, 1, KEY_LEFT_CTRL},
    }
};

uint16_t idle_count=0;

int main(void)
{
	uint8_t key, i, k, reset_idle, p_num = 0;
    uint8_t pinb[4], pinb_prev[4];

	// set for 16 MHz clock
	CPU_PRESCALE(0);


	DDRD = 0xFF;
	DDRB = 0x00;
    DDRF = 0xFF;

	// Initialize the USB, and wait for configuration.
	usb_init();
	while (!usb_configured()) /* wait */ ;

	_delay_ms(1000);

	// Configure timer 0 to generate a timer overflow interrupt every
	// 256*1024 clock cycles, or approx 61 Hz when using 16 MHz clock
	TCCR0A = 0x00;
	TCCR0B = 0x05;
	TIMSK0 = (1<<TOIE0);

	while (1) {

        for (i = 0; i < 4; i++) {
            PORTD = (1 << i);
            _delay_ms(1);
            pinb[i] = PINB;

            for (k = 0; k < 4; k++) {
                key = (i * 4) + k;

                if ((pinb[i] & (1 << k)) && !(pinb_prev[i] & (1 << k))) {
		            usb_keyboard_press_set(
                            presets[p_num][key].keys, 
                            presets[p_num][key].length, 
                            presets[p_num][key].modifier
                    );
                    reset_idle = 1;
                }
            }
        }

        if ((pinb[0] & 1 << 5) && !(pinb_prev[0] & 1 << 5)) {
            if (p_num == 0) {
                p_num = P_MAX - 1;
            }
            else {
                p_num--;
            }
            reset_idle = 1;
        }

        if ((pinb[0] & 1 << 4) && !(pinb_prev[0] & 1 << 4)) {
            if (p_num == P_MAX - 1) {
                p_num = 0;
            }
            else {
                p_num++;
            }
            reset_idle = 1;
        }
        
        PORTF = (1 << (7 - p_num));

		// if any keypresses were detected, reset the idle counter
		if (reset_idle) {
			cli();
			idle_count = 0;
			sei();
//            reset_idle = 0;
		}
        for (i = 0; i < 4; i++) {
            pinb_prev[i] = pinb[i];
        }
		_delay_ms(1);
	}
}

// This interrupt routine is run approx 61 times per second.
// A very simple inactivity timeout is implemented, where we
// will send a space character.
ISR(TIMER0_OVF_vect)
{
	idle_count++;
	if (idle_count > 61 * 8) {
		idle_count = 0;
		usb_keyboard_press(KEY_SPACE, 0);
	}
}


