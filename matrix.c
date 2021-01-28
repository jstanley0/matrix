// 8x8 Red/Green LED Matrix Project
// Scrolling messages / Conway's Game of Life / ???
// based on AVR ATMEGA88 running at 8MHz (pls to turn off the clock divider in lfuse)
// build with avr-gcc
//
// Source code (c) 2010 by Jeremy Stanley
// http://www.xmission.com/~jstanley/avrtimer.html
// Licensed under GNU GPL v2 or later
//

// Port assignments:
// PORTB              = unused
// PORTC0    (output) = Row shift register serial-out (row 0 red, row 0 green, row 1 red, etc., to row 7; 0 = on / 1 = off)
// PORTC1    (output) = Row shift register clock
// PORTC2    (output) = Row shift register latch
// PORTC3             = floating input for RNG
// PORTC4     (input) = button 1 (PCINT12)
// PORTC5     (input) = button 2 (PCINT13)
// PORTD0..7 (output) = column drivers (PD0 = rightmost; PD7 = leftmost)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdlib.h>

#include "font.h"

// Put the processor in idle mode for the specified number of "kiloclocks"
// (= periods of 1024 clock cycles)

volatile uint8_t wakeup;
ISR(TIMER1_COMPA_vect)
{
	wakeup = 1;
}

void Sleep(uint16_t kiloclocks)
{
	TCCR1A = 0;
	TCCR1B = 0;		// stop the timer
	TIFR1 = (1 << OCF1A);	// clear output-compare A flag
	OCR1A = kiloclocks;	// set compare match A target
	TCNT1 = 0;		// reset timer counter
	if (kiloclocks > 0) {
		TIMSK1 = (1 << OCIE1A);	// enable compare match A interrupt
	}
	TCCR1B = (1 << CS12) | (1 << CS10);	// start timer with 1/1024 prescaler

	// sleep until it's time to wake up
	// use a loop here because other interrupts will happen
	wakeup = 0;
	set_sleep_mode(SLEEP_MODE_IDLE);
	do {
		sleep_mode();
	} while( !wakeup );

	TIMSK1 = 0;		// stop the interrupt
	TCCR1B = 0;		// stop the timer
}


// frame buffer - each word stores one column, alternating between green and red.
// there are 8 visible columns; the leftmost column is drawn from framebuf[fb_off].
// fb_base can be adjusted between 0 and 15 to do things like scrolling or page flipping;
// if fb_base > 8, the display ISR will wrap around to 0.
volatile uint8_t fb_base = 0;
volatile uint16_t framebuf[16];

void clear_screen(uint8_t start, uint8_t cols)
{
	memset((void *)&framebuf[start], 0, cols << 1);
}

// display interrupt vector
#define LATCH_0() PORTC &= ~0x04
#define LATCH_1() PORTC |=  0x04
#define SCK_0()   PORTC &= ~0x02
#define SCK_1()   PORTC |=  0x02
#define DATA_0()  PORTC &= ~0x01
#define DATA_1()  PORTC |=  0x01

ISR(TIMER0_COMPA_vect)
{
	static uint8_t col = 0;
	uint8_t fb_off = (fb_base + col) & 0x0F;
	uint16_t c = framebuf[fb_off];
	uint8_t i;

	// shift the data for this column to the 595s
	LATCH_0();
	for(i = 0; i < 16; ++i)
	{
		SCK_0();
		if (c & 1)
			DATA_0();
		else
			DATA_1();
		SCK_1();
		c >>= 1;
	}
	SCK_0();

	// turn off the display
	PORTD = 0;

	// latch the new value
	LATCH_1();

	// turn on this column
	PORTD = (uint8_t)0x80U >> col;

	// next time we'll do the next column
	col = (col + 1) & 7;
}

// use this to fade the display in/out by turning the column off early
ISR(TIMER0_COMPB_vect)
{
	PORTD = 0;
}

// timer0 runs at 125kHz.  we refresh a column when this value is reached.
// 125000 / 157 = 796Hz column refresh = just under 100Hz per column
#define REFRESH 157

// to fade the display, we turn off the display prior to the row refresh.
#define FADE_DARK 1
#define FADE_BRIGHT (REFRESH - 1)
#define FADE_LEVEL(x) OCR0B = x
#define FADE_ON()  TIMSK0 |= (1<<OCIE0B)
#define FADE_OFF()  TIMSK0 &= ~(1<<OCIE0B)

uint16_t adc_sample(void)
{
	ADCSRA |= (1 << ADSC);			// start conversion
	while ((ADCSRA & (1 << ADIF))==0);	// wait for conversion
	ADCSRA |= (1 << ADIF);			// clear the flag
	return ADCW;
}

uint8_t badrand(void)
{
	uint8_t i, r;
	r = 0;
	for(i = 0; i < 8; ++i) {
		r <<= 1;
		r |= (adc_sample() & 1);
		Sleep(MILLIS(2));
	}
	return r;
}

// populates random Life board, only red ("mature") cells
void random_field(uint16_t *p)
{
	uint8_t i, j;
	for(i = 0; i < 8; ++i) {
		uint16_t b = 0;
		for(j = 0; j < 8; ++j) {
			b <<= 2;
			b |= (adc_sample() & 1);
			Sleep(MILLIS(2));
		}
		p[i] = (b << 1);
	}
}

// given the gameboard in src, compute the next generation and store in dst
// returns the state of the cells
#define DEAD 0
#define STEADY 1
#define ACTIVE 2
uint8_t life(uint16_t *src, uint16_t *dst)
{
	uint8_t i, j, ret = DEAD, colstate, deadcols = 0;
	for(i = 0; i < 8; ++i)
	{
		for(j = 0; j < 16; j += 2)
		{
			uint8_t curstate = (src[i] >> j) & 0x03;
			uint8_t nextstate;
			uint8_t L = (i - 1) & 0x07, R = (i + 1) & 0x07, U = (j - 2) & 0x0F, D = (j + 2) & 0x0F;
			uint8_t neighbors =
				!!(src[L] & (3 << U)) +
				!!(src[L] & (3 << j)) +
				!!(src[L] & (3 << D)) +
				!!(src[i] & (3 << U)) +
				!!(src[i] & (3 << D)) +
				!!(src[R] & (3 << U)) +
				!!(src[R] & (3 << j)) +
				!!(src[R] & (3 << D));

			if (curstate)
			{
				if (neighbors < 2 || neighbors > 3)
				{
					nextstate = 0;	// died
				}
				else if (curstate == 1)	// green
				{
					nextstate = 3;	// orange
				}
				else
				{
					nextstate = 2;	// red;
				}
			}
			else if (neighbors == 3)
			{
				nextstate = 1;	// green
			}
			else
			{
				nextstate = 0;	// still dead
			}

			dst[i] &= ~(3 << j);
			dst[i] |= (nextstate << j);
		}
		colstate = (dst[i] == 0) ? DEAD : (dst[i] == src[i]) ? STEADY : ACTIVE;
		if (colstate == DEAD)
			++deadcols;
		if (colstate > ret)
			ret = colstate;
	}
	// hack: if 7 columns are dead, fade out.  single spinners are teh boring.
	//       it would be better to store three (or more) complete states
	//       or perhaps hashes of complete states, or something, so we could
	//       detect cycles and start over.
	return (deadcols == 7) ? STEADY : ret;
}

// here's how button presses work:
// - a press is registered when a button is released.
// - a hold is registered when the same button has been down
//   for a specified number of cycles.  the button release
//   following the hold does not register.

#define BUTTON_LEFT  0x01
#define BUTTON_RIGHT 0x02
#define BUTTON_HOLD  0x10 	// button was held

#define REPEAT_THRESHOLD 20

uint8_t GetButtons(void)
{
	static uint8_t prevState = 0xff;
	static uint8_t repeat = 0;

	uint8_t curState = ((PINC & 0x30) >> 4);

	// if we've already registered a "hold"
	if (repeat >= REPEAT_THRESHOLD) {
		prevState = curState;
		if (curState == 3)
			repeat = 0;	// no buttons are down.
		return 0;
	}

	if (curState != prevState) {
		uint8_t pressed = ~prevState & curState;
		prevState = curState;
		return pressed;
	}
	else if (curState != 3) {
		// button(s) are being held
		if (++repeat == REPEAT_THRESHOLD) {
			return BUTTON_HOLD | ~(curState & 3);
		}
	}

	return 0;
}

const char p0[] PROGMEM = "\"Let it fester for a little bit, have your fun, then give me some relief later\" - CharlesS";
const char p1[] PROGMEM = "\"You did an amazing job, for a Brazilian\" - Romeo";
const char p2[] PROGMEM = "\"Then we can all stand in a dark room and bite it\" - RussS";
const char p3[] PROGMEM = "\"Every time I come up with the coolest thing ever, you say, `we don't need it, throw it away!'\" - stevens";
const char p4[] PROGMEM = "\"We should test people's blood sugar, or sift through random bowel movements\" - cbaconator";
const char p5[] PROGMEM = "\"The problems I'm expecting you to have [with your Mac] are the problems I would expect to see from people using Windows\" - cbacon";
const char p6[] PROGMEM = "\"Whenever I sit on somebody's lap, my tongue immediately comes out.\" - Paul D";
const char p7[] PROGMEM = "\"None of us really know anything\" - jmo";
const char p8[] PROGMEM = "\"I've never squatted so hard in my life. I didn't think I would be able to walk tomorrow.\" - Paul D";
const char p9[] PROGMEM = "\"I am the master drug dealer. I freebase the stuff all day long.\" - cbacon";
const char p10[] PROGMEM = "\"Your mom uses Model View Controller\" - DavidB";
const char p11[] PROGMEM = "\"When I was coming out of the closet\" - ScottL";
const char p12[] PROGMEM = "\"I wouldn't be a good salesman, because I am not good at smooching\" - PanchoA";
const char p13[] PROGMEM = "\"`Seed Device.' That just seems like low hanging fruit.\" - PaulD";
const char p14[] PROGMEM = "\"500 is often greater than 256.\" - bjh";
const char p15[] PROGMEM = "\"Unless you use it for assassinations, it really doesn't make economical sense.\" - pauld";
const char p16[] PROGMEM = "\"If I was a TV-watching person, I'd totally have a duck in my house.\" - Charles";
const char p17[] PROGMEM = "\"You smell like a dog but not in a bad way.\" - Mark M";
const char p18[] PROGMEM = "\"This carrot...it's a very painful carrot.\" - Chris C";
const char p19[] PROGMEM = "\"You guys have Mac Power here?\" - Ted H";
const char p20[] PROGMEM = "\"Strong, like a chicken\" - RyanC";
const char p21[] PROGMEM = "\"It's Milliner time!\" - DavidB";
const char p22[] PROGMEM = "\"How many arteries do you have in your butt?\" - RussS";
const char p23[] PROGMEM = "\"It's getting late earlier these days.\" - PaulD";
const char p24[] PROGMEM = "\"All you want is my sugar.\" - Fernandor";
const char p25[] PROGMEM = "\"Hey there giggle monster!\" - Jamie M.";
const char p26[] PROGMEM = "\"We need to get a picture of 50 engineers with burritos down their pants?\" - ChrisC";
const char p27[] PROGMEM = "\"People, like your wife, who don't think the way WE do...\" - StevenS";
const char p28[] PROGMEM = "\"Remember George Costanza and his hands?  That's my feet.\" - CBacon";
const char p29[] PROGMEM = "\"I don't often drink, but when I do, I do it recklessly and logged in as root.\" - Chuckles";
const char p30[] PROGMEM = "\"It's some good memories since we didn't die\" - BenD";
const char p31[] PROGMEM = "\"The pirates weren't dummies!\" - LanceH";
const char p32[] PROGMEM = "\"Fundamentals are great after you understand everything at a basic level.\" - PaulD";
const char p33[] PROGMEM = "\"I'm not going to let waiting for a baby hold up my life.\" - ScottL (before having a baby)";
const char p34[] PROGMEM = "\"I'm going to make this sharp and put it in your eye. When you start crying like a little girl I will say `See, you are a little girl just like we thought.' - fernandor";
const char p35[] PROGMEM = "\"I will fill my dog's bowl with your tears.\" - fernandor";
const char p36[] PROGMEM = "\"When are you going to be a man and stop crying? Nevermind, the best part of my day is when you cry.\" - fernandor";
const char p37[] PROGMEM = "\"I'm going to solve your face like a Rubik's Cube.\" - fernandor";
const char p38[] PROGMEM = "\"If you don't stop talking I'm going to remove your teeth and then take them for a walk!\" - fernandor";
const char p39[] PROGMEM = "\"You want me to chop off your arm? I'd be happy to do it.\" - fernandor";
const char p40[] PROGMEM = "\"I'm going to put a snake on your face and let it bite it.\" - fernandor";
const char p41[] PROGMEM = "\"My baby can bite your baby to death, and she barely got teeth.\" - fernandor";
const char p42[] PROGMEM = "\"I'm going to make you eat yellow snow\" - fernandor";
const char p43[] PROGMEM = "\"Your mom is a soccer hooligan.\" - fernandor";
const char p44[] PROGMEM = "\"You better watch your neck because when you aren't looking I'll cut it off.\" - fernandor";
const char p45[] PROGMEM = "\"I will pee in my cubicle to mark my territory.\" - fernandor";
const char p46[] PROGMEM = "\"Ridiculous! I'll shave your head on asphalt.\" - fernandor";
const char p47[] PROGMEM = "\"Passwords don't match? Your mom doesn't match.\" - fernandor";
const char p48[] PROGMEM = "\"Have you looked at yourself in the mirror? I don't know how you don't hate yourself.\" - fernandor";
const char p49[] PROGMEM = "\"I will insert your ipad in your head through your ears. That will help you think.\" - fernandor";
const char p50[] PROGMEM = "\"I am happy to bring pain to you.\" - fernandor";
const char p51[] PROGMEM = "\"no, that's not even possible, do I have to teach you where babies come from\" - fernandor";
const char p52[] PROGMEM = "\"I'm going to punch you in the back of the head so hard your eyes will pop out and then I'll hold them up to your face so you can see what a girl you are\" - fernandor";
const char p53[] PROGMEM = "\"OK you guys, I will hurt you with a spoon\" - fernandor";
const char p54[] PROGMEM = "\"Somebody is getting punched in the eye today\" - fernandor";
const char p55[] PROGMEM = "\"If you ever do that again I'll punch you\" - fernandor";
const char p56[] PROGMEM = "\"I will let my dog bite out your hair.\" - fernandor";
const char p57[] PROGMEM = "\"Ok, I am going to shove this pen up your nose into your brain. Then I will pull it out through your mouth.\" - fernandor";
const char p58[] PROGMEM = "\"Santa Claus is going to land on your face!\" - fernandor";
const char p59[] PROGMEM = "\"Don't come and hug me or I will break your nose.\" - fernandor";
const char p60[] PROGMEM = "\"I'm going to make you eat yellow snow.\" - fernandor";
const char p61[] PROGMEM = "\"I think you should hit your heads together until there is blood. This is dumb.\" - fernandor";
const char p62[] PROGMEM = "\"`Whitepaper' is racist.\" - fernandor";
const char p63[] PROGMEM = "\"I'm first going to hit you so hard in the middle section that your head explodes. Then i'm going to bring my dog and let him eat your insides that end up all over the ground. Then I will make you eat the dog's poop. It will be like you're eating yourself.\" - fernandor";

PROGMEM const char *const string_table[64] =
{
  p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p18, p19, p20,
  p21, p22, p23, p24, p25, p26, p27, p28, p29, p30, p31, p32, p33, p34, p35, p36, p37, p38, p39, p40,
  p41, p42, p43, p44, p45, p46, p47, p48, p49, p50, p51, p52, p53, p54, p55, p56, p57, p58, p59, p60, p61, p62, p63
};

void hello_world(void)
{
	// pick randomly, but if the message has been used already,
	// use the next highest message that we haven't yet seen.
	// so no messages repeat until we've seen them all.
	static uint64_t maskus = 0;
	uint8_t r = badrand() & 63, c = badrand() % 3 + 1;
	uint64_t bit;
	if (maskus == 0)
		maskus = ~0ULL;
	for(;;)
	{
		bit = (1ULL << r);
		if (maskus & bit)
			break;
		r = (r + 1) & 63;
	}
	maskus &= ~bit;

	clear_screen(0, 16);
	DrawTextP(pgm_read_word(&(string_table[r])), c);
	DrawText("   ", c);
}

void do_life(void)
{
	unsigned short iterations = 0;
	int8_t df = 2;
	uint8_t fade = FADE_DARK, speed = 8, itc = 0, ktc = 0;
	FADE_LEVEL(fade);
	FADE_ON();

	// initialize the gameboard
	clear_screen(0, 16);		// clear both buffers
	fb_base = 0;			// set the front buffer at 0
	random_field((uint16_t *)&framebuf[8]);	// draw on the back buffer
	fb_base ^= 8;			// flip buffers

	// run...
	for(;;)
	{
		if (++ktc == 5)
		{
			ktc = 0;

			// check input
			uint8_t buttons = GetButtons();
			//if ((buttons & (BUTTON_LEFT | BUTTON_HOLD)) == (BUTTON_LEFT | BUTTON_HOLD))	// exit
			//{
			//	FADE_OFF();
			//	return;
			//}
			//else
			if (buttons & BUTTON_LEFT)	// pause/speed
			{
				itc = 0;
				speed = (speed + 4) & 15;
			}
			else if (buttons & BUTTON_RIGHT)
			{
				// fade out
				if (df == 0) {
					fade = FADE_BRIGHT;
					FADE_LEVEL(fade);
					FADE_ON();
				}
				df = -2;
			}
		}

		// update state
		if (speed > 0 && ++itc == speed)
		{
			itc = 0;
			uint8_t life_state = life((uint16_t *)&framebuf[fb_base], (uint16_t *)&framebuf[fb_base ^ 8]);
			if (life_state != ACTIVE // uinteresting state
				|| ++iterations > 35)  // this pattern getting boring by now
			{
				// fade out, unless the board is already totally dead
				if (df == 0) {
					fade = (life_state == DEAD) ? FADE_DARK : FADE_BRIGHT;
					FADE_LEVEL(fade);
					FADE_ON();
				}
				df = -2;
			}
			// page flip
			fb_base ^= 8;
		}

		// handle fading
		if (df > 0)	// fading in
		{
			if (fade < FADE_BRIGHT)
			{
				fade += df;
				FADE_LEVEL(fade);
			}
			else	// done fading
			{
				df = 0;
				FADE_OFF();
			}
		}
		else if (df < 0)	// fading out
		{
			if (fade > -df)
			{
				fade += df;
				FADE_LEVEL(fade);
			}
			else	// done fading out; now reset the board and start fading in
			{
				break;
				//df = -df;
				//iterations = 0;
				//clear_screen(fb_base, 8);		// clear the front buffer
				//random_field((uint16_t *)&framebuf[fb_base ^ 8]);	// draw random data on the back buffer
				//fb_base ^= 8;				// flip buffers
			}
		}

		Sleep(MILLIS(10));
	}

	clear_screen(0, 16);
	FADE_OFF();
}

int main(void)
{
	// Initialize I/O
	DDRB  = 0x00; // 00000000
	PORTB = 0xff; // 11111111
	DDRC  = 0x07; // 00000111
	PORTC = 0xf0; // 11110000
	DDRD  = 0xff; // 11111111
	PORTD = 0x00; // 00000000

	// Setup the display timer...
	TCCR0A = (1<<WGM01);			// CTC mode
	TCCR0B = (1<<CS01) | (1<<CS00);  	// prescaler 1/64; at 8MHz system clock, this counts at 125kHz.
	OCR0A = REFRESH;
	TIMSK0 = (1<<OCIE0A);			// Enable refresh interrupt

	// Set up the ADC, for hokey RNG generation
	ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS2);
	ADMUX = 3;

	// Enable interrupts
	sei();

	// Do stuff
	for(;;)
	{
		do_life();
		Sleep(200);
		hello_world();
	}
}
