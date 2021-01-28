#ifndef FONT_H
#define FONT_H

#include <avr/pgmspace.h>

// convert milliseconds to "kiloclocks" for Sleep(), assuming 8MHz system clock
// intended for literals; let the compiler do the floating-point math, not the poor AVR ;)
#define MILLIS(x) ((uint16_t)(7.812 * (x)))

extern volatile uint16_t delay;

// Scrolls the specified text in the specified color,
// delaying for the specified number of kiloclocks between pixels.
// Starts by scrolling the existing frame data to the left,
// and ends when the last character is on the right,
// so you can issue another call to change attributes.
void DrawTextP(const char *text, uint8_t color);
void DrawText(const char *text, uint8_t color);

#endif

