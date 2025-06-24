// lcd_st7789_library.h

#ifndef LCD_ST7789_H
#define LCD_ST7789_H

#include <stdint.h>
#include "font_5x7.h"

// Define the dimensions of the LCD screen
#define WIDTH 320
#define HEIGHT 240

// Define rotation modes
#define ROTATION_PORTRAIT 0
#define ROTATION_LANDSCAPE 1
#define ROTATION_PORTRAIT_FLIP 2
#define ROTATION_LANDSCAPE_FLIP 3

// Function to initialize the LCD
// This should be called before using any other functions
void lcd_init();

// Function to fill the entire screen with a single color
// color: 16-bit color value
void lcd_fill_color(uint16_t color);

// Function to create a 16-bit color from RGB values
// r, g, b: 8-bit color intensities
// Returns: 16-bit color value
uint16_t create_color(uint8_t r, uint8_t g, uint8_t b);

// Function to draw a single pixel
// x, y: coordinates of the pixel
// color: 16-bit color value
void lcd_draw_pixel(int16_t x, int16_t y, uint16_t color);

// Function to draw a line between two points
// x0, y0: start coordinates
// x1, y1: end coordinates
// color: 16-bit color value
void lcd_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);

// Function to draw an empty rectangle
// x, y: top-left corner coordinates
// w, h: width and height of the rectangle
// color: 16-bit color value
void lcd_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

// Function to draw a filled rectangle
// x, y: top-left corner coordinates
// w, h: width and height of the rectangle
// color: 16-bit color value
void lcd_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

// Function to draw a circle
// x0, y0: center coordinates
// r: radius
// color: 16-bit color value
void lcd_draw_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

// Function to draw a filled circle
// x0, y0: center coordinates
// r: radius
// color: 16-bit color value
void lcd_draw_filled_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

// Function to draw a single character
// x, y: top-left corner coordinates of the character
// c: character to draw
// color: 16-bit foreground color
// bg: 16-bit background color
// size: size multiplier for the character
void lcd_draw_char(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);

// Function to draw a string of text
// x, y: top-left corner coordinates of the text
// text: null-terminated string to draw
// color: 16-bit foreground color
// bg: 16-bit background color
// size: size multiplier for the text
void lcd_draw_text(int16_t x, int16_t y, char *text, uint16_t color, uint16_t bg, uint8_t size);

// Function to set the display rotation
// rotation: Rotation mode (0-3)
//   0: Portrait (0 degrees)
//   1: Landscape (90 degrees)
//   2: Portrait flipped (180 degrees)
//   3: Landscape flipped (270 degrees)
void lcd_set_rotation(uint8_t rotation);

#endif // LCD_ST7789_H
