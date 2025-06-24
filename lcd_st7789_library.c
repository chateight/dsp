// lcd_st7789_library.c
// customized to adjust hardware wiring, spi format setting & spi clock freq

#include "lcd_st7789_library.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

// Macro for swapping two values
#define SWAP(a, b) do { typeof(a) temp = a; a = b; b = temp; } while (0)

// Pin definitions for the LCD connection
//#define BL_PIN 13 // Backlight pin
#define DC_PIN 20   // Data/Command pin
#define RST_PIN 21  // Reset pin
#define MOSI_PIN 19 // SPI MOSI pin
#define SCK_PIN 18  // SPI Clock pin
#define CS_PIN 17   // Chip Select pin

// SPI configuration
#define SPI_PORT spi0
#define SPI_BAUDRATE 40000000  // 20 MHz SPI clock

// Function prototypes for internal use
static void lcd_write_command(uint8_t cmd);
static void lcd_write_data(uint8_t data);
static void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

// Initialize the LCD
void lcd_init() {
    // Initialize SPI
    spi_init(SPI_PORT, SPI_BAUDRATE);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);     // to add this line
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SCK_PIN, GPIO_FUNC_SPI);

    // Initialize control pins
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);
    
    gpio_init(DC_PIN);
    gpio_set_dir(DC_PIN, GPIO_OUT);
    
    gpio_init(RST_PIN);
    gpio_set_dir(RST_PIN, GPIO_OUT);

     // Perform hardware reset
    gpio_put(RST_PIN, 1);
    sleep_ms(5);
    gpio_put(RST_PIN, 0);
    sleep_ms(20);
    gpio_put(RST_PIN, 1);
    sleep_ms(150);

    // Send initialization commands
    lcd_write_command(0x11);  // Sleep out
    sleep_ms(120);

    lcd_write_command(0x36);  // Memory Data Access Control
    lcd_write_data(0xA0);     // display rotation

    lcd_write_command(0x3A);  // Interface Pixel Format
    lcd_write_data(0x05);     // 16-bit color

    lcd_write_command(0xB2);  // Porch Setting
    lcd_write_data(0x0C);
    lcd_write_data(0x0C);
    lcd_write_data(0x00);
    lcd_write_data(0x33);
    lcd_write_data(0x33);

    lcd_write_command(0xB7);  // Gate Control
    lcd_write_data(0x35);

    lcd_write_command(0xBB);  // VCOM Setting
    lcd_write_data(0x19);

    lcd_write_command(0xC0);  // LCM Control
    lcd_write_data(0x2C);

    lcd_write_command(0xC2);  // VDV and VRH Command Enable
    lcd_write_data(0x01);

    lcd_write_command(0xC3);  // VRH Set
    lcd_write_data(0x12);

    lcd_write_command(0xC4);  // VDV Set
    lcd_write_data(0x20);

    lcd_write_command(0xC6);  // Frame Rate Control in Normal Mode
    lcd_write_data(0x0F);

    lcd_write_command(0xD0);  // Power Control 1
    lcd_write_data(0xA4);
    lcd_write_data(0xA1);

    lcd_write_command(0xE0);  // Positive Voltage Gamma Control
    lcd_write_data(0xD0);
    lcd_write_data(0x04);
    lcd_write_data(0x0D);
    lcd_write_data(0x11);
    lcd_write_data(0x13);
    lcd_write_data(0x2B);
    lcd_write_data(0x3F);
    lcd_write_data(0x54);
    lcd_write_data(0x4C);
    lcd_write_data(0x18);
    lcd_write_data(0x0D);
    lcd_write_data(0x0B);
    lcd_write_data(0x1F);
    lcd_write_data(0x23);

    lcd_write_command(0xE1);  // Negative Voltage Gamma Control
    lcd_write_data(0xD0);
    lcd_write_data(0x04);
    lcd_write_data(0x0C);
    lcd_write_data(0x11);
    lcd_write_data(0x13);
    lcd_write_data(0x2C);
    lcd_write_data(0x3F);
    lcd_write_data(0x44);
    lcd_write_data(0x51);
    lcd_write_data(0x2F);
    lcd_write_data(0x1F);
    lcd_write_data(0x1F);
    lcd_write_data(0x20);
    lcd_write_data(0x23);

    lcd_write_command(0x21);  // Display Inversion On

    lcd_write_command(0x29);  // Display On
}

// Write a command to the LCD
static void lcd_write_command(uint8_t cmd) {
    gpio_put(CS_PIN, 0);
    gpio_put(DC_PIN, 0);
    spi_write_blocking(SPI_PORT, &cmd, 1);
    gpio_put(CS_PIN, 1);
}

// Write data to the LCD
static void lcd_write_data(uint8_t data) {
    gpio_put(CS_PIN, 0);
    gpio_put(DC_PIN, 1);
    spi_write_blocking(SPI_PORT, &data, 1);
    gpio_put(CS_PIN, 1);
}

// Set the active window for drawing
static void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    lcd_write_command(0x2A);
    lcd_write_data(x1 >> 8);
    lcd_write_data(x1 & 0xFF);
    lcd_write_data(x2 >> 8);
    lcd_write_data(x2 & 0xFF);
    
    lcd_write_command(0x2B);
    lcd_write_data(y1 >> 8);
    lcd_write_data(y1 & 0xFF);
    lcd_write_data(y2 >> 8);
    lcd_write_data(y2 & 0xFF);
    
    lcd_write_command(0x2C);
}

// Fill the entire screen with a single color
void lcd_fill_color(uint16_t color) {
    lcd_set_window(0, 0, WIDTH - 1, HEIGHT - 1);
    
    gpio_put(CS_PIN, 0);
    gpio_put(DC_PIN, 1);
    
    uint8_t color_high = color >> 8;
    uint8_t color_low = color & 0xFF;
    
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        spi_write_blocking(SPI_PORT, &color_high, 1);
        spi_write_blocking(SPI_PORT, &color_low, 1);
    }
    
    gpio_put(CS_PIN, 1);
}

// Create a 16-bit color from RGB values
uint16_t create_color(uint8_t r, uint8_t g, uint8_t b) {
    return (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b >> 3);
}

// Draw a single pixel
void lcd_draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    lcd_set_window(x, y, x, y);
    lcd_write_data(color >> 8);
    lcd_write_data(color & 0xFF);
}

// Draw a line using Bresenham's algorithm
void lcd_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        SWAP(x0, y0);
        SWAP(x1, y1);
    }

    if (x0 > x1) {
        SWAP(x0, x1);
        SWAP(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) {
        if (steep) {
            lcd_draw_pixel(y0, x0, color);
        } else {
            lcd_draw_pixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

// Draw an empty rectangle
void lcd_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    lcd_draw_line(x, y, x+w-1, y, color);
    lcd_draw_line(x, y+h-1, x+w-1, y+h-1, color);
    lcd_draw_line(x, y, x, y+h-1, color);
    lcd_draw_line(x+w-1, y, x+w-1, y+h-1, color);
}

// Draw a filled rectangle
void lcd_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    for (int16_t i = x; i < x + w; i++) {
        lcd_draw_line(i, y, i, y+h-1, color);
    }
}


// Draw a circle using Bresenham's circle algorithm
void lcd_draw_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    lcd_draw_pixel(x0, y0 + r, color);
    lcd_draw_pixel(x0, y0 - r, color);
    lcd_draw_pixel(x0 + r, y0, color);
    lcd_draw_pixel(x0 - r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        lcd_draw_pixel(x0 + x, y0 + y, color);
        lcd_draw_pixel(x0 - x, y0 + y, color);
        lcd_draw_pixel(x0 + x, y0 - y, color);
        lcd_draw_pixel(x0 - x, y0 - y, color);
        lcd_draw_pixel(x0 + y, y0 + x, color);
        lcd_draw_pixel(x0 - y, y0 + x, color);
        lcd_draw_pixel(x0 + y, y0 - x, color);
        lcd_draw_pixel(x0 - y, y0 - x, color);
    }
}

// Draw a filled circle using an optimized algorithm
void lcd_draw_filled_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    lcd_draw_line(x0 - r, y0, x0 + r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        lcd_draw_line(x0 - x, y0 + y, x0 + x, y0 + y, color);
        lcd_draw_line(x0 - x, y0 - y, x0 + x, y0 - y, color);
        lcd_draw_line(x0 - y, y0 + x, x0 + y, y0 + x, color);
        lcd_draw_line(x0 - y, y0 - x, x0 + y, y0 - x, color);
    }
}


// Draw a single character
void lcd_draw_char(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg, uint8_t size) {
    if ((x >= WIDTH) || (y >= HEIGHT) || ((x + 6 * size - 1) < 0) || ((y + 8 * size - 1) < 0))
        return;

    for (int8_t i = 0; i < 5; i++) {
        uint8_t line = font_5x7[c - 0x20][i];
        for (int8_t j = 0; j < 8; j++) {
            if (line & 0x01) {
                if (size == 1)
                    lcd_draw_pixel(x + i, y + j, color);
                else
                    lcd_fill_rect(x + (i * size), y + (j * size), size, size, color);
            } else if (bg != color) {
                if (size == 1)
                    lcd_draw_pixel(x + i, y + j, bg);
                else
                    lcd_fill_rect(x + (i * size), y + (j * size), size, size, bg);
            }
            line >>= 1;
        }
    }
}

// Draw a string of text
void lcd_draw_text(int16_t x, int16_t y, char *text, uint16_t color, uint16_t bg, uint8_t size) {
    int16_t cursor_x = x;
    int16_t cursor_y = y;
    
    while (*text) {
        if (*text == '\n') {
            cursor_x = x;
            cursor_y += size * 8;
        } else if (*text == '\r') {
            // skip
        } else {
            lcd_draw_char(cursor_x, cursor_y, *text, color, bg, size);
            cursor_x += size * 6;
            if (cursor_x > WIDTH) {
                cursor_x = x;
                cursor_y += size * 8;
            }
        }
        text++;
    }
}

// Set the rotation
void lcd_set_rotation(uint8_t rotation) {
    lcd_write_command(0x36);  // MADCTL (Memory Access Control)
    
    switch (rotation) {
        case ROTATION_PORTRAIT:
            lcd_write_data(0x00);  // Normal portrait mode
            break;
        case ROTATION_LANDSCAPE:
            lcd_write_data(0x60);  // Rotate 90 degrees (landscape)
            break;
        case ROTATION_PORTRAIT_FLIP:
            lcd_write_data(0xC0);  // Rotate 180 degrees (portrait flip)
            break;
        case ROTATION_LANDSCAPE_FLIP:
            lcd_write_data(0xA0);  // Rotate 270 degrees (landscape flip)
            break;
        default:
            lcd_write_data(0x00);  // Default to normal portrait mode
            break;
    }
}
