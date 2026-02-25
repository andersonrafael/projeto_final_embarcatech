#ifndef DISPLAY_H
#define DISPLAY_H

#include "ssd1306.h"
#include <stdio.h>

#define DISPLAY_I2C_SDA_PIN 14
#define DISPLAY_I2C_SCL_PIN 15
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_ADDRESS 0x3C

extern ssd1306_t display;

void display_init();
void clear_display();
void show_display();
void display_text_no_clear(const char *text, uint32_t x, uint32_t y, uint32_t scale);
void display_text(const char *text, uint32_t x, uint32_t y, uint32_t scale);

#endif // DISPLAY_H