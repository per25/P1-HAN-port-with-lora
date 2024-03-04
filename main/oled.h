// oled.h

#ifndef OLED_H
#define OLED_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations
void oled_init();
void display_oled(char *text);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
