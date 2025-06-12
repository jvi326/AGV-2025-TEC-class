#ifndef WS2812B_DMA_H
#define WS2812B_DMA_H

#include "stm32f051x8.h"

#define LED_COUNT 10       // Number of LEDs
#define BYTES_PER_LED 24   // 8 bits × 3 colors (GRB)
#define RESET_BYTES 50     // 50µs reset
#define DMA_BUFFER_SIZE (LED_COUNT * BYTES_PER_LED + RESET_BYTES)

typedef struct {
    uint8_t green;
    uint8_t red;
    uint8_t blue;
} WS2812B_Color_t;

// Function prototypes
void WS2812B_DMA_Init(TIM_TypeDef* tim, uint16_t channel);
void WS2812B_DMA_SetColor(uint16_t led, WS2812B_Color_t color);
void WS2812B_DMA_Update(void);
void WS2812B_DMA_ClearAll(void);

// Predefined colors
#define WS2812B_OFF   ((WS2812B_Color_t){0, 0, 0})
#define WS2812B_RED   ((WS2812B_Color_t){0, 255, 0})
#define WS2812B_GREEN ((WS2812B_Color_t){255, 0, 0})
#define WS2812B_BLUE  ((WS2812B_Color_t){0, 0, 255})

#endif
