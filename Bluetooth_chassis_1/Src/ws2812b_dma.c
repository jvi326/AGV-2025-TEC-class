#include "ws2812b_dma.h"
#include <string.h>

static uint8_t dmaBuffer[DMA_BUFFER_SIZE];
static WS2812B_Color_t ledColors[LED_COUNT];
static TIM_TypeDef* ws2812b_tim;

// Convert color to bit pattern (0=0.4µs high, 1=0.8µs high)
static void colorToBits(uint8_t* bits, WS2812B_Color_t color) {
    uint8_t colors[3] = {color.green, color.red, color.blue};
    for (uint8_t c = 0; c < 3; c++) {
        for (int8_t b = 7; b >= 0; b--) {
            *bits++ = (colors[c] & (1 << b)) ? 0x70 : 0x30;
        }
    }
}

void WS2812B_DMA_Init(TIM_TypeDef* tim, uint16_t channel) {
    ws2812b_tim = tim;

    // TIM14 @48MHz → 1.25µs period (800kHz)
    tim->ARR = 59;  // 48MHz / 800kHz - 1
    tim->PSC = 0;
    
    // Enable TIM14 and DMA
    tim->CR1 |= TIM_CR1_CEN;
    tim->DIER |= TIM_DIER_UDE;  // DMA on update event
    
    WS2812B_DMA_ClearAll();
}

void WS2812B_DMA_SetColor(uint16_t led, WS2812B_Color_t color) {
    if (led < LED_COUNT) ledColors[led] = color;
}

void WS2812B_DMA_Update(void) {
    uint8_t* ptr = dmaBuffer;

    // Convert colors to bitstream
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        colorToBits(ptr, ledColors[i]);
        ptr += BYTES_PER_LED;
    }
    
    // Reset signal (50µs low)
    memset(ptr, 0, RESET_BYTES);
    
    // Start DMA transfer
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;  // Disable DMA
    DMA1_Channel1->CMAR = (uint32_t)dmaBuffer;
    DMA1_Channel1->CPAR = (uint32_t)&ws2812b_tim->CCR1;
    DMA1_Channel1->CNDTR = DMA_BUFFER_SIZE;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_EN;
}

void WS2812B_DMA_ClearAll(void) {
    memset(ledColors, 0, sizeof(ledColors));
    WS2812B_DMA_Update();
}
