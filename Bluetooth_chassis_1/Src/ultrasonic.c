/*
 * ultrasonic.c
 *
 *  Created on: Jun 11, 2025
 *      Author: pacheco arquitectos
 */


#include "stm32f051x8.h"
#include "ultrasonic.h"

// Pines de los 4 sensores
static const uint8_t trigger_pins[4] = {0, 1, 2, 11}; // PB0, PB1, PB10, PB11
static const uint8_t echo_pins[4]    = {9, 8, 7,  6};  // PB9, PB8, PB7, PB6

static volatile uint32_t echo_time = 0;

static void delay_us(uint32_t us) {
    TIM6->ARR = us;
    TIM6->CNT = 0;
    TIM6->CR1 |= TIM_CR1_CEN;
    while (!(TIM6->SR & TIM_SR_UIF));
    TIM6->SR &= ~TIM_SR_UIF;
    TIM6->CR1 &= ~TIM_CR1_CEN;
}

static void timer6_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 8 - 1;
}

static void timer17_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    TIM17->PSC = 8 - 1;
    TIM17->ARR = 0xFFFF;
    TIM17->CNT = 0;
    TIM17->CR1 |= TIM_CR1_CEN;
}

static void gpio_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    for (int i = 0; i < 4; i++) {
        GPIOB->MODER &= ~(3 << (trigger_pins[i]*2));
        GPIOB->MODER |=  (1 << (trigger_pins[i]*2));
        GPIOB->MODER &= ~(3 << (echo_pins[i]*2));
    }
}

static void ultrasonic_trigger(uint8_t tr_pin) {
    GPIOB->ODR &= ~(1 << tr_pin);
    delay_us(2);
    GPIOB->ODR |= (1 << tr_pin);
    delay_us(10);
    GPIOB->ODR &= ~(1 << tr_pin);
}

static float ultrasonic_measure_one(uint8_t tr_pin, uint8_t ec_pin) {
    ultrasonic_trigger(tr_pin);

    uint32_t timeout = 30000;
    while (!(GPIOB->IDR & (1 << ec_pin)) && --timeout);
    if (!timeout) return -1.0f;
    uint32_t start = TIM17->CNT;

    timeout = 30000;
    while ((GPIOB->IDR & (1 << ec_pin)) && --timeout);
    if (!timeout) return -1.0f;
    uint32_t end = TIM17->CNT;

    echo_time = (end >= start) ? (end - start) : (0xFFFF - start + end);
    return (echo_time * 0.0343f) / 2.0f;
}

void Ultrasonic_Init(void) {
    timer6_init();
    timer17_init();
    gpio_init();
}

float Ultrasonic_MeasureAll(void) {
    float d, min = -1.0f;
    for (int i = 0; i < 4; i++) {
        d = ultrasonic_measure_one(trigger_pins[i], echo_pins[i]);
        if (d > 0 && (min < 0 || d < min)) min = d;
        delay_us(5000); // separación entre sensores
    }
    return min;
}

bool Ultrasonic_Loop(void) {
    float min = Ultrasonic_MeasureAll();
    return (min > 0 && min < UMBRAL_CM);
}
