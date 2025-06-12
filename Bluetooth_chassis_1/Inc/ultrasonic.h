/*
 * ultrasonic.h
 *
 *  Created on: Jun 11, 2025
 *      Author: pacheco arquitectos
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

// Distancia umbral en cm (ajustable antes de compilar)
#define UMBRAL_CM 20.0f

// Prototipos públicos
void Ultrasonic_Init(void);
float Ultrasonic_MeasureAll(void);
bool Ultrasonic_Loop(void);

#endif // ULTRASONIC_H

