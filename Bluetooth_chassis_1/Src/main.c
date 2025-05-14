#include <stdint.h>
#include <stdbool.h>
#include "stm32f051x8.h"
#include "motor_controller.h"
#include "chassis.h"

// Buffer and status variables
#define RX_BUF_SIZE 32
volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint16_t rx_pos = 0;
volatile uint8_t rx_ready = 0;
volatile uint8_t bt_ready = 0;

// Digits logic variables
#define MAX_NUMBERS 10      // Maximum numbers to store
#define MAX_DIGITS 5        // Maximum digits per number
#define MAX_ARRAYS 10   	// Maximum sub arrays to create

// Structure to hold parsed number arrays
typedef struct {
    uint8_t array[MAX_DIGITS + 1];  // +1 for null terminator or newline
    uint8_t length;
} NumberArray;

// Structure to hold numbers extracted from USART_IRQ, either as float or integer
typedef union {
    int i;
    float f;
} Numeros;

// Global storage for parsed arrays and numbers ()integer or float
volatile NumberArray parsedArrays[MAX_ARRAYS];
volatile uint8_t numParsedArrays = 0;

volatile Numeros indicacionesArray[MAX_NUMBERS];
volatile uint8_t numindicaciones = 0;

// Variables to store ADC measurements
volatile unsigned int analogRead1=0; //Variable para bit de lectura adc
volatile unsigned int adc_data_ready=0; //Variable para indicar si lectura de ADC esta lista

// Break numbers in array function prototypes
uint8_t parseCSV(const volatile uint8_t* rx_buf, uint16_t buf_size, volatile NumberArray* result);
uint8_t arrayToArrayIntOrFloat(volatile NumberArray* numbersAsArray, uint8_t numberOfArrays, volatile Numeros* result);

// Utility function prototypes
void delay_ms(uint32_t ms);
int atoi(uint8_t* data, int size);
char* itoa(int num, char* str, int base);
float atof(volatile uint8_t* data, int size);
int strncmp(const char *s1, const char *s2, int n);

void decideNegPos(volatile Numeros* numeros, uint8_t count);

// Communication function prototypes
void USART1_IRQHandler(void);
void USART1_HandleMessage(void);
void USART1_SendChar(char c);
void USART1_SendString(const char *str);
void USART1_SendFloat(float value, uint8_t decimalPlaces);

// Hardware configuration functions
void USART1_Init_Interrupt(void);
void LED_Init(void);

// System initialization functions
void System_Ready_Indicator(void);

int main(void) {
	USART1_Init_Interrupt();
	//LED_Init();

	//System_Ready_Indicator();

    // Initialize motor (Dir1, Dir2, PWM, BrakePin)
    MotorController leftMotor;
    MotorController rightMotor;
    Motor_Init(&leftMotor, 6, 7, 8, 5);
    Motor_Init(&rightMotor, 11, 10, 9, 4);

    Motor_Invert(&rightMotor, 1);

    CHASSIS agv;
    Init_Chassis(&agv, leftMotor, rightMotor);


    // Test sequence
    while (1) {

        // === Motion Test Routine ===

        // 1. Go forward
        set_AdvanceSpeed(&agv, 0.8f);
        set_TurnSpeed(&agv, 0.0f);
        apply_CurrentSpeedsToMotors(&agv);
        delay_ms(2000);

        // 2. Go backward
        set_AdvanceSpeed(&agv, -0.8f);
        set_TurnSpeed(&agv, 0.0f);
        apply_CurrentSpeedsToMotors(&agv);
        delay_ms(2000);

        // 3. Turn right in place
        set_AdvanceSpeed(&agv, 0.0f);
        set_TurnSpeed(&agv, 1.0f);
        apply_CurrentSpeedsToMotors(&agv);
        delay_ms(2000);

        // 4. Turn left in place
        set_AdvanceSpeed(&agv, 0.0f);
        set_TurnSpeed(&agv, -1.0f);
        apply_CurrentSpeedsToMotors(&agv);
        delay_ms(2000);

        // 5. Arc turn right (fast right wheel, slow left wheel)
        set_AdvanceSpeed(&agv, 0.6f);
        set_TurnSpeed(&agv, 0.5f);
        apply_CurrentSpeedsToMotors(&agv);
        delay_ms(2000);

        // 6. Arc turn left (fast left wheel, slow right wheel)
        set_AdvanceSpeed(&agv, 0.6f);
        set_TurnSpeed(&agv, -0.5f);
        apply_CurrentSpeedsToMotors(&agv);
        delay_ms(2000);

        // Final brake
        set_BrakeMode(&agv);
        reset_ChassisSpeeds(&agv);
        apply_CurrentSpeedsToMotors(&agv);

    	if (rx_ready == 1) {
    		USART1_HandleMessage();
    	}
    }
}

/* System Initialization Functions */
void System_Ready_Indicator(void) {
    // Blink pattern: 3 fast blinks
    for (int i = 0; i < 3; i++) {
        GPIOC->ODR |= ((1<<8) | (1<<9));
        delay_ms(100);
        GPIOC->ODR &= ~((1<<8) | (1<<9));
        delay_ms(100);
    }

    USART1_SendString("\r\n=== STM32 Bluetooth Chassis Demo ===\r\n");
    USART1_SendString("Send numbers csv for instructions\r\n");
}

/* Hardware Configuration Functions */
void LED_Init(void) {
    // Enable GPIOC clock
    RCC->AHBENR |= (1 << 19);

    // Configure PC8 (Blue) and PC9 (Green) as outputs
    // Configure PC7 (Blue / even) and PC6 (Red / odd) as outputs
    GPIOC->MODER &= ~( (3 << (2 * 8)) | (3 << (2 * 9)) | (3 << (2 * 7)) | (3 << (2 * 6)) );
    GPIOC->MODER |=  ( (1 << (2 * 8)) | (1 << (2 * 9)) | (1 << (2 * 7)) | (1 << (2 * 6)) );


    // Turn off LEDs initially
    GPIOC->ODR &= ~((1 << 8) | (1 << 9) | (1 << 7) | (1 << 6));
}

void USART1_Init_Interrupt(void) {
	// Enable clocks
    RCC->AHBENR  |= (1 << 17);  // GPIOA
    RCC->APB2ENR |= (1 << 14);  // USART1

    // Configure PA9 (TX) and PA10 (RX)
    GPIOA->MODER &= ~((3 << 18) | (3 << 20));
    GPIOA->MODER |=  ((2 << 18) | (2 << 20));  // Alternate function mode

    // Set AF1 for USART1
    GPIOA->AFR[1] &= ~((0xF << 4) | (0xF << 8));
    GPIOA->AFR[1] |=  ((1 << 4) | (1 << 8));

    // Baud rate 9600 (8MHz clock)
    USART1->BRR = (8000000 / 9600);

    // Enable USART with interrupts
    USART1->CR1 |= (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5);
    // UE: USART Enable
    // RE: Receiver Enable
    // TE: Transmitter Enable
    // RXNEIE: RX Not Empty Interrupt Enable
    USART1->CR1 &= ~(1 << 6);  // Disable TC interrupt, no interruptions when TX is used

    // NVIC configuration
    USART1->ICR = 0xFFFFFFFF;			//Clear all interrups flags
    NVIC_EnableIRQ(USART1_IRQn);		//Enable USART1 global interrupt
    NVIC_SetPriority(USART1_IRQn, 0);	//Set priority
}

/* Communication Functions */
void USART1_IRQHandler(void){
    if(USART1->ISR & (1 << 5)) {
        uint8_t c = USART1->RDR; // Read and clear RXNE flag

        if(rx_pos < RX_BUF_SIZE-1) {
            rx_buf[rx_pos++] = c;
            if(c == '\n' || c == '\r') {
                rx_buf[rx_pos] = '\0';
                rx_ready = 1;
            }
        } else {
            rx_pos = 0; // Reset on overflow
        }
    }
    // Also clear any other potential flags
	//USART1->ICR = 0xFFFFFFFF;
}

void USART1_HandleMessage(void) {
	USART1_SendString("Echo: ");
	USART1_SendString((char*)rx_buf);

	uint8_t numArrays = parseCSV(rx_buf, rx_pos, parsedArrays);
	uint8_t numArrayIntFloat = arrayToArrayIntOrFloat(parsedArrays, numArrays, indicacionesArray);
	decideNegPos(indicacionesArray, numArrayIntFloat);

	// Reset for next message
	rx_pos = 0;
	rx_ready = 0;
}

void USART1_SendChar(char c) {
    while (!(USART1->ISR & (1 << 7)));
    USART1->TDR = c;
}

void USART1_SendString(const char *str) {
    while (*str) USART1_SendChar(*str++);
}

void USART1_SendFloat(float value, uint8_t decimalPlaces) {
    // Handle negative numbers
    if (value < 0) {
        USART1_SendChar('-');
        value = -value;
    }

    // Extract integer part
    int integerPart = (int)value;
    char buffer[12];
    USART1_SendString(itoa(integerPart, buffer, 10));  // Send integer part

    // Only send decimal point if needed
    if (decimalPlaces > 0) {
        USART1_SendChar('.');

        // Extract and send fractional part
        float fractionalPart = value - integerPart;
        for (uint8_t i = 0; i < decimalPlaces; i++) {
            fractionalPart *= 10;
            int digit = (int)fractionalPart;
            USART1_SendChar('0' + digit);
            fractionalPart -= digit;
        }
    }
}

/* Utility Functions */
void delay_ms(uint32_t ms) {
	/**/
    RCC->APB1ENR |= (1<<0); //Enable clock for TIM2

    TIM2->PSC = 8000000/1000 - 1; //Set 1kHz period

    TIM2->ARR = ms;	//Set goal, user defined, by miliseconds

    TIM2->CNT = 0; //Clears the counter

    TIM2->CR1 |= (1<<0); //Count enable

    while (!(TIM2->SR & TIM_SR_UIF)); //Bit set by hardware when the registers are updated

    TIM2->SR &= ~(1<<0); //Cleared by software
    TIM2->CR1 &= ~(1<<0); //Disable counter
    RCC->APB1ENR &= ~(1<<0); //Disable timer TIM2
}

// Array to integer
int atoi(uint8_t* data, int size) {
    int result = 0;
    bool isNegative = false;
    int i = 0;

    if (size == 0) {
        return 0;  // Buffer vacío
    }

    // Verificar signo negativo (si el primer byte es '-')
    if (data[0] == '-') {
        isNegative = true;
        i = 1;
    }

    // Procesar cada byte hasta encontrar '\n', '\r' o fin del buffer
    for (; i < size; ++i) {
        // Si encuentra un fin de línea, terminar
        if (data[i] == '\n' || data[i] == '\r') {
            break;
        }

        // Verificar que sea un dígito válido (0-9)
        if (data[i] < '0' || data[i] > '9') {
            return 0;  // Carácter no válido
        }
        result = result * 10 + (data[i] - '0');
    }

    if (isNegative) {
        result = -result;
    }

    return result;
}

// Integer to array
char* itoa(int num, char* str, int base) {
    int i = 0;
    bool isNegative = false;

    // Handle 0 explicitly
    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }

    // Handle negative numbers (only for base 10)
    if (num < 0 && base == 10) {
        isNegative = true;
        num = -num;
    }

    // Process individual digits
    while (num != 0) {
        int rem = num % base;
        str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
        num = num / base;
    }

    // Append negative sign (if needed)
    if (isNegative) {
        str[i++] = '-';
    }

    // Reverse the string
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }

    // Null-terminate the string
    str[i] = '\0';
    return str;
}

// Array to float
float atof(volatile uint8_t* data, int size) {
    float result = 0.0;
    bool isNegative = false;
    bool hasDecimal = false;
    float fractionMultiplier = 0.1;
    int i = 0;

    if (size == 0) {
        return 0.0;  // Empty buffer
    }

    // Check for sign
    if (data[0] == '-') {
        isNegative = true;
        i = 1;
    }

    // Process each byte
    for (; i < size; ++i) {
        // If we find a line terminator, stop
        if (data[i] == '\n' || data[i] == '\r') {
            break;
        }

        // Check for decimal point
        if (data[i] == '.') {
            if (hasDecimal) {
                return 0.0;  // Multiple decimal points, invalid
            }
            hasDecimal = true;
            continue;
        }

        // Verify it's a valid digit
        if (data[i] < '0' || data[i] > '9') {
            return 0.0;  // Invalid character
        }

        if (hasDecimal) {
            // Fractional part
            result += (data[i] - '0') * fractionMultiplier;
            fractionMultiplier *= 0.1;
        } else {
            // Integer part
            result = result * 10.0 + (data[i] - '0');
        }
    }

    if (isNegative) {
        result = -result;
    }

    return result;
}

// String comparition
int strncmp(const char *s1, const char *s2, int n) {
    while (n-- && *s1 && (*s1 == *s2)) {
        s1++;
        s2++;
    }
    return *(unsigned char *)s1 - *(unsigned char *)s2;
}

// Function to parse rx_buf into separate arrays
uint8_t parseCSV(const volatile uint8_t* rx_received, uint16_t buf_size, volatile NumberArray* result){
	uint8_t array_count = 0;
	uint8_t digit_pos = 0;
	bool new_number = true;

	// Initialize first array
	result[array_count].length = 0;

	for (uint16_t i = 0; i < buf_size; i++) {
		uint8_t c = rx_received[i];

		// Skip leading whitespace (optional)
		if (c == ' ' && new_number) continue;

		// Handle digits
		if ((c >= '0' && c <= '9')||(c=='.')||(c=='-')) {
			if (digit_pos < MAX_DIGITS) {
				result[array_count].array[digit_pos++] = c;
				new_number = false;
			}
		}
		// Handle comma or newline
		else if (c == ',' || c == '\n' || c == '\r') {
			if (!new_number) {  // Only close array if we have a number
				// Add terminator
				result[array_count].array[digit_pos] = '\n';
				result[array_count].length = digit_pos + 1;
				array_count++;

				// Prepare next array
				if (array_count < MAX_ARRAYS) {
					digit_pos = 0;
					result[array_count].length = 0;
					new_number = true;
				} else {
					break;  // Reached maximum arrays
				}
			}

			if (c == '\n' || c == '\r') {
				// Ensure we create a new array even if no digits before newline
				if (array_count > 0 && result[array_count-1].array[result[array_count-1].length-1] != '\n') {
					result[array_count].array[0] = '\n';
					result[array_count].length = 1;
					array_count++;
				}
			}
		}
	}

	// Handle last number if buffer ends without comma/newline
	if (!new_number && array_count < MAX_ARRAYS) {
		result[array_count].array[digit_pos] = '\n';
		result[array_count].length = digit_pos + 1;
		array_count++;
	}

	return array_count;
}

// Turn texts array for USART communication to int and floats array for motor controlling
uint8_t arrayToArrayIntOrFloat(volatile NumberArray* numbersAsArray, uint8_t numberOfArrays, volatile Numeros* result){
	uint8_t array_count = 0;
	float numero;

	// Process the parsed arrays
	for (uint8_t i = 0; i < numberOfArrays; i++) {
		USART1_SendString("Number ");
		USART1_SendChar('0' + i);
		USART1_SendString(": ");
		USART1_SendString((const char*)numbersAsArray[i].array);

		numero = atof(numbersAsArray[i].array, numbersAsArray[i].length);

		if (i == 0){
			result[i].i = (int)numero;
		} else {
			result[i].f = numero;
		}
		array_count++;
	}
	return array_count;
}

//Decide if negative o positive
void decideNegPos(volatile Numeros* numeros, uint8_t count) {
 	for (uint8_t i = 0; i < count; i++){
 		float value;

 		if (i==0){
 			value = (float)numeros[i].i;
 		} else {
 			value = numeros[i].f;
 		}

		if (value < 0) {
			GPIOC->ODR |= (1<<7);
			GPIOC->ODR &= ~(1<<6);
			USART1_SendString("NEGATIVE\r\n");
		} else {
			GPIOC->ODR |= (1<<6);
			GPIOC->ODR &= ~(1<<7);
			USART1_SendString("POSITIVE\r\n");
		}
		delay_ms(200);
		GPIOC->ODR &= ~(1<<7);
		GPIOC->ODR &= ~(1<<6);
		delay_ms(200);
	}
}
