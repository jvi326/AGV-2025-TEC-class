#include <stdint.h>
#include <stdbool.h>
#include "stm32f051x8.h"
#include "motor_controller.h"
#include "chassis.h"
#include "elevator.h"
#include "ws2812b_dma.h"
#include "ultrasonic.h"



#define MAX_MENSAJE 20
volatile float mensaje[MAX_MENSAJE];
volatile uint8_t mensaje_size = 0;

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


void decideDir(CHASSIS* AGV_Chassis, ELEVATOR* elevator,volatile Numeros* numeros, uint8_t count);

// Communication function prototypes
void USART2_IRQHandler(void);
void USART2_HandleMessage(CHASSIS* AGV_Chassis, ELEVATOR* elevator);
void USART2_SendChar(char c);
void USART2_SendString(const char *str);
void USART2_SendFloat(float value, uint8_t decimalPlaces);

// Hardware configuration functions
void USART2_Init_Interrupt(void);

// System initialization functions
void System_Ready_Indicator(void);


void sensores_init(void);

//boton emergencia
void EXTI4_15_IRQHandler(CHASSIS* AGV_Chassis);
void Boton_Seguridad_Init(void);
volatile bool boton_seguridad_activo = false;


int main(void) {
	USART2_Init_Interrupt();
	System_Ready_Indicator();


	MotorController motorA;
	MotorController motorB;
	CHASSIS agv;
	ELEVATOR elevator;

	// Initialize motor (Dir1, Dir2, PWM, BrakePin)
	Motor_Init(&motorA, 6, 7, 8, 5);
	Motor_Init(&motorB, 10, 11, 9, 4);
	Motor_Invert(&motorB, 1);

	Init_Chassis(&agv,motorA,motorB);
	set_AdvanceInverted(&agv, 1);

	Elevator_Init(&elevator, 3, 2);

	// Initialize TIM14 and DMA
	WS2812B_DMA_Init(TIM14, 1);

	// Set LED colors
	WS2812B_DMA_SetColor(0, WS2812B_RED);
	delay_ms(1000);
	WS2812B_DMA_SetColor(1, WS2812B_GREEN);
	WS2812B_DMA_Update();
	//Initialize Infrarred
	sensores_init();

	//emerrgency button
	//Boton_Seguridad_Init();

	//echos
	Ultrasonic_Init();


    // Test sequence
   while (1) {
    	// if ((GPIOB->IDR & (1 << 4)) == 0) {  // Si PB4 está en 0 → botón presionado
    	  //  	        if (!boton_seguridad_activo) {
    	    //	            stop_Chassis(&agv);
    	    	//            boton_seguridad_activo = true;
    	    	       // }
    	    	  //      continue; // Saltar el resto del código mientras se presione el botón
    	    	//    } else {
    	    	        // Si ya fue presionado y ahora se soltó
    	    	 //       if (boton_seguridad_activo) {
    	    	   //         boton_seguridad_activo = false;

    	    //	        }
    	   // 	    }

    	 if (Ultrasonic_Loop()) {
    	 	            stop_Chassis(&agv);  // Apaga LED si detecta objeto cerca
    	 }

    	if (rx_ready == 1) {
				USART2_HandleMessage(&agv, &elevator);

				// imprimir lo que llegó
				//USART2_SendString("Arreglo mensaje contiene:\r\n");
				//for (uint8_t i = 0; i < mensaje_size; i++) {
				//	USART2_SendFloat(mensaje[i], 2);
				//	USART2_SendString("\r\n");
				//}
			}
    	}
    }


//Boton de seguridad
void Boton_Seguridad_Init(void) {

    // Activar reloj del puerto B
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // PB4 como entrada (00)
    GPIOB->MODER &= ~GPIO_MODER_MODER4;

    // Pull-up interno activado (01)
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR4;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0;

    // Activar SYSCFG
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    // Conectar EXTI4 a PB4
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;

    // Desenmascarar interrupción EXTI4
    EXTI->IMR |= EXTI_IMR_IM4;

    // Activar interrupción por flanco de bajada (cuando se presiona el botón)
    EXTI->FTSR |= EXTI_FTSR_TR4;

    // Limpiar bandera pendiente
    EXTI->PR |= EXTI_PR_PR4;

    // Habilitar interrupción en NVIC
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void EXTI4_15_IRQHandler(CHASSIS* AGV_Chassis) {
    if (EXTI->PR & EXTI_PR_PR4) {
        EXTI->PR |= EXTI_PR_PR4; // Limpiar bandera

        // Apagar los motores y activar la bandera

        stop_Chassis(AGV_Chassis);
        boton_seguridad_activo = true;
    }
}

void sensores_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Activar reloj para GPIOB

    // Configurar PB6, PB7, PB8, PB10 y PB11 como entrada (00)
    GPIOB->MODER &= ~((0x3 << (6 * 2)) | (0x3 << (7 * 2)) |
                      (0x3 << (8 * 2)) | (0x3 << (10 * 2)) | (0x3 << (11 * 2)));
}

void calcular_velocidades(float* vel_izq, float* vel_der) {
    uint16_t lectura = GPIOB->IDR;

    // Sensores conectados a estos pines:
    int pines[5]  = {6, 7, 8, 10, 11};      // Sensor físico
    int pesos[5]  = {-2, -1, 0, 1, 2};      // Peso según la posición
    int suma = 0;
    int activos = 0;

    for (int i = 0; i < 5; i++) {
        if (lectura & (1 << pines[i])) {
            suma += pesos[i];
            activos++;
        }
    }

    if (activos == 0) {
        *vel_izq = 0.3f;
        *vel_der = 0.3f;
        return;
    }

    float error = (float)suma / activos;

    // Aquí ajustamos velocidad base y cuánto corrige
    float velocidad_base = 0.3f;
    float k = 0.2f; // Ganancia de corrección (ajústala si gira muy brusco)

    float ajuste = k * error;

    *vel_izq = velocidad_base - ajuste;
    *vel_der = velocidad_base + ajuste;

    // Limitar el valor entre -1.0 y 1.0
    if (*vel_izq > 1.0f) *vel_izq = 1.0f;
    if (*vel_izq < -1.0f) *vel_izq = -1.0f;
    if (*vel_der > 1.0f) *vel_der = 1.0f;
    if (*vel_der < -1.0f) *vel_der = -1.0f;
}


/* System Initialization Functions */
void System_Ready_Indicator(void) {
    USART2_SendString("\r\n=== STM32 Bluetooth Chassis Demo ===\r\n");
    USART2_SendString("Send numbers csv for instructions\r\n");
}





void USART2_Init_Interrupt(void) {
	// Enable clocks
    RCC->AHBENR  |= (1 << 17);  // GPIOA
    RCC->APB1ENR |= (1 << 17);  // USART2

    // Configure PA2 (TX) and PA3 (RX)
    GPIOA->MODER &= ~((3 << 2*2) | (3 << 2*3));
    GPIOA->MODER |=  ((2 << 2*2) | (2 << 2*3));  // Alternate function mode

    // Set AF1 for USART1
    GPIOA->AFR[0] &= ~((0xF << 4*2) | (0xF << 4*3));
    GPIOA->AFR[0] |=  ((1 << 4*2) | (1 << 4*3));

    // Baud rate 9600 (8MHz clock)
    USART2->BRR = (8000000 / 9600);

    // Enable USART with interrupts
    USART2->CR1 |= (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5);
    // UE: USART Enable
    // RE: Receiver Enable
    // TE: Transmitter Enable
    // RXNEIE: RX Not Empty Interrupt Enable
    USART2->CR1 &= ~(1 << 6);  // Disable TC interrupt, no interruptions when TX is used

    // NVIC configuration
    USART2->ICR = 0xFFFFFFFF;			//Clear all interrups flags
    NVIC_EnableIRQ(USART2_IRQn);		//Enable USART1 global interrupt
    NVIC_SetPriority(USART2_IRQn, 0);	//Set priority
}

/* Communication Functions */
void USART2_IRQHandler(void){
    if(USART2->ISR & (1 << 5)) {
        uint8_t c = USART2->RDR; // Read and clear RXNE flag

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
}

void USART2_HandleMessage(CHASSIS* AGV_Chassis, ELEVATOR* elevator) {


	USART2_SendString("Echo: ");
	USART2_SendString((char*)rx_buf);

	uint8_t numArrays = parseCSV(rx_buf, rx_pos, parsedArrays);
	uint8_t numArrayIntFloat = arrayToArrayIntOrFloat(parsedArrays, numArrays, indicacionesArray);
	decideDir(AGV_Chassis, elevator, indicacionesArray, numArrayIntFloat);

	// Reset for next message
	rx_pos = 0;
	rx_ready = 0;
}


void USART2_SendChar(char c) {
    while (!(USART2->ISR & (1 << 7)));
    USART2->TDR = c;
}

void USART2_SendString(const char *str) {
    while (*str) USART2_SendChar(*str++);
}

void USART2_SendFloat(float value, uint8_t decimalPlaces) {
    // Handle negative numbers
    if (value < 0) {
        USART2_SendChar('-');
        value = -value;
    }

    // Extract integer part
    int integerPart = (int)value;
    char buffer[12];
    USART2_SendString(itoa(integerPart, buffer, 10));  // Send integer part

    // Only send decimal point if needed
    if (decimalPlaces > 0) {
        USART2_SendChar('.');

        // Extract and send fractional part
        float fractionalPart = value - integerPart;
        for (uint8_t i = 0; i < decimalPlaces; i++) {
            fractionalPart *= 10;
            int digit = (int)fractionalPart;
            USART2_SendChar('0' + digit);
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
		USART2_SendString("Number ");
		USART2_SendChar('0' + i);
		USART2_SendString(": ");
		USART2_SendString((const char*)numbersAsArray[i].array);

		numero = atof(numbersAsArray[i].array, numbersAsArray[i].length);

		if (i < 2){
			result[i].i = (int)numero;
		} else {
			result[i].f = numero;
		}
		array_count++;
	}
	return array_count;
}

//Decide if negative o positive


void decideDir(CHASSIS* AGV_Chassis, ELEVATOR* elevator,volatile Numeros* numeros, uint8_t count) {

    // Asegura que no hay elementos extras para la decisión
    if (count < 4) return;

    if (numeros[0].i == 0) {
        // Apagar si el primer valor es 0
        stop_Chassis(AGV_Chassis);
        elevator_SetSpeed(elevator, 0);
        EXTI->IMR &= ~(1 << 1); // Desactiva la interrupción del sensor de proximidad momentáneamente
    }
    else if (numeros[0].i == 1) {
        if (numeros[1].i == 0) {
            stop_Chassis(AGV_Chassis);
            set_CoastMode(AGV_Chassis);
            EXTI->IMR &= ~(1 << 1);
        }
        else if (numeros[1].i == 1) {
            EXTI->IMR |= (1 << 1);
            float avance = numeros[2].f;
            float giro = numeros[3].f;
            float elevador = numeros[4].f;

            if (avance >= -1.0 && avance <= 1.0) {
                set_AdvanceSpeed(AGV_Chassis, avance);
            } else {
                set_AdvanceSpeed(AGV_Chassis, 0);
            }

            if (giro >= -1.0 && giro <= 1.0) {
                set_TurnSpeed(AGV_Chassis, giro);
            } else {
                set_TurnSpeed(AGV_Chassis, 0);
            }

            apply_CurrentSpeedsToMotors(AGV_Chassis);
            elevator_SetSpeed(elevator, elevador);
        }
        else {
            int elevador = numeros[3].f;
            if (numeros[2].f == 1) {
                float vel_izquierda = 0.0f;
                float vel_derecha = 0.0f;


                calcular_velocidades(&vel_izquierda, &vel_derecha);

                Motor_SetSpeed(&AGV_Chassis->wheelLeft, vel_izquierda);
                Motor_SetSpeed(&AGV_Chassis->wheelRight, vel_derecha);
                elevator_SetSpeed(elevator, elevador);
            } else {
                stop_Chassis(AGV_Chassis);
            }
        }
    }
    else {
        stop_Chassis(AGV_Chassis);
    }

}
