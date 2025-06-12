################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/chassis.c \
../Src/elevator.c \
../Src/main.c \
../Src/motor_controller.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/ultrasonic.c \
../Src/ws2812b_dma.c 

OBJS += \
./Src/chassis.o \
./Src/elevator.o \
./Src/main.o \
./Src/motor_controller.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/ultrasonic.o \
./Src/ws2812b_dma.o 

C_DEPS += \
./Src/chassis.d \
./Src/elevator.d \
./Src/main.d \
./Src/motor_controller.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/ultrasonic.d \
./Src/ws2812b_dma.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F051R8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/chassis.cyclo ./Src/chassis.d ./Src/chassis.o ./Src/chassis.su ./Src/elevator.cyclo ./Src/elevator.d ./Src/elevator.o ./Src/elevator.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/motor_controller.cyclo ./Src/motor_controller.d ./Src/motor_controller.o ./Src/motor_controller.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/ultrasonic.cyclo ./Src/ultrasonic.d ./Src/ultrasonic.o ./Src/ultrasonic.su ./Src/ws2812b_dma.cyclo ./Src/ws2812b_dma.d ./Src/ws2812b_dma.o ./Src/ws2812b_dma.su

.PHONY: clean-Src

