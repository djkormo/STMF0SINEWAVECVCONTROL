################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/_write.c \
../src/algorithm.c \
../src/config.c \
../src/main.c \
../src/resources.c \
../src/stm32f0_discovery.c 

OBJS += \
./src/_write.o \
./src/algorithm.o \
./src/config.o \
./src/main.o \
./src/resources.o \
./src/stm32f0_discovery.o 

C_DEPS += \
./src/_write.d \
./src/algorithm.d \
./src/config.d \
./src/main.d \
./src/resources.d \
./src/stm32f0_discovery.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra  -g -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F051 -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f0-stdperiph" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


