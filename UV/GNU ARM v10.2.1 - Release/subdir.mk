################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../i2c.c \
../si1132.c 

OBJS += \
./i2c.o \
./si1132.o 

C_DEPS += \
./i2c.d \
./si1132.d 


# Each subdirectory must supply rules for building sources it contributes
i2c.o: ../i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 -O2 -Wall -ffunction-sections -fdata-sections -mfpu=fpv5-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"i2c.d" -MT"i2c.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

si1132.o: ../si1132.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 -O2 -Wall -ffunction-sections -fdata-sections -mfpu=fpv5-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"si1132.d" -MT"si1132.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


