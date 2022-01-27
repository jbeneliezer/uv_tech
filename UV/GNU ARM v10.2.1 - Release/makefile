################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: UV.axf

# Tool invocations
UV.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "UV.ld" -nostdlib -Xlinker --gc-sections -Xlinker -Map="UV.map" -mfpu=fpv5-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o UV.axf "./i2c.o" "./si1132.o" 
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: UV.hex'
	arm-none-eabi-objcopy -O ihex "UV.axf" "UV.hex"
	@echo ' '

	@echo 'Building bin file: UV.bin'
	arm-none-eabi-objcopy -O binary "UV.axf" "UV.bin"
	@echo ' '

	@echo 'Building s37 file: UV.s37'
	arm-none-eabi-objcopy -O srec "UV.axf" "UV.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "UV.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) UV.axf
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
