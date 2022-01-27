################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include gecko_sdk_3.2.2/platform/service/udelay/src/subdir.mk
-include gecko_sdk_3.2.2/platform/service/system/src/subdir.mk
-include gecko_sdk_3.2.2/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_3.2.2/platform/service/device_init/src/subdir.mk
-include gecko_sdk_3.2.2/platform/emlib/src/subdir.mk
-include gecko_sdk_3.2.2/platform/driver/i2cspm/src/subdir.mk
-include gecko_sdk_3.2.2/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_3.2.2/platform/Device/SiliconLabs/EFR32BG22/Source/GCC/subdir.mk
-include gecko_sdk_3.2.2/platform/Device/SiliconLabs/EFR32BG22/Source/subdir.mk
-include gecko_sdk_3.2.2/hardware/driver/si1133/src/subdir.mk
-include autogen/subdir.mk
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
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "C:\Users\creat\SimplicityStudio\v5_workspace\UV/autogen/linkerfile.ld" -Xlinker --gc-sections -Xlinker -Map="UV.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o UV.axf -Wl,--start-group "./app.o" "./i2c.o" "./main.o" "./si1132.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_emlib_gpio_simple_init.o" "./autogen/sl_event_handler.o" "./gecko_sdk_3.2.2/hardware/driver/si1133/src/sl_si1133.o" "./gecko_sdk_3.2.2/platform/Device/SiliconLabs/EFR32BG22/Source/GCC/startup_efr32bg22.o" "./gecko_sdk_3.2.2/platform/Device/SiliconLabs/EFR32BG22/Source/system_efr32bg22.o" "./gecko_sdk_3.2.2/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_3.2.2/platform/driver/i2cspm/src/sl_i2cspm.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_assert.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_burtc.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_cmu.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_core.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_emu.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_gpio.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_i2c.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_rtcc.o" "./gecko_sdk_3.2.2/platform/emlib/src/em_system.o" "./gecko_sdk_3.2.2/platform/service/device_init/src/sl_device_init_dcdc_s2.o" "./gecko_sdk_3.2.2/platform/service/device_init/src/sl_device_init_emu_s2.o" "./gecko_sdk_3.2.2/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_3.2.2/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_3.2.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_burtc.o" "./gecko_sdk_3.2.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.o" "./gecko_sdk_3.2.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "./gecko_sdk_3.2.2/platform/service/system/src/sl_system_init.o" "./gecko_sdk_3.2.2/platform/service/system/src/sl_system_process_action.o" "./gecko_sdk_3.2.2/platform/service/udelay/src/sl_udelay.o" "./gecko_sdk_3.2.2/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
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
