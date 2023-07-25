################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := ACE_Pilling_Rig_CAN
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
ACE_Pilling_Rig_CAN.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ACE_Pilling_Rig_CAN.list \

OBJCOPY_HEX += \
ACE_Pilling_Rig_CAN.hex \

OBJCOPY_BIN += \
ACE_Pilling_Rig_CAN.bin \


# All Target
all: main-build

# Main-build Target
main-build: ACE_Pilling_Rig_CAN.elf secondary-outputs

# Tool invocations
ACE_Pilling_Rig_CAN.elf: $(OBJS) $(USER_OBJS) D:\Agalya\D\ Drive\5.\ NPD_Projects\23_24\Q2\ACE_Pilling\Workspace\ACE_Pilling_Rig_CAN\STM32L433RCTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "ACE_Pilling_Rig_CAN.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Agalya\D Drive\5. NPD_Projects\23_24\Q2\ACE_Pilling\Workspace\ACE_Pilling_Rig_CAN\STM32L433RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ACE_Pilling_Rig_CAN.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ACE_Pilling_Rig_CAN.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ACE_Pilling_Rig_CAN.list"
	@echo 'Finished building: $@'
	@echo ' '

ACE_Pilling_Rig_CAN.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "ACE_Pilling_Rig_CAN.hex"
	@echo 'Finished building: $@'
	@echo ' '

ACE_Pilling_Rig_CAN.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "ACE_Pilling_Rig_CAN.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) ACE_Pilling_Rig_CAN.bin ACE_Pilling_Rig_CAN.elf ACE_Pilling_Rig_CAN.hex ACE_Pilling_Rig_CAN.list default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
