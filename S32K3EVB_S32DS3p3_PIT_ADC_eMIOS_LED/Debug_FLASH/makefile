################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include Project_Settings/Startup_Code/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_SIZE += \
S32K3EVB_S32DS3p3_PIT_ADC_eMIOS_LED.siz \


# All Target
all: S32K3EVB_S32DS3p3_PIT_ADC_eMIOS_LED.elf secondary-outputs

# Tool invocations
S32K3EVB_S32DS3p3_PIT_ADC_eMIOS_LED.elf: $(OBJS) C:/Users/nxf60347/worksapceS32DS_3_migration/S32K3EVB_S32DS3p3_PIT_ADC_eMIOS_LED/Project_Settings/Linker_Files/S32K314_flash.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Standard S32DS C Linker'
	arm-none-eabi-gcc -o "S32K3EVB_S32DS3p3_PIT_ADC_eMIOS_LED.elf" "@S32K3EVB_S32DS3p3_PIT_ADC_eMIOS_LED.args"  $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

S32K3EVB_S32DS3p3_PIT_ADC_eMIOS_LED.siz: S32K3EVB_S32DS3p3_PIT_ADC_eMIOS_LED.elf
	@echo 'Invoking: Standard S32DS Print Size'
	arm-none-eabi-size --format=berkeley S32K3EVB_S32DS3p3_PIT_ADC_eMIOS_LED.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) ./*
	-@echo ' '

secondary-outputs: $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
