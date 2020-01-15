################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/peripheral_driver/src/pca_0.c 

OBJS += \
./lib/efm8bb1/peripheralDrivers/src/pca_0.OBJ 


# Each subdirectory must supply rules for building sources it contributes
lib/efm8bb1/peripheralDrivers/src/pca_0.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/peripheral_driver/src/pca_0.c
	@echo 'Building file: $<'
	@echo 'Invoking: Keil 8051 Compiler'
	C51 "@$(patsubst %.OBJ,%.__i,$@)" || $(RC)
	@echo 'Finished building: $<'
	@echo ' '

lib/efm8bb1/peripheralDrivers/src/pca_0.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/peripheral_driver/inc/pca_0.h C:/SiliconLabs/SimplicityStudio/v4/developer/toolchains/keil_8051/9.53/INC/ASSERT.H C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/config/efm8_config.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/inc/SI_EFM8BB1_Register_Enums.h C:/SiliconLabs/SimplicityStudio/v4/developer/toolchains/keil_8051/9.53/INC/STDIO.H C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/inc/SI_EFM8BB1_Defs.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/si_toolchain.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/stdint.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/stdbool.h


