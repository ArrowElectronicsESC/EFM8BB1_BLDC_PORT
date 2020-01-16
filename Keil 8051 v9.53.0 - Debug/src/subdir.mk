################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
A51_UPPER_SRCS += \
C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/SILABS_STARTUP.A51 

C_SRCS += \
../src/EFM8BB1_BLDC_PORT.c \
../src/InitDevice.c \
../src/Interrupts.c \
../src/pca.c 

OBJS += \
./src/EFM8BB1_BLDC_PORT.OBJ \
./src/InitDevice.OBJ \
./src/Interrupts.OBJ \
./src/SILABS_STARTUP.OBJ \
./src/pca.OBJ 


# Each subdirectory must supply rules for building sources it contributes
src/%.OBJ: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Keil 8051 Compiler'
	C51 "@$(patsubst %.OBJ,%.__i,$@)" || $(RC)
	@echo 'Finished building: $<'
	@echo ' '

src/EFM8BB1_BLDC_PORT.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/kits/common/bsp/bsp.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/InitDevice.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/peripheral_driver/inc/pca_0.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/kits/EFM8BB1_LCK/config/bsp_config.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/config/efm8_config.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/inc/SI_EFM8BB1_Register_Enums.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/inc/SI_EFM8BB1_Defs.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/si_toolchain.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/stdint.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/stdbool.h

src/InitDevice.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/inc/SI_EFM8BB1_Register_Enums.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/InitDevice.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/inc/SI_EFM8BB1_Defs.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/si_toolchain.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/stdint.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/stdbool.h

src/Interrupts.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/inc/SI_EFM8BB1_Register_Enums.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/bldcdk.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/inc/SI_EFM8BB1_Defs.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/compiler_defs.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/InitDevice.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/kits/common/drivers/efm8_retargetserial/retargetserial.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/kits/common/bsp/bsp.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/peripheral_driver/inc/pca_0.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/pca.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/motor.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/BLDC_RD_Build_Params.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/BLDC_RD_System.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/si_toolchain.h C:/SiliconLabs/SimplicityStudio/v4/developer/toolchains/keil_8051/9.53/INC/STDIO.H C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/kits/EFM8BB1_LCK/config/bsp_config.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/config/efm8_config.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/stdint.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/stdbool.h

src/SILABS_STARTUP.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/SILABS_STARTUP.A51
	@echo 'Building file: $<'
	@echo 'Invoking: Keil 8051 Assembler'
	AX51 "@$(patsubst %.OBJ,%.__ia,$@)" || $(RC)
	@echo 'Finished building: $<'
	@echo ' '

src/pca.OBJ: C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/bldcdk.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/inc/SI_EFM8BB1_Register_Enums.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/compiler_defs.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/InitDevice.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/kits/common/drivers/efm8_retargetserial/retargetserial.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/kits/common/bsp/bsp.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/peripheral_driver/inc/pca_0.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/pca.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/motor.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/BLDC_RD_Build_Params.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/BLDC_RD_System.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/EFM8BB1/inc/SI_EFM8BB1_Defs.h C:/SiliconLabs/SimplicityStudio/v4/developer/toolchains/keil_8051/9.53/INC/STDIO.H C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/kits/EFM8BB1_LCK/config/bsp_config.h C:/Users/A92862/SimplicityStudio/v4_workspace/EFM8BB1_BLDC_PORT/inc/config/efm8_config.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/si_toolchain.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/stdint.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.7/Device/shared/si8051Base/stdbool.h


