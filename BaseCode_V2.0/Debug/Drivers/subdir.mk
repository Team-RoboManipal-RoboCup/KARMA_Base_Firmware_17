################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/bno085.c \
../Drivers/hw_encoders.c \
../Drivers/i2c_recovery.c \
../Drivers/motor_driver.c \
../Drivers/optical_flow_tof.c 

OBJS += \
./Drivers/bno085.o \
./Drivers/hw_encoders.o \
./Drivers/i2c_recovery.o \
./Drivers/motor_driver.o \
./Drivers/optical_flow_tof.o 

C_DEPS += \
./Drivers/bno085.d \
./Drivers/hw_encoders.d \
./Drivers/i2c_recovery.d \
./Drivers/motor_driver.d \
./Drivers/optical_flow_tof.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/%.o Drivers/%.su Drivers/%.cyclo: ../Drivers/%.c Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I.. -I../Core/Inc -I../Drivers -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../state_machines -I../tasks -I../ThirdParty/FreeRTOS/Source/include -I../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers

clean-Drivers:
	-$(RM) ./Drivers/bno085.cyclo ./Drivers/bno085.d ./Drivers/bno085.o ./Drivers/bno085.su ./Drivers/hw_encoders.cyclo ./Drivers/hw_encoders.d ./Drivers/hw_encoders.o ./Drivers/hw_encoders.su ./Drivers/i2c_recovery.cyclo ./Drivers/i2c_recovery.d ./Drivers/i2c_recovery.o ./Drivers/i2c_recovery.su ./Drivers/motor_driver.cyclo ./Drivers/motor_driver.d ./Drivers/motor_driver.o ./Drivers/motor_driver.su ./Drivers/optical_flow_tof.cyclo ./Drivers/optical_flow_tof.d ./Drivers/optical_flow_tof.o ./Drivers/optical_flow_tof.su

.PHONY: clean-Drivers

