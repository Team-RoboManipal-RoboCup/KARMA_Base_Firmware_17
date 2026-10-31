################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/Segger/SEGGER_RTT.c \
../ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_GCC.c \
../ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_IAR.c \
../ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_KEIL.c \
../ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_SES.c \
../ThirdParty/SEGGER/Segger/SEGGER_RTT_printf.c \
../ThirdParty/SEGGER/Segger/SEGGER_SYSVIEW.c 

S_UPPER_SRCS += \
../ThirdParty/SEGGER/Segger/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./ThirdParty/SEGGER/Segger/SEGGER_RTT.o \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_ASM_ARMv7M.o \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_GCC.o \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_IAR.o \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_KEIL.o \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_SES.o \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_printf.o \
./ThirdParty/SEGGER/Segger/SEGGER_SYSVIEW.o 

S_UPPER_DEPS += \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./ThirdParty/SEGGER/Segger/SEGGER_RTT.d \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_GCC.d \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_IAR.d \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_KEIL.d \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_SES.d \
./ThirdParty/SEGGER/Segger/SEGGER_RTT_printf.d \
./ThirdParty/SEGGER/Segger/SEGGER_SYSVIEW.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/Segger/%.o ThirdParty/SEGGER/Segger/%.su ThirdParty/SEGGER/Segger/%.cyclo: ../ThirdParty/SEGGER/Segger/%.c ThirdParty/SEGGER/Segger/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I.. -I../Core/Inc -I../Drivers -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../state_machines -I../tasks -I../ThirdParty/FreeRTOS/Source/include -I../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/SEGGER/Segger/%.o: ../ThirdParty/SEGGER/Segger/%.S ThirdParty/SEGGER/Segger/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-ThirdParty-2f-SEGGER-2f-Segger

clean-ThirdParty-2f-SEGGER-2f-Segger:
	-$(RM) ./ThirdParty/SEGGER/Segger/SEGGER_RTT.cyclo ./ThirdParty/SEGGER/Segger/SEGGER_RTT.d ./ThirdParty/SEGGER/Segger/SEGGER_RTT.o ./ThirdParty/SEGGER/Segger/SEGGER_RTT.su ./ThirdParty/SEGGER/Segger/SEGGER_RTT_ASM_ARMv7M.d ./ThirdParty/SEGGER/Segger/SEGGER_RTT_ASM_ARMv7M.o ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_GCC.cyclo ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_GCC.d ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_GCC.o ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_GCC.su ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_IAR.cyclo ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_IAR.d ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_IAR.o ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_IAR.su ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_KEIL.cyclo ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_KEIL.d ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_KEIL.o ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_KEIL.su ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_SES.cyclo ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_SES.d ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_SES.o ./ThirdParty/SEGGER/Segger/SEGGER_RTT_Syscalls_SES.su ./ThirdParty/SEGGER/Segger/SEGGER_RTT_printf.cyclo ./ThirdParty/SEGGER/Segger/SEGGER_RTT_printf.d ./ThirdParty/SEGGER/Segger/SEGGER_RTT_printf.o ./ThirdParty/SEGGER/Segger/SEGGER_RTT_printf.su ./ThirdParty/SEGGER/Segger/SEGGER_SYSVIEW.cyclo ./ThirdParty/SEGGER/Segger/SEGGER_SYSVIEW.d ./ThirdParty/SEGGER/Segger/SEGGER_SYSVIEW.o ./ThirdParty/SEGGER/Segger/SEGGER_SYSVIEW.su

.PHONY: clean-ThirdParty-2f-SEGGER-2f-Segger

