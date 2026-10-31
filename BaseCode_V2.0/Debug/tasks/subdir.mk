################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tasks/acquisition_task.c \
../tasks/communication_task.c \
../tasks/estimation_task.c \
../tasks/motor_task.c \
../tasks/task_manager.c 

OBJS += \
./tasks/acquisition_task.o \
./tasks/communication_task.o \
./tasks/estimation_task.o \
./tasks/motor_task.o \
./tasks/task_manager.o 

C_DEPS += \
./tasks/acquisition_task.d \
./tasks/communication_task.d \
./tasks/estimation_task.d \
./tasks/motor_task.d \
./tasks/task_manager.d 


# Each subdirectory must supply rules for building sources it contributes
tasks/%.o tasks/%.su tasks/%.cyclo: ../tasks/%.c tasks/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I.. -I../Core/Inc -I../Drivers -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../state_machines -I../tasks -I../ThirdParty/FreeRTOS/Source/include -I../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tasks

clean-tasks:
	-$(RM) ./tasks/acquisition_task.cyclo ./tasks/acquisition_task.d ./tasks/acquisition_task.o ./tasks/acquisition_task.su ./tasks/communication_task.cyclo ./tasks/communication_task.d ./tasks/communication_task.o ./tasks/communication_task.su ./tasks/estimation_task.cyclo ./tasks/estimation_task.d ./tasks/estimation_task.o ./tasks/estimation_task.su ./tasks/motor_task.cyclo ./tasks/motor_task.d ./tasks/motor_task.o ./tasks/motor_task.su ./tasks/task_manager.cyclo ./tasks/task_manager.d ./tasks/task_manager.o ./tasks/task_manager.su

.PHONY: clean-tasks

