################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CANLib/CAN_Main/CAN_Main.c 

OBJS += \
./CANLib/CAN_Main/CAN_Main.o 

C_DEPS += \
./CANLib/CAN_Main/CAN_Main.d 


# Each subdirectory must supply rules for building sources it contributes
CANLib/CAN_Main/%.o CANLib/CAN_Main/%.su CANLib/CAN_Main/%.cyclo: ../CANLib/CAN_Main/%.c CANLib/CAN_Main/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../CANLib/Defines/Inc -I../CANLib/CAN_Main -I../CANLib_RoboMas/CAN_C620 -I../CANLib_RoboMas/CAN_C620_System -I../CANLib_RoboMas/CAN_C620_Def -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CANLib-2f-CAN_Main

clean-CANLib-2f-CAN_Main:
	-$(RM) ./CANLib/CAN_Main/CAN_Main.cyclo ./CANLib/CAN_Main/CAN_Main.d ./CANLib/CAN_Main/CAN_Main.o ./CANLib/CAN_Main/CAN_Main.su

.PHONY: clean-CANLib-2f-CAN_Main

