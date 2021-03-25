################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MahonyAHRS.c \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

OBJS += \
./Core/Src/MahonyAHRS.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/MahonyAHRS.d \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/MahonyAHRS.o: ../Core/Src/MahonyAHRS.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Core/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Include" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/MahonyAHRS.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/kalman.o: ../Core/Src/kalman.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Core/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Include" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/kalman.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/main.o: ../Core/Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Core/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Include" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/stm32f1xx_hal_msp.o: ../Core/Src/stm32f1xx_hal_msp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Core/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Include" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f1xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/stm32f1xx_it.o: ../Core/Src/stm32f1xx_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Core/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Include" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f1xx_it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/syscalls.o: ../Core/Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Core/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Include" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/sysmem.o: ../Core/Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Core/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Include" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/system_stm32f1xx.o: ../Core/Src/system_stm32f1xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Core/Inc" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Include" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/wandog/STM32CubeIDE/workspace_1.5.1/mpu9250_test_VER1/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/system_stm32f1xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

