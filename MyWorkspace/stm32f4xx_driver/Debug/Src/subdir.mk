################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/007_spi_cmd_request_response_IT.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/007_spi_cmd_request_response_IT.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/007_spi_cmd_request_response_IT.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"E:/Anand/Embedded_Driver_Dev/MyWorkspace/stm32f4xx_driver/driver/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

