################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/Src/stm32f407xx_gpio_driver.c \
../driver/Src/stm32f407xx_spi_driver.c 

OBJS += \
./driver/Src/stm32f407xx_gpio_driver.o \
./driver/Src/stm32f407xx_spi_driver.o 

C_DEPS += \
./driver/Src/stm32f407xx_gpio_driver.d \
./driver/Src/stm32f407xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/Src/%.o: ../driver/Src/%.c driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"E:/Anand/Embedded_Driver_Dev/MyWorkspace/stm32f4xx_driver/driver/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

