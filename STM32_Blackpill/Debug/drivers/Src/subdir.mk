################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f411xx_ADC.c \
../drivers/Src/stm32f411xx_DMA.c \
../drivers/Src/stm32f411xx_GPIO.c \
../drivers/Src/stm32f411xx_Timer.c \
../drivers/Src/stm32f411xx_USART.c 

OBJS += \
./drivers/Src/stm32f411xx_ADC.o \
./drivers/Src/stm32f411xx_DMA.o \
./drivers/Src/stm32f411xx_GPIO.o \
./drivers/Src/stm32f411xx_Timer.o \
./drivers/Src/stm32f411xx_USART.o 

C_DEPS += \
./drivers/Src/stm32f411xx_ADC.d \
./drivers/Src/stm32f411xx_DMA.d \
./drivers/Src/stm32f411xx_GPIO.d \
./drivers/Src/stm32f411xx_Timer.d \
./drivers/Src/stm32f411xx_USART.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411CEUx -c -I../Inc -I"D:/Karthik/Embedded C/my_workspace/target/STM32_Blackpill/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f411xx_ADC.cyclo ./drivers/Src/stm32f411xx_ADC.d ./drivers/Src/stm32f411xx_ADC.o ./drivers/Src/stm32f411xx_ADC.su ./drivers/Src/stm32f411xx_DMA.cyclo ./drivers/Src/stm32f411xx_DMA.d ./drivers/Src/stm32f411xx_DMA.o ./drivers/Src/stm32f411xx_DMA.su ./drivers/Src/stm32f411xx_GPIO.cyclo ./drivers/Src/stm32f411xx_GPIO.d ./drivers/Src/stm32f411xx_GPIO.o ./drivers/Src/stm32f411xx_GPIO.su ./drivers/Src/stm32f411xx_Timer.cyclo ./drivers/Src/stm32f411xx_Timer.d ./drivers/Src/stm32f411xx_Timer.o ./drivers/Src/stm32f411xx_Timer.su ./drivers/Src/stm32f411xx_USART.cyclo ./drivers/Src/stm32f411xx_USART.d ./drivers/Src/stm32f411xx_USART.o ./drivers/Src/stm32f411xx_USART.su

.PHONY: clean-drivers-2f-Src

