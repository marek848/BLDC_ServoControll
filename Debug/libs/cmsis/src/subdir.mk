################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libs/cmsis/src/core_cm3.c \
../libs/cmsis/src/startup_cm.c \
../libs/cmsis/src/startup_stm32f10x.c \
../libs/cmsis/src/system_stm32f10x.c \
../libs/cmsis/src/vectors_stm32f10x.c 

OBJS += \
./libs/cmsis/src/core_cm3.o \
./libs/cmsis/src/startup_cm.o \
./libs/cmsis/src/startup_stm32f10x.o \
./libs/cmsis/src/system_stm32f10x.o \
./libs/cmsis/src/vectors_stm32f10x.o 

C_DEPS += \
./libs/cmsis/src/core_cm3.d \
./libs/cmsis/src/startup_cm.d \
./libs/cmsis/src/startup_stm32f10x.d \
./libs/cmsis/src/system_stm32f10x.d \
./libs/cmsis/src/vectors_stm32f10x.d 


# Each subdirectory must supply rules for building sources it contributes
libs/cmsis/src/%.o: ../libs/cmsis/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DDEBUG -I"../include" -I"../include/MyFun" -I"../libs/cmsis/include" -I"../libs/StdPeriph/include" -I"../libs/inc" -I"../libs/src" -I"../libs/misc/include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


