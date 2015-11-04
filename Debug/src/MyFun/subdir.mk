################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/MyFun/handlers.c \
../src/MyFun/init_fun.c \
../src/MyFun/main_fun.c 

OBJS += \
./src/MyFun/handlers.o \
./src/MyFun/init_fun.o \
./src/MyFun/main_fun.o 

C_DEPS += \
./src/MyFun/handlers.d \
./src/MyFun/init_fun.d \
./src/MyFun/main_fun.d 


# Each subdirectory must supply rules for building sources it contributes
src/MyFun/%.o: ../src/MyFun/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall  -g3 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DDEBUG -I"../include" -I"../include/MyFun" -I"../libs/cmsis/include" -I"../libs/StdPeriph/include" -I"../libs/inc" -I"../libs/src" -I"../libs/misc/include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


