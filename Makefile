
######################################
# target
######################################
TARGET = basic_framework
SHELL = cmd.exe


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0 # O0避免没有使用到的变量被优化,如果没有特殊的调试需求请修改成-Og.
          # 为了更高的性能,正式上车不需要调试时修改成-O3/-Ofast


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
HAL_N_Middlewares/Src/main.c \
HAL_N_Middlewares/Src/gpio.c \
HAL_N_Middlewares/Src/freertos.c \
HAL_N_Middlewares/Src/adc.c \
HAL_N_Middlewares/Src/can.c \
HAL_N_Middlewares/Src/crc.c \
HAL_N_Middlewares/Src/dac.c \
HAL_N_Middlewares/Src/dma.c \
HAL_N_Middlewares/Src/i2c.c \
HAL_N_Middlewares/Src/rng.c \
HAL_N_Middlewares/Src/rtc.c \
HAL_N_Middlewares/Src/spi.c \
HAL_N_Middlewares/Src/tim.c \
HAL_N_Middlewares/Src/usart.c \
HAL_N_Middlewares/Src/usb_device.c \
HAL_N_Middlewares/Src/usbd_conf.c \
HAL_N_Middlewares/Src/usbd_desc.c \
HAL_N_Middlewares/Src/usbd_cdc_if.c \
HAL_N_Middlewares/Src/stm32f4xx_it.c \
HAL_N_Middlewares/Src/stm32f4xx_hal_msp.c \
HAL_N_Middlewares/Src/stm32f4xx_hal_timebase_tim.c \
HAL_N_Middlewares/Src/system_stm32f4xx.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
HAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
HAL_N_Middlewares/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
HAL_N_Middlewares/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
HAL_N_Middlewares/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
HAL_N_Middlewares/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
HAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
HAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
HAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/list.c \
HAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
HAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
HAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
HAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/timers.c \
HAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
HAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
HAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
HAL_N_Middlewares/Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c \
HAL_N_Middlewares/Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c \
bsp/dwt/bsp_dwt.c \
bsp/pwm/bsp_pwm.c \
bsp/bsp_legacy_support/bsp_temperature.c \
bsp/bsp_legacy_support/bsp_buzzer.c \
bsp/bsp_legacy_support/bsp_led.c \
bsp/gpio/bsp_gpio.c \
bsp/spi/bsp_spi.c \
bsp/iic/bsp_iic.c \
bsp/can/bsp_can.c \
bsp/usart/bsp_usart.c \
bsp/usb/bsp_usb.c \
bsp/log/bsp_log.c \
bsp/bsp_init.c \
modules/algorithm/controller.c \
modules/algorithm/kalman_filter.c \
modules/algorithm/QuaternionEKF.c \
modules/algorithm/crc8.c \
modules/algorithm/crc16.c \
modules/algorithm/user_lib.c \
modules/BMI088/bmi088.c \
modules/imu/BMI088driver.c \
modules/imu/BMI088Middleware.c \
modules/imu/ins_task.c \
modules/ist8310/ist8310.c \
modules/led_task/led_task.c \
modules/led/led.c \
modules/master_machine/master_process.c \
modules/master_machine/seasky_protocol.c \
modules/motor/DJImotor/dji_motor.c \
modules/motor/HTmotor/HT04.c \
modules/motor/LKmotor/LK9025.c \
modules/motor/step_motor/step_motor.c \
modules/motor/servo_motor/servo_motor.c \
modules/motor/motor_task.c \
modules/oled/oled.c \
modules/referee/crc_ref.c \
modules/referee/referee.c \
modules/referee/referee_UI.c \
modules/referee/referee_communication.c \
modules/remote/remote_control.c \
modules/super_cap/super_cap.c \
modules/can_comm/can_comm.c \
modules/message_center/message_center.c \
modules/daemon/daemon.c \
modules/vofa/vofa.c \
modules/ps_handle/ps_handle.c \
application/gimbal/gimbal.c \
application/chassis/chassis.c \
application/shoot/shoot.c \
application/cmd/robot_cmd.c \
application/balance_chassis/balance.c \
application/robot.c 


# ASM sources
ASM_SOURCES +=  \
startup_stm32f407xx.s \
HAL_N_Middlewares/Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# PREFIX = ccache arm-none-eabi- 可以通过安装ccache来缓存makefile内容加速编译
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx \
-DARM_MATH_CM4 \
-DARM_MATH_MATRIX_CHECK \
-DARM_MATH_ROUNDING

# AS includes
AS_INCLUDES = -IHAL_N_Middlewares/Inc

# C includes
C_INCLUDES =  \
-IHAL_N_Middlewares/Inc \
-IHAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Inc \
-IHAL_N_Middlewares/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IHAL_N_Middlewares/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
-IHAL_N_Middlewares/Drivers/CMSIS/Include \
-IHAL_N_Middlewares/Drivers/CMSIS/DSP/Include \
-IHAL_N_Middlewares/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IHAL_N_Middlewares/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IHAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-IHAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IHAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/include  \
-IHAL_N_Middlewares/Middlewares/Third_Party/FreeRTOS/Source/include \
-IHAL_N_Middlewares/Middlewares/Third_Party/SEGGER/RTT \
-IHAL_N_Middlewares/Middlewares/Third_Party/SEGGER/Config \
-IHAL_N_Middlewares/Middlewares/ST/ARM/DSP/Inc \
-Iapplication/chassis \
-Iapplication/shoot \
-Iapplication/gimbal \
-Iapplication/cmd \
-Iapplication/balance_chassis \
-Iapplication \
-Ibsp/dwt \
-Ibsp/can \
-Ibsp/usart \
-Ibsp/usb \
-Ibsp/gpio \
-Ibsp/spi \
-Ibsp/iic \
-Ibsp/log \
-Ibsp/pwm \
-Ibsp/bsp_legacy_support \
-Ibsp \
-Imodules/algorithm \
-Imodules/BMI088 \
-Imodules/imu \
-Imodules/ist8310 \
-Imodules/led \
-Imodules/led_task \
-Imodules/master_machine \
-Imodules/motor/DJImotor \
-Imodules/motor/LKmotor \
-Imodules/motor/HTmotor \
-Imodules/motor/step_motor \
-Imodules/motor/servo_motor \
-Imodules/motor \
-Imodules/oled \
-Imodules/referee \
-Imodules/remote \
-Imodules/super_cap \
-Imodules/can_comm \
-Imodules/message_center \
-Imodules/daemon \
-Imodules/vofa \
-Imodules/ps_handle \
-Imodules

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407IGHx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys  \
-larm_cortexM4lf_math
LIBDIR =  \
-LHAL_N_Middlewares/Drivers/CMSIS/Lib/GCC
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# 以下是编译命令,命令之前被粉色高亮的@就是静默输出的指令.删除前面的@会将输出显示到命令行.
# 如@$(CC) -c $(CFLAGS) ...... 去掉第一个@即可.
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	@mkdir $@

#######################################
# clean up
#######################################
clean:
	rd /s/q $(BUILD_DIR)
# linux: rm -rf $(BUILD_DIR)

#######################################
# download directl without debugging
#######################################
OPENOCD_FLASH_START = 0x08000000 # 如果切换芯片可能需要修改此值

download_dap:
	openocd -f openocd_dap.cfg -c init -c halt -c "flash write_image erase $(BUILD_DIR)/$(TARGET).bin $(OPENOCD_FLASH_START)" -c reset -c shutdown

download_jlink:
	JFlash -openprj'stm32.jflash' -open'$(BUILD_DIR)/$(TARGET).hex',0x8000000 -auto -startapp -exit
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
