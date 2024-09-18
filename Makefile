CROSS_COMPILE   := arm-none-eabi-
CC              := $(CROSS_COMPILE)gcc
LD              := $(CROSS_COMPILE)ld
OBJCOPY         := $(CROSS_COMPILE)objcopy
OBJDUMP         := $(CROSS_COMPILE)objdump

IMAGE           := rom.axf
IMG_BIN         := rom.bin
IMG_DIS         := rom.dis
IMG_HEX         := rom.hex
IMG_MAP         := rom.map

OBJS := Device/MM32F5330/HAL_Lib/src/hal_exti.o \
	Device/MM32F5330/HAL_Lib/src/hal_adc.o \
	Device/MM32F5330/HAL_Lib/src/hal_lptim.o \
	Device/MM32F5330/HAL_Lib/src/hal_comp.o \
	Device/MM32F5330/HAL_Lib/src/hal_misc.o \
	Device/MM32F5330/HAL_Lib/src/hal_mds.o \
	Device/MM32F5330/HAL_Lib/src/hal_flash.o \
	Device/MM32F5330/HAL_Lib/src/hal_dac.o \
	Device/MM32F5330/HAL_Lib/src/hal_gpio.o \
	Device/MM32F5330/HAL_Lib/src/hal_lpuart.o \
	Device/MM32F5330/HAL_Lib/src/hal_i2c.o \
	Device/MM32F5330/HAL_Lib/src/hal_tim.o \
	Device/MM32F5330/HAL_Lib/src/hal_crs.o \
	Device/MM32F5330/HAL_Lib/src/hal_syscfg.o \
	Device/MM32F5330/HAL_Lib/src/hal_pwr.o \
	Device/MM32F5330/HAL_Lib/src/hal_bkp.o \
	Device/MM32F5330/HAL_Lib/src/hal_uart.o \
	Device/MM32F5330/HAL_Lib/src/hal_spi.o \
	Device/MM32F5330/HAL_Lib/src/hal_dma.o \
	Device/MM32F5330/HAL_Lib/src/hal_iwdg.o \
	Device/MM32F5330/HAL_Lib/src/hal_usart.o \
	Device/MM32F5330/HAL_Lib/src/hal_usbfs.o \
	Device/MM32F5330/HAL_Lib/src/hal_dbg.o \
	Device/MM32F5330/HAL_Lib/src/hal_flexcan.o \
	Device/MM32F5330/HAL_Lib/src/hal_uid.o \
	Device/MM32F5330/HAL_Lib/src/hal_wwdg.o \
	Device/MM32F5330/HAL_Lib/src/hal_cordic.o \
	Device/MM32F5330/HAL_Lib/src/hal_rtc.o \
	Device/MM32F5330/HAL_Lib/src/hal_crcpoly.o \
	Device/MM32F5330/HAL_Lib/src/hal_sram.o \
	Device/MM32F5330/HAL_Lib/src/hal_i3c.o \
	Device/MM32F5330/HAL_Lib/src/hal_rcc.o \
	Device/MM32F5330/Source/system_mm32f5330.o \
	Device/MM32F5330/Source/KEIL_StartAsm/startup_mm32f5330_keil.o

OBJS += Soruce/mm32f5330_it.o \
	Soruce/paj7620u2.o \
	Soruce/exti_interrupt.o \
	Soruce/platform.o \
	Soruce/uart_interrupt.o \
	Soruce/main.o \
	Soruce/i2c_master_polling.o \
	Soruce/i2c_master_interrupt.o \
	Soruce/gpio_key_input.o

INC_FILE := -I Device/CMSIS/Core/Include -I Device/MM32F5330/HAL_Lib/inc -I Device/MM32F5330/Include -I Soruce

DEPS := $(patsubst %.o,%.d,$(OBJS))

CPPFLAGS := -O1 -g -Wall -MMD -MP -fno-builtin -fno-common -Werror
CPPFLAGS += $(INC_FILE)

LDFLAGS += -g --entry=_start -Map $(IMG_MAP) -T Device/MM32F5330/Source/mm32f5330.sct

all: $(IMAGE)
$(IMAGE): $(OBJS)
	$(LD) $(LDFLAGS) $(OBJS) -o $@ -L $(shell dirname `$(CC) -print-libgcc-file-name`) -lgcc
	$(OBJCOPY) -O binary $(IMAGE) $(IMG_BIN)
	$(OBJCOPY) -O ihex $(IMAGE) $(IMG_HEX)
	$(OBJDUMP) -S $(IMAGE) > $(IMG_DIS)

clean:
	rm -rf $(IMAGE) $(OBJS) $(DEPS) $(LD_SCRIPT) $(IMG_BIN) $(IMG_DIS) $(IMG_HEX) $(IMG_MAP)

.PHONY: all clean

-include $(DEPS)