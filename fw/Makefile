# aioc-slop firmware: plain C, CMSIS headers only
TARGET  := aioc-slop
BUILD   := build
SRCS    := $(wildcard src/*.c)
OBJS    := $(SRCS:src/%.c=$(BUILD)/%.o)

CROSS   := arm-none-eabi-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size

ARCH    := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS  := $(ARCH) -std=c11 -Os -g3 -Wall -Wextra -Wno-unused-parameter \
           -ffunction-sections -fdata-sections -fno-common \
           -DSTM32F302xC -Isrc -Icmsis/core -Icmsis/device $(EXTRA)
LDFLAGS := $(ARCH) -nostartfiles -Taioc.ld -Wl,--gc-sections -Wl,--no-warn-rwx-segments \
           -Wl,-Map=$(BUILD)/$(TARGET).map --specs=nano.specs --specs=nosys.specs

# USB IDs: runtime (this firmware) and the ST ROM bootloader in DFU mode
RUN_ID  := 1209:0001
DFU_ID  := 0483:df11

all: $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex
	@$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/%.o: src/%.c $(wildcard src/*.h) | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) aioc.ld
	$(CC) $(LDFLAGS) $(OBJS) -o $@ -lm

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

# the audio filters are hot loops
$(BUILD)/audio.o: CFLAGS += -O2

$(BUILD):
	mkdir -p $@

# Flash over USB DFU. Works from the ROM bootloader (BOOT0 shorted) and from
# a running aioc-slop (dfu-util detaches it through the DFU runtime interface).
flash: $(BUILD)/$(TARGET).bin
	dfu-util -d $(RUN_ID),$(DFU_ID) -a 0 -s 0x08000000:leave -D $<

clean:
	rm -rf $(BUILD)

.PHONY: all flash clean
