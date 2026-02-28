SHELL     := cmd.exe
MAKEFLAGS += --silent
program: all flash clean

# =============================================================================
# Toolchain
# =============================================================================

TOOLCHAIN_DIR := ..\toolchains\xpack-arm-none-eabi-gcc-14.2.1-1.1\bin
PROG_DIR      := ..\toolchains\Cube-Programmer\bin

CC := $(TOOLCHAIN_DIR)\arm-none-eabi-gcc.exe
AS := $(TOOLCHAIN_DIR)\arm-none-eabi-gcc.exe
LD := $(TOOLCHAIN_DIR)\arm-none-eabi-gcc.exe

# =============================================================================
# Paths & Files
# =============================================================================

BUILD_DIR := build
ELF       := $(BUILD_DIR)\firmware.elf

SRC_C     := main.c
SRC_S     := startup.s
LINKER    := linker.ld

OBJ_C     := $(BUILD_DIR)\main.o
OBJ_S     := $(BUILD_DIR)\startup.o

# =============================================================================
# MCU Flags (Cortex-M33)
# =============================================================================

CPU_FLAGS := -mcpu=cortex-m33 -mthumb
LTO       := -flto

# =============================================================================
# Size Optimization Flags
# =============================================================================

OPT_FLAGS := -Os

COMMON := $(CPU_FLAGS) \
          $(OPT_FLAGS) \
	  $(LTO) \
          -ffunction-sections \
          -fdata-sections \
          -fno-unwind-tables \
          -fno-asynchronous-unwind-tables \
          -fno-exceptions \
          -Wall

CFLAGS  := $(COMMON) -std=c11
ASFLAGS := $(COMMON) -x assembler-with-cpp

LDFLAGS := $(CPU_FLAGS) \
           $(LTO)\
           -T$(LINKER) \
           -Wl,--gc-sections \
           -nostartfiles \
           -specs=nano.specs \
           -specs=nosys.specs

# =============================================================================
# Build
# =============================================================================

all: $(ELF)

# -----------------------------------------------------------------------------
# Build rules
# -----------------------------------------------------------------------------

$(BUILD_DIR):
	@if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)

$(OBJ_C): $(SRC_C) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_S): $(SRC_S) | $(BUILD_DIR)
	$(AS) $(ASFLAGS) -c $< -o $@

$(ELF): $(OBJ_C) $(OBJ_S)
	$(LD) $(LDFLAGS) $^ -o $@

# -----------------------------------------------------------------------------
# Flash
# -----------------------------------------------------------------------------

flash: $(ELF)
	$(PROG_DIR)\STM32_Programmer_CLI.exe -c port=SWD -w $(ELF) -v -rst > NUL

# -----------------------------------------------------------------------------
# Clean
# -----------------------------------------------------------------------------

clean:
	@if exist $(BUILD_DIR) rmdir /S /Q $(BUILD_DIR)
