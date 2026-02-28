ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 0x00200000
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00080000
}

__StackTop = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .vector_table :
    {
        KEEP(*(.vector_table))
    } > FLASH

    .text :
    {
        *(.text*)
        *(.rodata*)
    } > FLASH

    .data :
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM AT>FLASH

    _sidata = LOADADDR(.data);

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM
}

