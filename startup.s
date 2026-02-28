.syntax unified
.cpu cortex-m33
.thumb

.global __StackTop
.global Reset_Handler

//Symbols from linker
.extern _sidata
.extern _sdata
.extern _edata
.extern _sbss
.extern _ebss
.extern main

//Vector table
.section .vector_table, "a", %progbits
.align 2
__isr_vector:
    .word __StackTop
    .word Reset_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word 0
    .word 0
    .word 0
    .word Default_Handler
    .word Default_Handler
    .word 0
    .word Default_Handler
    .word Default_Handler
    .rept 43
        .word Default_Handler
    .endr
    .word I2C1_EV_IRQHandler
    .word I2C1_ER_IRQHandler
    .word Default_Handler
    .word USART1_IRQHandler
    .rept 34
    .word Default_Handler
    .endr

// Reset handler
.section .text.Reset_Handler, "ax", %progbits
Reset_Handler:

// Copy data Flash to RAM
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
1:
    cmp r1, r2
    bcs 3f
2:
    ldr r3, [r0], #4
    str r3, [r1], #4
    b 1b

// Zero BSS
3:
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
4:
    cmp r0, r1
    bcc 5f
    b 6f
5:
    str r2, [r0], #4
    b 4b

// Jump to main()
6:
    bl main

//Infinite loop if main returns
7:
    b 7b

// Default handler
.section .text.Default_Handler, "ax", %progbits
Default_Handler:
    b .

// Weak interrupt handlers
.weak I2C1_EV_IRQHandler
.weak I2C1_ER_IRQHandler
.weak USART1_IRQHandler

.thumb_func
I2C1_EV_IRQHandler:
    b Default_Handler

.thumb_func
I2C1_ER_IRQHandler:
    b Default_Handler

.thumb_func
USART1_IRQHandler:
    b Default_Handler

