// Boot assembly for Raspberry Pi Zero 2 W (ARM Cortex-A53)
// Using ARMv7 assembly syntax for LLVM compatibility

.section ".text._start"

.global _start

_start:
    // Disable interrupts
    cpsid if

    // Check which core we're running on (simplified for ARMv7)
    mrc p15, 0, r0, c0, c0, 5   // Read Multiprocessor Affinity Register
    and r0, r0, #3
    cmp r0, #0
    bne halt_other_cores        // If not core 0, halt

    // Core 0 initialization - set up stack
    ldr sp, =_stack_end

    // Clear frame pointer
    mov fp, #0

    // Jump to Rust kernel
    bl kernel_main

halt_other_cores:
    // Put other cores to sleep
    wfe
    b halt_other_cores

halt_forever:
    // Infinite loop for errors
    b halt_forever
