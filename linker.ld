ENTRY(_start)

SECTIONS
{
    . = 0x8000;

    .text : {
        *(.text._start)
        *(.text*)
    }

    .rodata : {
        *(.rodata*)
    }

    .data : {
        *(.data*)
    }

    __bss_start = .;
    .bss : {
        *(.bss*)
    }
    __bss_end = .;
    __bss_size = __bss_end - __bss_start;

    .ARM.exidx : {
        *(.ARM.exidx*)
    }

    /* Place defmt sections after everything else */
    .defmt.default : { *(.defmt.default*) }
    .defmt.trace : { *(.defmt.trace*) }  
    .defmt.debug : { *(.defmt.debug*) }
    .defmt.info : { *(.defmt.info*) }
    .defmt.warn : { *(.defmt.warn*) }
    .defmt.error : { *(.defmt.error*) }
    .defmt.fmt : { *(.defmt.fmt*) }
    .defmt.end : { *(.defmt.end*) }

    __end = .;

    /* Discard debug sections that cause conflicts */
    /DISCARD/ : {
        *(.debug*)
        *(.ARM.attributes)
    }
}

_stack_end = 0x20000000;
