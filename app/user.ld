OUTPUT_ARCH("riscv")
OUTPUT_FORMAT("elf32-littleriscv", "elf32-littleriscv", "elf32-littleriscv")


ENTRY(_enter)

MEMORY
{
    ram (RWX) : ORIGIN = 0x02000000, LENGTH = 16K /* Internal 16 Kb SRAM*/
    rom (RX) :ORIGIN = 0x01000000, LENGTH = 8K /* Internal eeprom 8 Kb */
}

SECTIONS
{


    /* ROM SECTION
     *
     * The following sections contain data which lives in read-only memory, if
     * such memory is present in the design, for the entire duration of program
     * execution.
     */

    .text : {
        PROVIDE(__TEXT_START__ = .);
        KEEP(*(.text.init.enter ))
        . = __TEXT_START__ + 0xC0;
        KEEP(*(.text.trap_handler))
        *(.text)
        *(.text.*)
        *(.gnu.linkonce.t.*)
        PROVIDE(__TEXT_END__ = .);
    } >rom

    .fini (READONLY): {
        KEEP (*(SORT_NONE(.fini)))
    } >rom

    .preinit_array (READONLY): ALIGN(4) {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >rom

    .init_array (READONLY): ALIGN(4) {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >rom

    .fini_array (READONLY): ALIGN(4) {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
        KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >rom

    .rodata (READONLY) : {
        *(.rdata)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.r.*)
    } >rom

    /* TEXT SECTION
     *
     * The following section contains the code of the program, excluding
     * everything that's been allocated into the ITIM/LIM already
     */



    /* RAM SECTION
     *
     * The following sections contain data which is copied from read-only
     * memory into a RAM memory
     */


    .data : ALIGN(4) {
        PROVIDE(__data_start = .);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        . = ALIGN(4);
        /*__global_pointer$ - 12bit offset from start of .sdata section, used for Linker Relaxation.
          Should be written to the gp register
        */
        PROVIDE( __global_pointer = . + 0x800 );
        *(.srodata.cst16)
        *(.srodata.cst8)
        *(.srodata.cst4)
        *(.srodata.cst2)
        *(.srodata .srodata.*)
        *(.sdata .sdata.* .sdata2.*)
        *(.gnu.linkonce.s.*)
        PROVIDE(__data_end = .);
    } >ram AT>rom

    PROVIDE( __data_source_start = LOADADDR(.data) );
    PROVIDE( __data_target_start = ADDR(.data) );
    PROVIDE( __data_target_end = ADDR(.data) + SIZEOF(.data) );


    .bss (NOLOAD): ALIGN(4) {
        __bss_start = .;
        *(.sbss*) /* Must be located immediately after the .sdata section */
        *(.gnu.linkonce.sb.*)
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        __bss_end = .;
    } >ram

    __stack_size = DEFINED(__stack_size) ? __stack_size : 1024;
    PROVIDE(__stack_size = __stack_size);

    .stack ORIGIN(ram) + LENGTH(ram) - __stack_size :ALIGN(16)
    {
      . = __stack_size;
      PROVIDE( __sp = . );
    } >ram
}
