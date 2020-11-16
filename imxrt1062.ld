MEMORY
{
    ITCM (rwx):  ORIGIN = 0x00000000, LENGTH = 512K
    DTCM (rwx):  ORIGIN = 0x20000000, LENGTH = 512K
    RAM (rwx):   ORIGIN = 0x20200000, LENGTH = 512K
    FLASH (rwx): ORIGIN = 0x60000000, LENGTH = 1984K
}

ENTRY(ImageVectorTable)

SECTIONS
{
    .text.progmem : {
        KEEP(*(.flashconfig))
        FILL(0xFF)
        . = ORIGIN(FLASH) + 0x1000;
        KEEP(*(.ivt))
        KEEP(*(.bootdata))
        KEEP(*(.startup))
        *(.text.startup*)
        *(.flashmem*)
        *(.progmem*)
        *libc.a:*(.text .text.*)
        *libc_nano.a:*(.text .text.*)
        *libm.a:*(.text .text.*)
        *libstdc++.a:*(.text .text.*)
        *libstdc++_nano.a:*(.text .text.*)
        *libsupc++.a:*(.text .text.*)
        *libsupc++_nano.a:*(.text .text.*)
        *(.text._ZSt*)
        *(.text._ZNSt*)
        *(.text._ZNKSt*)
        *(.text._ZZNSt*)
        *(.text.*ArduinoJson*)
        *(.rodata._ZNSt8__detail12__prime_list*)
        *(.rodata._ZZNSt8__detail18__to_chars_10_impl*)
        . = ALIGN(4);
        KEEP(*(.init))
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;
        __init_array_start = .;
        KEEP (*(.init_array))
        __init_array_end = .;
        . = ALIGN(16);
    } > FLASH

    .ARM.exidx : {
        *(.ARM.exidx.startup* .gnu.linkonce.armexidx.*)
        *(.ARM.exidx.flashmem* .gnu.linkonce.armexidx.*)
        *(.ARM.exidx.text.startup* .gnu.linkonce.armexidx.*)
        *(.ARM.exidx.text._ZSt* .gnu.linkonce.armexidx.*)
        *(.ARM.exidx.text._ZNSt* .gnu.linkonce.armexidx.*)
        *(.ARM.exidx.text._ZNKSt* .gnu.linkonce.armexidx.*)
        *(.ARM.exidx.text._ZZNSt* .gnu.linkonce.armexidx.*)
        *(.ARM.exidx.text.*ArduinoJson* .gnu.linkonce.armexidx.*)
        . = ALIGN(4);
    } > FLASH
 
    .ARM.extab : {
        *(.ARM.extab.flashmem* .gnu.linkonce.armextab.*)
        *(.ARM.extab.text._ZSt* .gnu.linkonce.armextab.*)
        *(.ARM.extab.text._ZNSt* .gnu.linkonce.armextab.*)
        *(.ARM.extab.text._ZNKSt* .gnu.linkonce.armextab.*)
        *(.ARM.extab.text._ZZNSt* .gnu.linkonce.armextab.*)
        *(.ARM.extab.text.*ArduinoJson* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } > FLASH

    .data.func : {
        *(.*__func__*)
        *(.*__PRETTY_FUNCTION__*)
        . = ALIGN(4);
    } > FLASH

    .text.itcm : {
        . = . + 32; /* MPU to trap NULL pointer deref */
        *(.fastrun)
        *(.text*)
        . = ALIGN(16);
    } > ITCM AT> FLASH

    .ARM.exidx.itcm : {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        . = ALIGN(4);
    } > ITCM AT> FLASH

    .ARM.extab.itcm : {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } > ITCM AT> FLASH

    .text.itcm.padding (NOLOAD) : {
        . = ALIGN(32768);
    } > ITCM

    .data : {
        *(.rodata*)
        *(.data*)
    } > DTCM AT> FLASH

    .bss ALIGN(4) : {
        *(.bss*)
        *(COMMON)
        . = ALIGN(32);
        . = . + 32; /* MPU to trap stack overflow */
    } > DTCM

    .bss.dma (NOLOAD) : {
        *(.hab_log)
        *(.dmabuffers)
        . = ALIGN(32);
    } > RAM

    .text.csf : {
        FILL(0xFF)
        . = ALIGN(4);
        KEEP(*(.csf))
    } > FLASH

    _stext = ADDR(.text.itcm);
    _etext = ADDR(.text.itcm) + SIZEOF(.text.itcm) + SIZEOF(.ARM.exidx) + SIZEOF(.ARM.extab);
    _stextload = LOADADDR(.text.itcm);

    _sdata = ADDR(.data);
    _edata = ADDR(.data) + SIZEOF(.data);
    _sdataload = LOADADDR(.data);

    _sbss = ADDR(.bss);
    _ebss = ADDR(.bss) + SIZEOF(.bss);

    _heap_start = ADDR(.bss.dma) + SIZEOF(.bss.dma);
    _heap_end = ORIGIN(RAM) + LENGTH(RAM);

    _itcm_block_count = (SIZEOF(.text.itcm) + SIZEOF(.ARM.exidx) + SIZEOF(.ARM.extab) + 0x7FFF) >> 15;
    _flexram_bank_config = 0xAAAAAAAA | ((1 << (_itcm_block_count * 2)) - 1);
    _estack = ORIGIN(DTCM) + ((16 - _itcm_block_count) << 15) - 128;

    _flashimagelen = SIZEOF(.text.progmem) + SIZEOF(.text.itcm) + SIZEOF(.ARM.exidx) + SIZEOF(.ARM.extab) + SIZEOF(.data) + SIZEOF(.data.func) + SIZEOF(.text.csf);
    _teensy_model_identifier = 0x24;

    __exidx_start = ADDR(.ARM.exidx);
    __exidx_end = ADDR(.ARM.exidx) + SIZEOF(.ARM.exidx);

    .debug_info     0 : { *(.debug_info) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }

}
