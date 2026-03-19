/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * @file yt_linker.ld
 * @brief 
 * 
 */











/* MEMORY MAP */
MEMORY
{
    IVT   (RX) : ORIGIN = 0x0, LENGTH = 0x400
    TEXT   (RX) : ORIGIN = 0x400, LENGTH = 0xffc00
    IVT_RAM   (RW) : ORIGIN = 0x1fff0000, LENGTH = 0x400
    STACK   (RW) : ORIGIN = 0x2000fc00, LENGTH = 0x400
    RAM   (RW) : ORIGIN = 0x1fff0400, LENGTH = 0x1f800
}
/* SECTIONS */
SECTIONS
{

    .IVT  : {
        
        . = ALIGN(8);
        IVT_start = .;
        isr_vector_region_start = .;
        KEEP(*(.isr_vector))
        isr_vector_region_end = .;
        
        IVT_end = .;
    } > IVT
   
    



    .TEXT  : {
        
        . = ALIGN(8);
        TEXT_start = .;
        rodata_region_start = .;
        *(.rodata)
        *(.rodata*)
        rodata_region_end = .;
        
        text_region_start = .;
        *(.text)
        *(.text*)
        text_region_end = .;
        
        TEXT_end = .;
    } > TEXT
   
    



    .ARM  : {
        
        . = ALIGN(8);
        ARM_start = .;
        __exidx_start = .;
        ARM.exidx_region_start = .;
        *(.ARM.exidx)
        *(.ARM.exidx*)
        ARM.exidx_region_end = .;
        __exidx_end = .;
        
        ARM_end = .;
    } > TEXT
   
    



    CODE_RAM_rom_start_not_align = .;
    CODE_RAM_rom_start = (((CODE_RAM_rom_start_not_align + 8 - 1) / 8) * 8);
   
    CODE_RAM_rom_end = CODE_RAM_rom_start + CODE_RAM_ram_end - CODE_RAM_ram_start;




    DATA_RAM_rom_start_not_align = CODE_RAM_rom_end;
    DATA_RAM_rom_start = (((DATA_RAM_rom_start_not_align + 8 - 1) / 8) * 8);
   
    DATA_RAM_rom_end = DATA_RAM_rom_start + DATA_RAM_ram_end - DATA_RAM_ram_start;




    .IVT_RAM  : {
        
        . = ALIGN(1024);
        IVT_RAM_start = .;
        . += 0X400;
        IVT_RAM_end = .;
    } > IVT_RAM
   
    



    .STACK  : {
        
        . = ALIGN(4);
        STACK_start = .;
        . += 1024;
        STACK_end = .;
    } > STACK
   
    



    .BSS  (NOLOAD): {
        
        . = ALIGN(4);
        BSS_start = .;
        bss_region_start = .;
        *(.bss)
        *(.bss*)
        bss_region_end = .;
        
        BSS_end = .;
    } > RAM
   
    



    .CODE_RAM  : AT(CODE_RAM_rom_start) {
        
        . = ALIGN(4);
        CODE_RAM_ram_start = .;
        CODE_RAM_start = .;
        code_ram_region_start = .;
        *(.code_ram)
        code_ram_region_end = .;
        
        CODE_RAM_end = .;
        CODE_RAM_ram_end = .;
    } > RAM
   
    
    ASSERT((CODE_RAM_ram_end - CODE_RAM_ram_start) == (CODE_RAM_rom_end - CODE_RAM_rom_start), "Copy Section CODE_RAM Size non-aligned")



    .DATA_RAM  : AT(DATA_RAM_rom_start) {
        
        . = ALIGN(4);
        DATA_RAM_ram_start = .;
        DATA_RAM_start = .;
        data_region_start = .;
        *(.data)
        *(.data*)
        data_region_end = .;
        
        . = ALIGN(4);
        DATA_RAM_end = .;
        DATA_RAM_ram_end = .;
    } > RAM
   
    
    ASSERT((DATA_RAM_ram_end - DATA_RAM_ram_start) == (DATA_RAM_rom_end - DATA_RAM_rom_start), "Copy Section DATA_RAM Size non-aligned")


    IVT_memory_start = ORIGIN(IVT);
    IVT_memory_end = ORIGIN(IVT) + LENGTH(IVT);
    IVT_memory_size = LENGTH(IVT);
    TEXT_memory_start = ORIGIN(TEXT);
    TEXT_memory_end = ORIGIN(TEXT) + LENGTH(TEXT);
    TEXT_memory_size = LENGTH(TEXT);
    IVT_RAM_memory_start = ORIGIN(IVT_RAM);
    IVT_RAM_memory_end = ORIGIN(IVT_RAM) + LENGTH(IVT_RAM);
    IVT_RAM_memory_size = LENGTH(IVT_RAM);
    STACK_memory_start = ORIGIN(STACK);
    STACK_memory_end = ORIGIN(STACK) + LENGTH(STACK);
    STACK_memory_size = LENGTH(STACK);
    RAM_memory_start = ORIGIN(RAM);
    RAM_memory_end = ORIGIN(RAM) + LENGTH(RAM);
    RAM_memory_size = LENGTH(RAM);
}
