/*
    Confusing naming warning: ictm and itcm in STM32 are two totally different things.
    ictm is a dedicated bus between flash and cpu
    itcm is a piece of ram, directly coupled to cpu, for instructions, for quick execution
*/

MEMORY
{
  ram_main (rwx)        : ORIGIN = 0x20020000, LENGTH = 368K
  ram_backup (rwx)      : ORIGIN = 0x2007C000, LENGTH = 16K
  ram_dtcm (rwx)        : ORIGIN = 0x20000000, LENGTH = 128K
  ram_itcm (rwx)        : ORIGIN = 0x00000000, LENGTH = 16K
  stack(rwx)            : ORIGIN = 0x2007BFF8, LENGTH = 0K

  rom (rx)              : ORIGIN = 0x00200000, LENGTH = 1024K  /* through the ICTM bus - it seems that the DMA devices etc. understand and convert (mask) these addresses automatically */

/*
  rom_sect0_ictm   (rx) : ORIGIN = 0x00200000, LENGTH = 32K
  rom_sect1_ictm   (rx) : ORIGIN = 0x00208000, LENGTH = 32K
  rom_sect2_7_ictm (rx) : ORIGIN = 0x00210000, LENGTH = 960K
*/
}

SECTIONS
{
    .nvic_vector : 
    {
        *(.nvic_vector)    /* Vector table */
    } >rom

    . = ALIGN(8);

    .text : 
    {
        *(.text)      /* Program code */
        *(.text.*)
        *(.rodata)    /* Read only data */
        *(.rodata.*)
    } >rom

    . = ALIGN(8);

    _DATAI_BEGIN = LOADADDR(.data);

    .data : 
    {
        _DATA_BEGIN = .;
        *(.data)
        *(.data.*)        
        _DATA_END = .;
    } >ram_main AT>rom

    . = ALIGN(8);

    _DATA_DTCM_I_BEGIN = LOADADDR(.data_dtcm);

    .data_dtcm : 
    {
        _DATA_DTCM_BEGIN = .;
        *(.data_dtcm)
        *(.data_dtcm.*)        
        _DATA_DTCM_END = .;
    } >ram_dtcm AT>rom

    . = ALIGN(8);

    _TEXT_ITCM_I_BEGIN = LOADADDR(.text_itcm);

    .text_itcm :  /* Code run from 16K ITCM RAM block */ 
    {
        _TEXT_ITCM_BEGIN = .;
        *(.text_itcm)
	_TEXT_ITCM_END = .;
    } >ram_itcm AT>rom

    . = ALIGN(8);


    .bss :
    {
        _BSS_BEGIN = .;
        *(.bss)
        *(COMMON)        
        _BSS_END = .;
    } >ram_main 

    . = ALIGN(8);

    .heap :
    {
        _HEAP = .;
    } >ram_main

    .stack :
    {
        _STACKTOP = .;
    } >stack


    . = ALIGN(8);

}  
