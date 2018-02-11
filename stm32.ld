MEMORY
{
  ram_main (rwx)        : ORIGIN = 0x20020000, LENGTH = 368K
  ram_backup (rwx)      : ORIGIN = 0x2007C000, LENGTH = 16K
  ram_dtcm (rwx)        : ORIGIN = 0x20000000, LENGTH = 128K
  stack(rwx)        : ORIGIN = 0x2007BFF8, LENGTH = 0K

  rom (rx)         : ORIGIN = 0x00200000, LENGTH = 1024K
}

SECTIONS
{
    .nvic_vector : 
    {
        *(vectors)    /* Vector table */
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
