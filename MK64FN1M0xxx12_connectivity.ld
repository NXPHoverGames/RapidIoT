ENTRY(Reset_Handler)
_RAM_START_ = (0x20000000);
_RAM_END_ = (0x2002FFFF);
        __BootFlags_Start__ = (((0x14000 - 1 + 1) + (256*4) - 1) + 1);
FREESCALE_PROD_DATA_BASE_ADDR = ((0x000FFFFF) - ( 4 * 1024 ) + 1);
__RAM_VECTOR_TABLE_SIZE = ((256*4));
__BOOT_STACK_ADDRESS = ((((((0x2002FFFF)) - 0x400) - 1) - 0x00 - 0x4) - 1)-0x0F;
MEMORY
{
        Bootloader_region (RW) : ORIGIN = ((0x00000000)), LENGTH = (0x14000 - 1 - ((0x00000000)))
        TEXT_region1 (RX) : ORIGIN = ((0x14000 - 1 + 1)), LENGTH = ((((0x14000 - 1 + 1) + (256*4) - 1) + 1) - ((0x14000 - 1 + 1)))
        BootFlags_region (RX) : ORIGIN = (((0x14000 - 1 + 1) + (256*4) - 1) + 1), LENGTH = (((((0x14000 - 1 + 1) + (256*4) - 1) + 1) + 23) - (((0x14000 - 1 + 1) + (256*4) - 1) + 1))
        TEXT_region2 (RX) : ORIGIN = ((((0x14000 - 1 + 1) + (256*4) - 1) + 1) + 23)+1, LENGTH = ((((0x000FFFFF) - ( 4 * 1024 ) + 1) - 1) - ((((0x14000 - 1 + 1) + (256*4) - 1) + 1) + 23) - 1)
        DATA_region (RW) : ORIGIN = (((0x20000000))), LENGTH = ((0x2002FFFF) - (0x20000000) + 1)
        PRODUCT_INFO_region (RX) : ORIGIN = ((0x000FFFFF) - ( 4 * 1024 ) + 1), LENGTH = (((0x000FFFFF)) - ((0x000FFFFF) - ( 4 * 1024 ) + 1))
}
SECTIONS
{
  .interrupts :
  {
    __VECTOR_TABLE = .;
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > TEXT_region1
  .freescale_prod_data :
  {
    . = ALIGN(4);
    KEEP(*(.FREESCALE_PROD_DATA))
    . = ALIGN(4);
  } > PRODUCT_INFO_region
        .BootloaderFlags :
        {
            . = ALIGN(4);
            KEEP(*(.BootloaderFlags));
            . = ALIGN(4);
        } > BootFlags_region
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
  } > TEXT_region2
  .VERSION_TAGS :
  {
    . = ALIGN(4);
    PROVIDE(__start_VERSION_TAGS = .);
    KEEP(*(.VERSION_TAGS));
    PROVIDE(__stop_VERSION_TAGS = .);
    . = ALIGN(4);
  } > TEXT_region2
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > TEXT_region2
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > TEXT_region2
 .ctors :
  {
    __CTOR_LIST__ = .;
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    __CTOR_END__ = .;
  } > TEXT_region2
  .dtors :
  {
    __DTOR_LIST__ = .;
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __DTOR_END__ = .;
  } > TEXT_region2
  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } > TEXT_region2
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } > TEXT_region2
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } > TEXT_region2
  __etext = .;
  __DATA_ROM = .;
  .mtb :
  {
    . = ALIGN(8);
    _mtb_start = .;
    KEEP(*(.mtb_buf))
    . = ALIGN(8);
    _mtb_end = .;
  } > DATA_region
  .interrupts_ram :
  {
    . = ALIGN(4);
    __VECTOR_RAM__ = .;
    __interrupts_ram_start__ = .;
    *(.m_interrupts_ram)
    . += ((256*4));
    . = ALIGN(4);
    __interrupts_ram_end__ = .;
  } > DATA_region
  __VECTOR_RAM = __VECTOR_RAM__;
  __RAM_VECTOR_TABLE_SIZE_BYTES = (__interrupts_ram_end__ - __interrupts_ram_start__);
  .data : AT(__DATA_ROM)
  {
    . = ALIGN(4);
    __DATA_RAM = .;
    __data_start__ = .;
    *(.data)
    *(.data*)
    KEEP(*(.jcr*))
    . = ALIGN(4);
    __data_end__ = .;
  } > DATA_region
  __DATA_END = __DATA_ROM + (__data_end__ - __data_start__);
  text_end = ORIGIN(TEXT_region2) + LENGTH(TEXT_region2);
  ASSERT(__DATA_END <= text_end, "region m_text overflowed with text and data")
  USB_RAM_GAP = DEFINED(__usb_ram_size__) ? __usb_ram_size__ : 0x400;
  .bss :
  {
    . = ALIGN(4);
    __START_BSS = .;
    __bss_start__ = .;
    *(.bss)
    *(.bss*)
    . = ALIGN(512);
    USB_RAM_START = .;
    . += USB_RAM_GAP;
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    __END_BSS = .;
  } > DATA_region
  .heap :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    _pvHeapStart = .;
    __HeapBase = .;
    . += 0x00;
    __HeapLimit = .;
  } > DATA_region
  .stack :
  {
    . = ALIGN(8);
    . += 0x400;
  } > DATA_region
  m_usb_bdt USB_RAM_START (NOLOAD) :
  {
    *(m_usb_bdt)
    USB_RAM_BDT_END = .;
  }
  m_usb_global USB_RAM_BDT_END (NOLOAD) :
  {
    *(m_usb_global)
  }
  __StackTop = ORIGIN(DATA_region) + LENGTH(DATA_region);
  __StackLimit = __StackTop - 0x400;
  PROVIDE(__stack = __StackTop);
  .ARM.attributes 0 : { *(.ARM.attributes) }
  ASSERT(__StackLimit >= __HeapLimit, "region m_data overflowed with stack and heap")
}
