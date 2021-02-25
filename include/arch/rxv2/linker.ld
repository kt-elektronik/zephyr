#include <soc.h>
#include <devicetree.h>

#include <autoconf.h>
#include <linker/sections.h>

#include <linker/linker-defs.h>
#include <linker/linker-tool.h>


#define ROMABLE_REGION ROM
#define RAMABLE_REGION RAM

#define ROM_START 0xFFE00000   /* Start at 0xFFE00000 in normal mode (non dualbank) */
#define ROM_END   0xFFFFFFFF+1 /* '+1' because linker can't handle 64 bit integers  */
#define ROM_SIZE  0x100000
#define RAM_START 0x0
#define RAM_SIZE  0xA0000

_region_min_align = 4;

MEMORY
{
	ROM (rx) : ORIGIN = ROM_START, LENGTH = (ROM_END - ROM_START) 
	RAM (rwx): ORIGIN = RAM_START, LENGTH = RAM_SIZE
	OFS      : ORIGIN = 0xFE7F5D00, LENGTH = 256
}

ENTRY(CONFIG_KERNEL_ENTRY)

SECTIONS
{	
	/* The rxgcc compiler adds per default a leading underscore (-fleading-underscore) to symbols that are 
	 * created from assembly sources. But the zephyr symbols are without that leading underscore. 
	 * This workaround allows the linker to find the zephyr symbols.   Compiling with -fnoleading-underscore 
	 * is no option because it causes that the linker will not find the symbols from the gcclib.
	 */
	PROVIDE(_PrepC = __PrepC);
	PROVIDE(___init_PRE_KERNEL_1_start = __init_PRE_KERNEL_1_start);
	PROVIDE(___init_PRE_KERNEL_2_start = __init_PRE_KERNEL_2_start);
	PROVIDE(___init_POST_KERNEL_start = __init_POST_KERNEL_start);
	PROVIDE(___init_APPLICATION_start = __init_APPLICATION_start);
	PROVIDE(___device_start = __device_start);
	PROVIDE(___device_end = __device_end);
	PROVIDE(___device_init_status_start = __device_init_status_start);
	PROVIDE(__k_heap_list_start = _k_heap_list_start);
	PROVIDE(__k_heap_list_end = _k_heap_list_end);
	PROVIDE(___static_thread_data_list_start = __static_thread_data_list_start);
	PROVIDE(___static_thread_data_list_end = __static_thread_data_list_end);
	PROVIDE(___init_end = __init_end);
	/* PROVIDE(___ExceptVectors = _ExceptVectors); */

#include <linker/rel-sections.ld>

	GROUP_START(ROMABLE_REGION)
    _image_rom_start = ROM_START;   /* for kernel logging */

 	.exvectors 0xFFFFFF80: AT(0xFFFFFF80)
	{	
		KEEP(*(.exvectors))
	} GROUP_LINK_IN(ROMABLE_REGION)

	 .fvectors 0xFFFFFFFC: AT(0xFFFFFFFC)
	{
		KEEP(*(.fvectors))
	} GROUP_LINK_IN(ROMABLE_REGION)

	/* following block makes .text starting at 0xFFF00000 like 
	   .text 0xFFF00000: AT(0xFFF00000) does */
	SECTION_PROLOGUE(rom_start,,)
    {
		. = ROM_START;
	} GROUP_LINK_IN(ROMABLE_REGION)

    SECTION_PROLOGUE(_TEXT_SECTION_NAME,,) /* .text 0xFFF00000: AT(0xFFF00000) */
	{
		_image_text_start = .;
		*(.text)
		*(.text.*)
		*(P)
		etext = .;
	} GROUP_LINK_IN(ROMABLE_REGION)

	_image_text_end = .;
    
	#include <linker/common-rom.ld>

	SECTION_PROLOGUE(.rvectors,,)
	{
		_rvectors_start = .;
		KEEP(*(.rvectors))
		_rvectors_end = .;
	} GROUP_LINK_IN(ROMABLE_REGION)
	SECTION_PROLOGUE(init,,)
	{
		KEEP(*(.init))
		__preinit_array_start = .;
		KEEP(*(.preinit_array))
		__preinit_array_end = .;
		__init_array_start = (. + 3) & ~ 3;
		KEEP(*(.init_array))
		KEEP(*(SORT(.init_array.*)))
		__init_array_end = .;
		__fini_array_start = .;
		KEEP(*(.fini_array))
		KEEP(*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} GROUP_LINK_IN(ROMABLE_REGION)
	SECTION_PROLOGUE(fini,,)
	{
		KEEP(*(.fini))
	} GROUP_LINK_IN(ROMABLE_REGION)
	SECTION_PROLOGUE(got,,)
	{
		*(.got)
		*(.got.plt)
	} GROUP_LINK_IN(ROMABLE_REGION)
	SECTION_PROLOGUE(_RODATA_SECTION_NAME,,)
	{
		*(.rodata)
		*(.rodata.*)
		*(C_1)
		*(C_2)
		*(C)
		_erodata = .;
	} GROUP_LINK_IN(ROMABLE_REGION)
	SECTION_PROLOGUE(eh_frame_hdr,,)
	{
		*(.eh_frame_hdr)
	} GROUP_LINK_IN(ROMABLE_REGION)
	SECTION_PROLOGUE(eh_frame,,)
	{
		*(.eh_frame)
	} GROUP_LINK_IN(ROMABLE_REGION)
	SECTION_PROLOGUE(jcr,,)
	{
		*(.jcr)
	} GROUP_LINK_IN(ROMABLE_REGION)
	SECTION_PROLOGUE(tors,,)
	{
		__CTOR_LIST__ = .;
		. = ALIGN(2);
		___ctors = .;
		*(.ctors)
		___ctors_end = .;
		__CTOR_END__ = .;
		__DTOR_LIST__ = .;
		___dtors = .;
		*(.dtors)
		___dtors_end = .;
		__DTOR_END__ = .;
		. = ALIGN(2);
		_mdata = .;
	} GROUP_LINK_IN(ROMABLE_REGION)
 	_image_rom_end = .;
    GROUP_END(ROMABLE_REGION)

    GROUP_START(RAMABLE_REGION)

#include <linker/common-ram.ld>

	.ustack 0x1E00: AT(0x1E00)
	{
		_ustack = .;
	} GROUP_LINK_IN(RAMABLE_REGION)

	.istack 0xF00: AT(0xF00)
	{
		_istack = .;
	} GROUP_LINK_IN(RAMABLE_REGION)

	.data 0x1E04: AT(_mdata)
	{
		_data = .;
		*(.data)
		*(.data.*)
		*(D)
		*(D_1)
		*(D_2)
		_edata = .;
	} GROUP_LINK_IN(RAMABLE_REGION)
	SECTION_PROLOGUE(gcc_exc,,)
	{
		*(.gcc_exc)
	} GROUP_LINK_IN(RAMABLE_REGION)


	SECTION_PROLOGUE(_BSS_SECTION_NAME,,)
	{
		_bss = .;
		*(.bss)
		*(.bss.**)
		*(COMMON)
		*(B)
		*(B_1)
		*(B_2)
		_ebss = .;
		_end = .;
	} GROUP_LINK_IN(RAMABLE_REGION)


#include <linker/common-noinit.ld>

	GROUP_END(RAMABLE_REGION)


	.ofs1 0xFE7F5D40: AT(0xFE7F5D00)
	{
		KEEP(*(.ofs1))
	} > OFS
	.ofs2 0xFE7F5D48: AT(0xFE7F5D10)
	{
		KEEP(*(.ofs2))
	} > OFS
	.ofs3 0xFE7F5D50: AT(0xFE7F5D40)
	{
		KEEP(*(.ofs3))
	} > OFS
	.ofs4 0xFE7F5D10: AT(0xFE7F5D48)
	{
		KEEP(*(.ofs4))
	} > OFS
	.ofs5 0xFE7F5D10: AT(0xFE7F5D50)
	{
		KEEP(*(.ofs5))
	} > OFS
	.ofs6 0xFE7F5D10: AT(0xFE7F5D64)
	{
		KEEP(*(.ofs6))
	} > OFS
	.ofs7 0xFE7F5D10: AT(0xFE7F5D70)
	{
		KEEP(*(.ofs7))
	} > OFS


/* Located in generated directory. This file is populated by the
* zephyr_linker_sources() Cmake function.
*/
#include <snippets-sections.ld>

#include <linker/debug-sections.ld>

}
