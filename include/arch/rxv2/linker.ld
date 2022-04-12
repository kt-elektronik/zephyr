/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <devicetree.h>

#include <autoconf.h>
#include <linker/sections.h>

#include <linker/linker-defs.h>
#include <linker/linker-tool.h>

#define ROMABLE_REGION ROM
#define RAMABLE_REGION RAM

/* single bank configuration with 2 MB of code flash */
#define ROM_START 0xFFE00000
#define ROM_SIZE  0x200000

#if CONFIG_SRAM_SIZE == 0
#error need SRAM size !
#endif

#define RAM_START (CONFIG_SRAM_BASE_ADDRESS)
#define RAM_SIZE  (KB(CONFIG_SRAM_SIZE))

_region_min_align = 4;

MEMORY
{
	ROM (rx) : ORIGIN = ROM_START, LENGTH = ROM_SIZE
	RAM (rwx): ORIGIN = RAM_START, LENGTH = RAM_SIZE
	OFS (r)  : ORIGIN = 0xFE7F5D00, LENGTH = 256
	/* IDT_LIST is only required for Zephyr pre-build (see zephyr\include\linker\intlist.ld) */
	IDT_LIST (rx) : ORIGIN = 0xFFFFF780, LENGTH = 2K
}

ENTRY(CONFIG_KERNEL_ENTRY)

SECTIONS
{
	/* The rxgcc compiler per default adds a leading underscore
	 * (-fleading-underscore) to symbols that are created from assembly
	 * sources. But the zephyr symbols are without that leading underscore.
	 * This workaround allows the linker to find the zephyr symbols.
	 * Compiling with -fnoleading-underscore is no option because it causes
	 * that the linker will not find the symbols from the gcclib.
	 */
	PROVIDE(z_cstart = _z_cstart);

#include <linker/rel-sections.ld>

	GROUP_START(ROMABLE_REGION)
	__rodata_start = ROM_START;   /* for kernel logging */

	.exvectors 0xFFFFFF80: AT(0xFFFFFF80)
	{
		KEEP(*(.exvectors))
	} GROUP_LINK_IN(ROMABLE_REGION)

	 .fvectors 0xFFFFFFFC: AT(0xFFFFFFFC)
	{
		KEEP(*(.fvectors))
	} GROUP_LINK_IN(ROMABLE_REGION)

	SECTION_PROLOGUE(_TEXT_SECTION_NAME,ROM_START,)
	{
		PLACE_SYMBOL_HERE(_image_text_start);
		*(.text)
		*(.text.*)
		*(P)
		PLACE_SYMBOL_HERE(etext);
	} GROUP_LINK_IN(ROMABLE_REGION)

	PLACE_SYMBOL_HERE(_image_text_end);

	#include <linker/common-rom.ld>

	SECTION_PROLOGUE(.rvectors,,)
	{
		PLACE_SYMBOL_HERE(_rvectors_start);
		KEEP(*(.rvectors))
		PLACE_SYMBOL_HERE(_rvectors_end);
	} GROUP_LINK_IN(ROMABLE_REGION)
	SECTION_PROLOGUE(init,,)
	{
		KEEP(*(.init))
		PLACE_SYMBOL_HERE(__preinit_array_start);
		KEEP(*(.preinit_array))
		PLACE_SYMBOL_HERE(__preinit_array_end);
		__init_array_start = (. + 3) & ~ 3;
		KEEP(*(.init_array))
		KEEP(*(SORT(.init_array.*)))
		PLACE_SYMBOL_HERE(__init_array_end);
		PLACE_SYMBOL_HERE(__fini_array_start);
		KEEP(*(.fini_array))
		KEEP(*(SORT(.fini_array.*)))
		PLACE_SYMBOL_HERE(__fini_array_end);
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
		PLACE_SYMBOL_HERE(_erodata);
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

	/* TODO: is this section necessary? There is a similar section
	 * (_CTOR_SECTION_NAME) in common-rom.ld. This seems to be for
	 * C++ Constructors/Destructors? */
	SECTION_PROLOGUE(tors,,)
	{
		PLACE_SYMBOL_HERE(__CTOR_LIST__);
		. = ALIGN(2);
		PLACE_SYMBOL_HERE(__ctors);
		*(.ctors)
		PLACE_SYMBOL_HERE(__ctors_end);
		PLACE_SYMBOL_HERE(__CTOR_END__);
		PLACE_SYMBOL_HERE(__DTOR_LIST__);
		PLACE_SYMBOL_HERE(__dtors);
		*(.dtors)
		PLACE_SYMBOL_HERE(__dtors_end);
		PLACE_SYMBOL_HERE(__DTOR_END__);
		. = ALIGN(2);
	} GROUP_LINK_IN(ROMABLE_REGION)

	PLACE_SYMBOL_HERE(_rodata_end);
	PLACE_SYMBOL_HERE(_mdata);
	GROUP_END(ROMABLE_REGION)

	GROUP_START(RAMABLE_REGION)

	PLACE_SYMBOL_HERE(_image_ram_start);

	#if CONFIG_SRAM_BASE_ADDRESS == 0
		/* RX65N memory starts at address 0 which can be confused with NULL. To prevent this, block
		 * the first memory page (16 Bytes).
		 */
		SECTION_DATA_PROLOGUE(.null_blocker,,)
		{
			. = 0x10;
		} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
	#endif

	SECTION_DATA_PROLOGUE(_DATA_SECTION_NAME,,)
	{
		PLACE_SYMBOL_HERE(_data);
		*(.data)
		*(.data.*)
		*(D)
		*(D_1)
		*(D_2)
		*(.gnu.linkonce.*)

#include <snippets-rwdata.ld>
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

	/* the sections defined in common-ram.ld have to be initialized on
	 * reset as well, to place them before _edata */
#include <linker/common-ram.ld>

	PLACE_SYMBOL_HERE(_edata);

	SECTION_PROLOGUE(gcc_exc,,)
	{
		*(.gcc_exc)
	} GROUP_LINK_IN(RAMABLE_REGION)

	SECTION_PROLOGUE(_BSS_SECTION_NAME,,)
	{
		PLACE_SYMBOL_HERE(_bss);
		*(.bss)
		*(.bss.**)
		*(COMMON)
		*(B)
		*(B_1)
		*(B_2)
	} GROUP_LINK_IN(RAMABLE_REGION)

	PLACE_SYMBOL_HERE(_ebss);

#if CONFIG_ETH_RX65N
	/* Descriptors and Buffers for the Renesas RX65N ethernet interface.
	 * It is essential that these are correctly aligned, corresponding
	 * to the EDMR register (EDMR.BIT.DL = 0 (default) -> 16 bytes)*/
	SECTION_PROLOGUE(B_ETHERNET_BUFFERS_1, ,ALIGN(32))
	{
	_B_ETHERNET_BUFFERS_1_start = .;
	*(B_ETHERNET_BUFFERS_1)
	_B_ETHERNET_BUFFERS_1_end = .;
	} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

	SECTION_PROLOGUE(B_RX_DESC_1, ,ALIGN(16))
	{
	_B_RX_DESC_1_start = .;
	*(B_RX_DESC_1)
	_B_RX_DESC_1_end = .;
	} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

	SECTION_PROLOGUE(B_TX_DESC_1, ,ALIGN(16))
	{
	_B_TX_DESC_1_start = .;
	*(B_TX_DESC_1)
	_B_TX_DESC_1_end = .;
	} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
#endif /* CONFIG_ETH_RX65N */

#include <linker/common-noinit.ld>

	PLACE_SYMBOL_HERE(_image_ram_end);
	PLACE_SYMBOL_HERE(_end);

	GROUP_END(RAMABLE_REGION)

	.ofs_mde 0xFE7F5D00: AT(0xFE7F5D00)
	{
		KEEP(*(.ofs_mde))
	} > OFS
	.ofs_tminf 0xFE7F5D10: AT(0xFE7F5D10)
	{
		KEEP(*(.ofs_tminf))
	} > OFS
	.ofs_spcc 0xFE7F5D40: AT(0xFE7F5D40)
	{
		KEEP(*(.ofs_spcc))
	} > OFS
	.ofs_tmef 0xFE7F5D48: AT(0xFE7F5D48)
	{
		KEEP(*(.ofs_tmef))
	} > OFS
	.ofs_osis 0xFE7F5D50: AT(0xFE7F5D50)
	{
		KEEP(*(.ofs_osis))
	} > OFS
	.ofs_faw 0xFE7F5D64: AT(0xFE7F5D64)
	{
		KEEP(*(.ofs_faw))
	} > OFS
	.ofs_romcode 0xFE7F5D70: AT(0xFE7F5D70)
	{
		KEEP(*(.ofs_romcode))
	} > OFS

/* Located in generated directory. This file is populated by the
* zephyr_linker_sources() CMake function.
*/
#include <snippets-sections.ld>

#include <linker/debug-sections.ld>

}
