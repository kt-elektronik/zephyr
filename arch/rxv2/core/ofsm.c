/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *
 * Option-Setting Memory for the RX651. This region of memory (located in flash)
 * determines the state of the MCU after reset and can not be changed on runtime
 *
 * All registers are set to 0xffffffff by default, which are "safe" settings.
 * Please refer to the Renesas RX651 Group User's Manual before changing any of
 * the values as some changes can be permanent or lock access to the device.
 *
 * Address range: 0xFE7F5D00 to 0xFE7F5D7F (128 Bytes)
 */

#define __OFS_MDE __attribute__((section(".ofs_mde")))
#define __OFS_TMINF __attribute__((section(".ofs_tminf")))
#define __OFS_SPCC __attribute__((section(".ofs_spcc")))
#define __OFS_TMEF __attribute__((section(".ofs_tmef")))
#define __OFS_OSIS __attribute__((section(".ofs_osis")))
#define __OFS_FAW __attribute__((section(".ofs_faw")))
#define __OFS_ROMCODE __attribute__((section(".ofs_romcode")))

/* Endian Select Register (MDE) at 0xFE7F5D00
 *
 * b2 to b0: endian select between (0 0 0) for big endian and (1 1 1) for little
 * endian. Set this according to __BYTE_ORDER__ (cf. include\toolchain\gcc.h)
 *
 * b6-b4 (Bank Mode Select) indicate whether the flash is operated in
 * Dual mode (0 0 0) or Linear mode (1 1 1).
 *
 * all other bits are reserved and have to be set to 1
 */
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
const unsigned long __OFS_MDE __MDEreg = 0xfffffff8; /* big */
#else
const unsigned long __OFS_MDE __MDEreg = 0xffffffff; /* little */
#endif

struct st_ofs0 {
#ifdef __RX_LITTLE_ENDIAN__
	unsigned long res0 : 1;
	unsigned long IWDTSTRT : 1;
	unsigned long IWDTTOPS : 2;
	unsigned long IWDTCKS : 4;
	unsigned long IWDTRPES : 2;
	unsigned long IWDTRPSS : 2;
	unsigned long IWDTRSTIRQS : 1;
	unsigned long res1 : 1;
	unsigned long IWDTSLCSTP : 1;
	unsigned long res2 : 2;
	unsigned long WDTSTRT : 1;
	unsigned long WDTTOPS : 2;
	unsigned long WDTCKS : 4;
	unsigned long WDTRPES : 2;
	unsigned long WDTRPSS : 2;
	unsigned long WDTRSTIRQS : 1;
	unsigned long res3 : 3;
#else
	unsigned long res3 : 3;
	unsigned long WDTRSTIRQS : 1;
	unsigned long WDTRPSS : 2;
	unsigned long WDTRPES : 2;
	unsigned long WDTCKS : 4;
	unsigned long WDTTOPS : 2;
	unsigned long WDTSTRT : 1;
	unsigned long res2 : 2;
	unsigned long IWDTSLCSTP : 1;
	unsigned long res1: 1;
	unsigned long IWDTRSTIRQS : 1;
	unsigned long IWDTRPSS : 2;
	unsigned long IWDTRPES : 2;
	unsigned long IWDTCKS : 4;
	unsigned long IWDTTOPS : 2;
	unsigned long IWDTSTRT : 1;
	unsigned long res0 : 1;
#endif /* __RX_LITTLE_ENDIAN__ */
};

/* Option Function Select Register 0 (OFS0) at 0xFE7F5D04 (Watchdog settings) */
#ifdef CONFIG_IWDT_RX65N_AUTO_START_MODE
const struct st_ofs0 __OFS_MDE __OFS0reg = {
	.res3 = 0x7,
	/* default WDT configuration (no auto-start): */
	.WDTRSTIRQS = 1,
	.WDTRPSS = 0x3,
	.WDTRPES = 0x3,
	.WDTCKS = 0xF,
	.WDTTOPS = 0x3,
	.WDTSTRT = 1,
	/* end default WDT configuration */
	.res2 = 0x3,
	.IWDTSLCSTP = CONFIG_IWDT_RX65N_OFS0_IWDTSLCSTP,
	.res1 = 1,       /* default value (reserved) */
	.IWDTRSTIRQS = CONFIG_IWDT_RX65N_OFS0_IWDTRSTIRQS,
	.IWDTRPSS = CONFIG_IWDT_RX65N_OFS0_IWDTRPSS,
	.IWDTRPES = CONFIG_IWDT_RX65N_OFS0_IWDTRPES,
	.IWDTCKS = CONFIG_IWDT_RX65N_OFS0_IWDTCKS,
	.IWDTTOPS = CONFIG_IWDT_RX65N_OFS0_IWDTTOPS,
	.IWDTSTRT = CONFIG_IWDT_RX65N_OFS0_IWDTSTRT,
	.res0 = 1       /* default value (reserved) */
};
#else
const unsigned long __OFS_MDE __OFS0reg = 0xffffffff;
#endif /* CONFIG_IWDT_RX65N_AUTO_START_MODE */

/* Option Function Select Register 1 (OFS1) at 0xFE7F5D08 (Voltage detection and
 * HOCO)
 */
const unsigned long __OFS_MDE __OFS1reg = 0xffffffff;

/* TM (Trusted memory) Identification Data Register (TMINF) at 0xFE7F5D10
 * currently disabled
 */
const unsigned long __OFS_TMINF __TMINFreg = 0xffffffff;

/* Missing: Bank select Register (BANKSEL) at 0xFE7F5D20, which only is relevant
 * in dual flash mode (b6-b4 in __MDEreg set to 0)
 */

/* Serial Programmer Command Control Register (SPCC) at 0xFE7F5D40
 * setting bit 27 of this register to 0 disables the connection of a serial
 * programmer
 */
const unsigned long __OFS_SPCC __SPCCreg = 0xffffffff;

/* TM (Trusted Memory) Enable Flag Register (TMEF) at 0xFE7F5D48
 * currently disabled (set b26 to b24 to enable)
 */
const unsigned long __OFS_TMEF __TMEFreg = 0xffffffff;

/* OCD/Serial Programmer ID Setting Register (OSIS) at 0xFE7F5D50h to 0xFE7F5D5F */
const unsigned long __OFS_OSIS __OSISreg[4] = {
	0xffffffff,
	0xffffffff,
	0xffffffff,
	0xffffffff,
};

/* Flash Access Window Setting Register (FAW) at 0xFE7F5D64
 * WARNING: setting bit 15 of this register to 0 is PERMANENT and the bit can
 *          never be set to 1 again on the device
 */
const unsigned long __OFS_FAW __FAWreg = 0xffffffff;

/* ROM Code Protection Register (ROMCODE) at 0xFE7F5D70 */
const unsigned long __OFS_ROMCODE __ROMCODEreg = 0xffffffff;
