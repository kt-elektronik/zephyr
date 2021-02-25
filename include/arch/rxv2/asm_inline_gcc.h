/* asm_inline_gcc.h - ARC inline assembler and macros for public functions */

/*
 * Copyright (c) 2015 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RXV2_ASM_INLINE_GCC_H_
#define ZEPHYR_INCLUDE_ARCH_RXV2_ASM_INLINE_GCC_H_

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>
#include <stddef.h>
#include <irq.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 *  @brief read timestamp register (CPU frequency)
 */
//extern uint64_t z_tsc_read(void); // check if: required

static ALWAYS_INLINE void __DSB(void){
//	__asm__ volatile ("dsb sy" : : : "memory"); // check if: TODO: replace with rxv2 code
}

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_RXV2_ASM_INLINE_GCC_H_ */


// /* GNURX specific public inline assembler functions and macros */
// 
// /*
//  *
//  * SPDX-License-Identifier: Apache-2.0
//  */
// 
// /* Either public functions or macros or invoked by public functions */
// 
// #ifndef ZEPHYR_INCLUDE_ARCH_RX_ASM_INLINE_GCC_H_
// #define ZEPHYR_INCLUDE_ARCH_RX_ASM_INLINE_GCC_H_
// 
// /*
//  * The file must not be included directly
//  * Include arch/cpu.h instead
//  */
// 
// #ifndef _ASMLANGUAGE
// 
// #include <zephyr/types.h>
// #include <arch/rxv2/exc.h>
// #include <irq.h>
// 
// #if defined(CONFIG_CPU_CORTEX_R)
// #include <arch/arm/aarch32/cortex_a_r/cpu.h>
// #endif
// 
// #ifdef __cplusplus
// extern "C" {
// #endif
// 
// /* On ARMv7-M and ARMv8-M Mainline CPUs, this function prevents regular
//  * exceptions (i.e. with interrupt priority lower than or equal to
//  * _EXC_IRQ_DEFAULT_PRIO) from interrupting the CPU. NMI, Faults, SVC,
//  * and Zero Latency IRQs (if supported) may still interrupt the CPU.
//  *
//  * On ARMv6-M and ARMv8-M Baseline CPUs, this function reads the value of
//  * PRIMASK which shows if interrupts are enabled, then disables all interrupts
//  * except NMI.
//  */
// 
// static ALWAYS_INLINE unsigned int arch_irq_lock(void)
// {
// 	unsigned int key;
// 
// #if defined(CONFIG_RXV2)
//   __asm volatile("clrpsw i;");
// //#warning return value does not meet the specs yet --> TODO
// 
// #else
// #error Unknown architecture
// #endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
// 
// 	return key;
// }
// 
// 
// /* On Cortex-M0/M0+, this enables all interrupts if they were not
//  * previously disabled.
//  */
// 
// static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
// {
// #if defined(CONFIG_RXV2)
// 	// if (key) {
// 	//   return;
// 	// }  
//   __asm volatile("setpsw i;");
//   
// //#warning parameter key not evaluated --> TODO
// 
// #else
// #error Unknown architecture
// #endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
// }
// 
// static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
// {
// 	/* This convention works for both PRIMASK and BASEPRI */
// 	return key == 0;
// }
// 
// #ifdef __cplusplus
// }
// #endif
// 
// #endif /* _ASMLANGUAGE */
// 
// #endif /* ZEPHYR_INCLUDE_ARCH_RX_ASM_INLINE_GCC_H_ */
// 