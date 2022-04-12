/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <kernel_internal.h>
#include <ksched.h>
#include <logging/log.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

/* variables to store the arguments of z_rxv2_context_switch_isr() (zephyr\arch\rxv2\core\switch.S)
 * when performing a cooperative thread switch. In that case, z_rxv2_context_switch_isr() triggers
 * unmaskable interrupt 1 to actually perform the switch. The ISR to interrupt 1
 * (switch_isr_wrapper()) reads the arguments from these variables.
 */
void *coop_switch_to;
void **coop_switched_from;

/* the initial content of the stack */
struct init_stack_frame {
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r4;
	uint32_t r5;
	uint32_t r6;
	uint32_t r7;
	uint32_t r8;
	uint32_t r9;
	uint32_t r10;
	uint32_t r11;
	uint32_t r12;
	uint32_t r13;
	uint32_t r14;
	uint32_t r15;
	uint32_t entry_point;
	uint32_t psw;
};

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
		     char *stack_ptr, k_thread_entry_t entry, void *arg1,
		     void *arg2, void *arg3)
{
	struct init_stack_frame *iframe;
	iframe = Z_STACK_PTR_TO_FRAME(struct init_stack_frame, stack_ptr);

	/* initial value for the PSW (bits U and I are set) */
	iframe->psw = 0x30000;
	/* the initial entry point is the function z_thread_entry */
	iframe->entry_point = (uint32_t)z_thread_entry;
	/* arguments for the call of z_thread_entry (to be written to r1-r4) */
	iframe->r1 = (uint32_t)entry;
	iframe->r2 = (uint32_t)arg1;
	iframe->r3 = (uint32_t)arg2;
	iframe->r4 = (uint32_t)arg3;
	/* for debugging: */
	iframe->r5 = 5;
	iframe->r6 = 6;
	iframe->r7 = 7;
	iframe->r8 = 8;
	iframe->r9 = 9;
	iframe->r10 = 10;
	iframe->r11 = 11;
	iframe->r12 = 12;
	iframe->r13 = 13;
	iframe->r14 = 14;
	iframe->r15 = 15;

	thread->switch_handle = (void *)iframe;
}

/**
 * @brief determine the program counter of a thread
 *
 * @param thread	thread structure
 *
 * @return		program counter (pc) of the thread
 */
static void *get_thread_pc(const struct k_thread *thread)
{
	struct init_stack_frame *thread_stack_frame;

	if (thread == _current) {
		if (k_is_in_isr()) {
			/* when in an interrupt, the pc of the current thread has been stored
			 * the the interrupt stack on entering the ISR
			 */
			thread_stack_frame =
				(struct init_stack_frame *)((uint32_t)&z_interrupt_stacks[0] +
								CONFIG_ISR_STACK_SIZE -
								sizeof(struct init_stack_frame));
		} else {
			/* the pc of the current thread if not in an interrupt is just pc and
			 * points to this function
			 */
			void *thread_pc;

			__asm__ volatile("MVFC pc, %0" : "=r"(thread_pc) : : "cc");
			return thread_pc;
		}
	} else {
		/* for non-current threads, the stack state - including the pc - has been stored
		 * on the thread stack as init_stack_frame struct
		 */
		thread_stack_frame = (struct init_stack_frame *)thread->switch_handle;
	}
	return (void *)thread_stack_frame->entry_point;
}

/**
 * @brief return the stack pointer of a thread
 *
 * @param thread	thread structure
 *
 * @return		stack pointer (sp) of the thread
 */
static void *get_thread_sp(const struct k_thread *thread)
{
	void *thread_sp;

	if (thread == _current) {
		__asm__ volatile("MVFC usp, %0" : "=r"(thread_sp) : : "cc");
	} else {
		thread_sp = (void *)((uint32_t)thread->switch_handle
					+ sizeof(struct init_stack_frame));
	}

	return thread_sp;
}

/**
 * @brief output the state of a thread to the log output
 *
 * @param thread	pointer to the thread strcture
 * @param user_data	unused
 */
static void thread_state_cb(const struct k_thread *thread, void *user_data)
{
	ARG_UNUSED(user_data);

	#if defined(CONFIG_THREAD_STACK_INFO)
		void *thread_sp = get_thread_sp(thread);

		LOG_INF(" %-20s: PC: %10p SP: %10p ( %10p - %10p ) prio: %3d state: %-10s%s",
			log_strdup(k_thread_name_get((k_tid_t)thread)),
			get_thread_pc(thread), thread_sp,
			(void *)thread->stack_info.start,
			(void *)((uint32_t)thread->stack_info.start + thread->stack_info.size),
			k_thread_priority_get((k_tid_t)thread),
			log_strdup(k_thread_state_str((k_tid_t)thread)),
			log_strdup((thread == _current)?" (current)":""));
		if ((uint32_t)thread_sp < (uint32_t)thread->stack_info.start) {
			LOG_INF("                       Stack overflow");
		}
		if ((uint32_t)thread_sp > ((uint32_t)thread->stack_info.start +
								thread->stack_info.size)) {
			LOG_INF("                       Stack underflow");
		}
	#else
		LOG_INF(" %-20s: PC: %10p SP: %10p prio: %3d state: %-10s%s",
			log_strdup(k_thread_name_get((k_tid_t)thread)),
			get_thread_pc(thread), get_thread_sp(thread),
			k_thread_priority_get((k_tid_t)thread),
			log_strdup(k_thread_state_str((k_tid_t)thread)),
			log_strdup((thread == _current)?" (current)":""));
	#endif
}

/**
 * @brief output information about all active thread to the log system
 */
void list_thread_states(void)
{
	void *isr_sp;

	__asm__ volatile("MVFC isp, %0" : "=r"(isr_sp) : : "cc");

	if (k_is_in_isr()) {
		void *isr_pc;

		__asm__ volatile("MVFC pc, %0" : "=r"(isr_pc) : : "cc");
		LOG_INF(" IRQ                 : PC: %10p SP: %10p ( %10p - %10p )",
			isr_pc, isr_sp,
			Z_KERNEL_STACK_BUFFER(z_interrupt_stacks[0]),
			Z_KERNEL_STACK_BUFFER(z_interrupt_stacks[0])
			+ K_KERNEL_STACK_SIZEOF(z_interrupt_stacks[0]));
	} else {
		LOG_INF(" IRQ (not in ISR)    :                SP: %10p ( %10p - %10p )",
			isr_sp,
			Z_KERNEL_STACK_BUFFER(z_interrupt_stacks[0]),
			Z_KERNEL_STACK_BUFFER(z_interrupt_stacks[0])
			+ K_KERNEL_STACK_SIZEOF(z_interrupt_stacks[0]));

	}

	k_thread_foreach(thread_state_cb, NULL);
}
