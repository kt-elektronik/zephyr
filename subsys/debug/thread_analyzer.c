/*
 * Copyright (c) 2019 - 2020 Nordic Semiconductor ASA
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 *  @brief Thread analyzer implementation
 */

#include <kernel.h>
#include <kernel_internal.h>
#include <debug/thread_analyzer.h>
#include <debug/stack.h>
#include <kernel.h>
#include <logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(thread_analyzer, CONFIG_THREAD_ANALYZER_LOG_LEVEL);

#if IS_ENABLED(CONFIG_THREAD_ANALYZER_USE_PRINTK)
#define THREAD_ANALYZER_PRINT(...) printk(__VA_ARGS__)
#define THREAD_ANALYZER_FMT(str)   str "\n"
#define THREAD_ANALYZER_VSTR(str)  (str)
#else
#define THREAD_ANALYZER_PRINT(...) LOG_INF(__VA_ARGS__)
#define THREAD_ANALYZER_FMT(str)   str
#define THREAD_ANALYZER_VSTR(str)  log_strdup(str)
#endif

/* @brief Maximum length of the pointer when converted to string
 *
 * Pointer is converted to string in hexadecimal form.
 * It would use 2 hex digits for every single byte of the pointer
 * but some implementations adds 0x prefix when used with %p format option.
 */
#define PTR_STR_MAXLEN (sizeof(void *) * 2 + 2)

static void thread_print_cb(struct thread_analyzer_info *info)
{
	size_t pcnt = (info->stack_used * 100U) / info->stack_size;
#ifdef CONFIG_THREAD_RUNTIME_STATS
	THREAD_ANALYZER_PRINT(
		THREAD_ANALYZER_FMT(
			" %-20s: STACK: unused %zu usage %zu / %zu (%zu %%); CPU: %u %%"),
		THREAD_ANALYZER_VSTR(info->name),
		info->stack_size - info->stack_used, info->stack_used,
		info->stack_size, pcnt,
		info->utilization);
#else
	THREAD_ANALYZER_PRINT(
		THREAD_ANALYZER_FMT(
			" %-20s: unused %zu usage %zu / %zu (%zu %%)"),
		THREAD_ANALYZER_VSTR(info->name),
		info->stack_size - info->stack_used, info->stack_used,
		info->stack_size, pcnt);
#endif
}

static void thread_analyze_cb(const struct k_thread *cthread, void *user_data)
{
	struct k_thread *thread = (struct k_thread *)cthread;
#ifdef CONFIG_THREAD_RUNTIME_STATS
	k_thread_runtime_stats_t rt_stats_all;
	k_thread_runtime_stats_t rt_stats_thread;
	int ret;
#endif
	size_t size = thread->stack_info.size;
	thread_analyzer_cb cb = user_data;
	struct thread_analyzer_info info;
	char hexname[PTR_STR_MAXLEN + 1];
	const char *name;
	size_t unused;
	int err;



	name = k_thread_name_get((k_tid_t)thread);
	if (!name || name[0] == '\0') {
		name = hexname;
		snprintk(hexname, sizeof(hexname), "%p", (void *)thread);
	}

	err = k_thread_stack_space_get(thread, &unused);
	if (err) {
		THREAD_ANALYZER_PRINT(
			THREAD_ANALYZER_FMT(
				" %-20s: unable to get stack space (%d)"),
			name, err);

		unused = 0;
	}

	info.name = name;
	info.stack_size = size;
	info.stack_used = size - unused;

#ifdef CONFIG_THREAD_RUNTIME_STATS
	ret = 0;

	if (k_thread_runtime_stats_get(thread, &rt_stats_thread) != 0) {
		ret++;
	}

	if (k_thread_runtime_stats_all_get(&rt_stats_all) != 0) {
		ret++;
	}
	if (ret == 0) {
		info.utilization = (rt_stats_thread.execution_cycles * 100U) /
			rt_stats_all.execution_cycles;
	}
#endif
	cb(&info);
}

void thread_analyzer_run(thread_analyzer_cb cb)
{
	if (IS_ENABLED(CONFIG_THREAD_ANALYZER_RUN_UNLOCKED)) {
		k_thread_foreach_unlocked(thread_analyze_cb, cb);
	} else {
		k_thread_foreach(thread_analyze_cb, cb);
	}
}

/**
 * @brief output isr stack information
 *
 * Based on cmd_kernel_stacks() in zephyr\subsys\shell\modules\kernel_service.c
 */
static void print_irq_stack(void)
{
	uint8_t *buf;
	size_t size, unused, pcnt;

	buf = Z_KERNEL_STACK_BUFFER(z_interrupt_stacks[0]);
	size = K_KERNEL_STACK_SIZEOF(z_interrupt_stacks[0]);

	unused = 0;
	for (size_t i = 0; i < size; i++) {
		if (buf[i] == 0xAAU) {
			unused++;
		} else {
			break;
		}
	}
	pcnt = ((size-unused) * 100U) / size;

	THREAD_ANALYZER_PRINT(
#ifdef CONFIG_THREAD_RUNTIME_STATS
		THREAD_ANALYZER_FMT(
			" %-20s: STACK: unused %zu usage %zu / %zu (%zu %%)"),
#else
		THREAD_ANALYZER_FMT(
			" %-20s: unused %zu usage %zu / %zu (%zu %%)"),
#endif
		THREAD_ANALYZER_VSTR("IRQ"),
		unused, size - unused, size, pcnt);
}

void thread_analyzer_print(void)
{
	THREAD_ANALYZER_PRINT(THREAD_ANALYZER_FMT("Thread analyze:"));
	print_irq_stack();
	thread_analyzer_run(thread_print_cb);
}

#if IS_ENABLED(CONFIG_THREAD_ANALYZER_AUTO)

void thread_analyzer_auto(void)
{
	for (;;) {
		thread_analyzer_print();
		k_sleep(K_SECONDS(CONFIG_THREAD_ANALYZER_AUTO_INTERVAL));
	}
}

K_THREAD_DEFINE(thread_analyzer,
		CONFIG_THREAD_ANALYZER_AUTO_STACK_SIZE,
		thread_analyzer_auto,
		NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO,
		0, 0);

#endif
