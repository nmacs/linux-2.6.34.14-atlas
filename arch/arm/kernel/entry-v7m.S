/*
 * linux/arch/arm/kernel/entry-v7m.S
 *
 * Copyright (C) 2008 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Low-level vector interface routines for the ARMv7-M architecture
 */
#include <asm/memory.h>
#include <asm/glue.h>
#include <asm/thread_notify.h>

#include <mach/entry-macro.S>

#include "entry-header.S"

#ifdef CONFIG_PREEMPT
#error "CONFIG_PREEMPT not supported on the current ARMv7M implementation"
#endif
#ifdef CONFIG_TRACE_IRQFLAGS
#error "CONFIG_TRACE_IRQFLAGS not supported on the current ARMv7M implementation"
#endif

__invalid_entry:
	v7m_exception_entry
#ifdef CONFIG_PRINTK
	adr	r0, strerr
	mrs	r1, ipsr
	mov	r2, lr
	bl	printk
#endif
	mov	r0, sp
	bl	show_regs
1:	b	1b
ENDPROC(__invalid_entry)

__exception_entry:
	v7m_exception_entry
#ifdef CONFIG_PLAT_STELLARIS
	mov r0, sp
	bl stellaris_die
#else
	ldr r0, =strex
	mov r1, sp
	mov r2, #0
	bl die
#endif
ENDPROC(__exception_entry)

strerr:	.asciz	"\nUnhandled exception: IPSR = %08lx LR = %08lx\n"

#ifndef CONFIG_PLAT_STELLARIS
strex: .asciz "MCU exception"
#endif

	.align	2
__irq_entry:
	v7m_exception_entry

	@
	@ Invoke the IRQ handler
	@
	mrs	r0, ipsr
	and	r0, #0xff
	sub	r0, #16			@ IRQ number
	mov	r1, sp
	@ routine called with r0 = irq number, r1 = struct pt_regs *
	bl	asm_do_IRQ

	@
	@ Check for any pending work if returning to user
	@
	ldr	lr, [sp, #S_EXC_RET]
	tst	lr, #0x8		@ check the return stack
	beq	2f			@ returning to handler mode
	get_thread_info tsk
	ldr	r1, [tsk, #TI_FLAGS]
	tst	r1, #_TIF_WORK_MASK
	beq	2f			@ no work pending
	ldr	r1, =0xe000ed04		@ ICSR
	mov	r0, #1 << 28		@ ICSR.PENDSVSET
	str	r0, [r1]		@ raise PendSV

2:
	v7m_exception_fast_exit
ENDPROC(__irq_entry)

__pendsv_entry:
	v7m_exception_entry

	ldr	r1, =0xe000ed04		@ ICSR
	mov	r0, #1 << 27		@ ICSR.PENDSVCLR
	str	r0, [r1]		@ clear PendSV

	@ execute the pending work, including reschedule
	get_thread_info tsk
	mov	why, #0
	b	ret_to_user
ENDPROC(__pendsv_entry)

/*
 * Register switch for ARMv7-M processors.
 * r0 = previous task_struct, r1 = previous thread_info, r2 = next thread_info
 * previous and next are guaranteed not to be the same.
 */
ENTRY(__switch_to)
	.fnstart
	.cantunwind
	add	ip, r1, #TI_CPU_SAVE
	stmia	ip!, {r4 - r11}		@ Store most regs on stack
	str	sp, [ip], #4
	str	lr, [ip], #4
	mov	r5, r0
	add	r4, r2, #TI_CPU_SAVE
	ldr	r0, =thread_notify_head
	mov	r1, #THREAD_NOTIFY_SWITCH
	bl	atomic_notifier_call_chain
	mov	ip, r4
	mov	r0, r5
	ldmia	ip!, {r4 - r11}		@ Load all regs saved previously
	ldr	sp, [ip], #4
	ldr	pc, [ip]
	.fnend
ENDPROC(__switch_to)

	.data
	.align	8
/*
 * Vector table (64 words => 256 bytes natural alignment)
 */
ENTRY(vector_table)
	.long	0			@ 0 - Reset stack pointer
	.long	__invalid_entry		@ 1 - Reset
	.long	__invalid_entry		@ 2 - NMI
	.long	__exception_entry		@ 3 - HardFault
	.long	__exception_entry		@ 4 - MemManage
	.long	__exception_entry		@ 5 - BusFault
	.long	__exception_entry		@ 6 - UsageFault
	.long	__invalid_entry		@ 7 - Reserved
	.long	__invalid_entry		@ 8 - Reserved
	.long	__invalid_entry		@ 9 - Reserved
	.long	__invalid_entry		@ 10 - Reserved
	.long	vector_swi		@ 11 - SVCall
	.long	__invalid_entry		@ 12 - Debug Monitor
	.long	__invalid_entry		@ 13 - Reserved
	.long	__pendsv_entry		@ 14 - PendSV
	.long	__invalid_entry		@ 15 - SysTick
#if   defined(CONFIG_ARCH_LM3S1D21)
	.rept 71 - 16 @ 16..70 - External Interrupts
#elif defined(CONFIG_ARCH_TM4C12)
	.rept 114 - 16 @ 16..70 - External Interrupts
#else
	.rept	64 - 16 @ 16..64 - External Interrupts
#endif
	.long __irq_entry
	.endr
ENTRY(vector_table_end)

	.globl	cr_alignment
	.globl	cr_no_alignment
cr_alignment:
	.space	4
cr_no_alignment:
	.space	4
