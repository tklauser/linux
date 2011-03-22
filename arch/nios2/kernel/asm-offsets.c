/*
 * This program is used to generate definitions needed by
 * assembly language modules.
 *
 * We use the technique used in the OSF Mach kernel code:
 * generate asm statements containing #defines,
 * compile this file to assembler, and then extract the
 * #defines from the assembly-language output.
 */

#include <linux/stddef.h>
#include <linux/sched.h>
#include <linux/kernel_stat.h>
#include <linux/ptrace.h>
#include <linux/hardirq.h>
#include <linux/thread_info.h>
#include <linux/kbuild.h>
#include <asm/irq.h>
#include <asm/nios.h>

int main(void)
{
	/* offsets into the task struct */
	DEFINE(TASK_STATE, offsetof(struct task_struct, state));
	DEFINE(TASK_FLAGS, offsetof(struct task_struct, flags));
	DEFINE(TASK_PTRACE, offsetof(struct task_struct, ptrace));
	DEFINE(TASK_BLOCKED, offsetof(struct task_struct, blocked));
	DEFINE(TASK_THREAD, offsetof(struct task_struct, thread));
	DEFINE(TASK_THREAD_INFO, offsetof(struct task_struct, stack));
	DEFINE(TASK_MM, offsetof(struct task_struct, mm));
	DEFINE(TASK_ACTIVE_MM, offsetof(struct task_struct, active_mm));
	BLANK();

	/* offsets into the irq_cpustat_t struct */
	DEFINE(CPUSTAT_SOFTIRQ_PENDING, offsetof(irq_cpustat_t, __softirq_pending));

	/* offsets into the thread struct */
	DEFINE(THREAD_KSP, offsetof(struct thread_struct, ksp));
	DEFINE(THREAD_KPSR, offsetof(struct thread_struct, kpsr));
#ifndef CONFIG_MMU
	DEFINE(THREAD_KESR, offsetof(struct thread_struct, kesr));
#endif
	DEFINE(THREAD_FLAGS, offsetof(struct thread_struct, flags));
	BLANK();

	/* offsets into the pt_regs */
	DEFINE(PT_ORIG_R2, offsetof(struct pt_regs, orig_r2));
#ifdef CONFIG_MMU
	DEFINE(PT_ORIG_R7, offsetof(struct pt_regs, orig_r7));
#endif
	DEFINE(PT_R1, offsetof(struct pt_regs, r1));
	DEFINE(PT_R2, offsetof(struct pt_regs, r2));
	DEFINE(PT_R3, offsetof(struct pt_regs, r3));
	DEFINE(PT_R4, offsetof(struct pt_regs, r4));
	DEFINE(PT_R5, offsetof(struct pt_regs, r5));
	DEFINE(PT_R6, offsetof(struct pt_regs, r6));
	DEFINE(PT_R7, offsetof(struct pt_regs, r7));
	DEFINE(PT_R8, offsetof(struct pt_regs, r8));
	DEFINE(PT_R9, offsetof(struct pt_regs, r9));
	DEFINE(PT_R10, offsetof(struct pt_regs, r10));
	DEFINE(PT_R11, offsetof(struct pt_regs, r11));
	DEFINE(PT_R12, offsetof(struct pt_regs, r12));
	DEFINE(PT_R13, offsetof(struct pt_regs, r13));
	DEFINE(PT_R14, offsetof(struct pt_regs, r14));
	DEFINE(PT_R15, offsetof(struct pt_regs, r15));
	DEFINE(PT_EA, offsetof(struct pt_regs, ea));
	DEFINE(PT_RA, offsetof(struct pt_regs, ra));
	DEFINE(PT_FP, offsetof(struct pt_regs, fp));
	DEFINE(PT_SP, offsetof(struct pt_regs, sp));
	DEFINE(PT_GP, offsetof(struct pt_regs, gp));
	DEFINE(PT_ESTATUS, offsetof(struct pt_regs, estatus));
#ifndef CONFIG_MMU
	DEFINE(PT_STATUS_EXTENSION, offsetof(struct pt_regs, status_extension));
#endif
	DEFINE(PT_REGS_SIZE, sizeof(struct pt_regs));
	BLANK();

	/* offsets into the switch_stack */
	DEFINE(SW_R16, offsetof(struct switch_stack, r16));
	DEFINE(SW_R17, offsetof(struct switch_stack, r17));
	DEFINE(SW_R18, offsetof(struct switch_stack, r18));
	DEFINE(SW_R19, offsetof(struct switch_stack, r19));
	DEFINE(SW_R20, offsetof(struct switch_stack, r20));
	DEFINE(SW_R21, offsetof(struct switch_stack, r21));
	DEFINE(SW_R22, offsetof(struct switch_stack, r22));
	DEFINE(SW_R23, offsetof(struct switch_stack, r23));
	DEFINE(SW_FP, offsetof(struct switch_stack, fp));
	DEFINE(SW_GP, offsetof(struct switch_stack, gp));
	DEFINE(SW_RA, offsetof(struct switch_stack, ra));
	DEFINE(SWITCH_STACK_SIZE, sizeof(struct switch_stack));
	BLANK();

	/* Offsets in thread_info structure, used in assembly code */
	DEFINE(TI_TASK, offsetof(struct thread_info, task));
	DEFINE(TI_EXECDOMAIN, offsetof(struct thread_info, exec_domain));
	DEFINE(TI_FLAGS, offsetof(struct thread_info, flags));
	DEFINE(TI_CPU, offsetof(struct thread_info, cpu));
	DEFINE(TI_PREEMPT_COUNT, offsetof(struct thread_info, preempt_count));
	BLANK();

	return 0;
}
