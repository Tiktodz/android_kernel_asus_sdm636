#ifndef __ASM_SPINLOCK_H
#define __ASM_SPINLOCK_H

#include <asm/barrier.h>
#include <asm/processor.h>

#ifdef CONFIG_METAG_ATOMICITY_LOCK1
#include <asm/spinlock_lock1.h>
#else
#include <asm/spinlock_lnkget.h>
#endif

/*
 * both lock1 and lnkget are test-and-set spinlocks with 0 unlocked and 1
 * locked.
 */

static inline void arch_spin_unlock_wait(arch_spinlock_t *lock)
{
	smp_cond_load_acquire(&lock->lock, !VAL);
}

#endif /* __ASM_SPINLOCK_H */
