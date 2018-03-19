/*
 * Header file for Linux support for U-Boot
 *
 * Adaptation from kernel to U-Boot
 *
 *  Copyright (C) 2005-2007 Samsung Electronics
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __UBOOT_LINUX_PORTING_COMPAT_H
#define __UBOOT_LINUX_PORTING_COMPAT_H

#include <common.h>
#include <compiler.h>
#include <malloc.h>
#include <div64.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/rbtree.h>
#include <linux/string.h>

#define DPRINTK(format, args...)					\
do {									\
	printf("%s[%d]: " format "\n", __func__, __LINE__, ##args);	\
} while (0)

#ifndef msleep
#define msleep	mdelay
#endif

#ifndef min_t
#define min_t(type, x, y) ({			\
	type __min1 = (x);			\
	type __min2 = (y);			\
	__min1 < __min2 ? __min1: __min2; })
#endif /* mint_t */

#ifndef max_t
#define max_t(type, x, y) ({			\
	type __max1 = (x);			\
	type __max2 = (y);			\
	__max1 > __max2 ? __max1: __max2; })
#endif /* max_t */

/* FIXME */
#define MKDEV(...)			0
#define MAJOR(dev)			0
#define MINOR(dev)			0

#define alloc_chrdev_region(...)	0
#define unregister_chrdev_region(...)

#define class_create(...)		__builtin_return_address(0)
#define class_create_file(...)		0
#define class_remove_file(...)
#define class_destroy(...)
#define misc_register(...)		0
#define misc_deregister(...)

/* vmt.c */
#define device_register(...)		0
#define volume_sysfs_init(...)		0
#define volume_sysfs_close(...)		do { } while (0)

/* kapi.c */

/* eba.c */

/* io.c */
#define init_waitqueue_head(...)	do { } while (0)
#define wait_event_interruptible(...)	0
#define wake_up_interruptible(...)	do { } while (0)
#define print_hex_dump(...)		do { } while (0)
#define dump_stack(...)			do { } while (0)

/* wl.c */
#define task_pid_nr(x)			0
#define set_freezable(...)		do { } while (0)
#define try_to_freeze(...)		0
#define set_current_state(...)		do { } while (0)
#define kthread_should_stop(...)	0
#define schedule()			do { } while (0)

/* upd.c */
static inline unsigned long copy_from_user(void *dest, const void *src,
					   unsigned long count)
{
	memcpy((void *)dest, (void *)src, count);
	return 0;
}

/* common */
typedef int	spinlock_t;
typedef int	wait_queue_head_t;
#define spin_lock_init(...)
#define spin_lock(...)
#define spin_unlock(...)
#define spin_lock_irqsave(lock, flags) do {flags = 1; } while (0)
#define spin_unlock_irqrestore(lock, flags) do {flags = 0; } while (0)

#define mutex_init(...)
#define mutex_lock(...)
#define mutex_unlock(...)

#define init_rwsem(...)			do { } while (0)
#define down_read(...)			do { } while (0)
#define down_write(...)			do { } while (0)
#define down_write_trylock(...)		1
#define up_read(...)			do { } while (0)
#define up_write(...)			do { } while (0)

struct kmem_cache { int i; };
#define kmem_cache_create(...)		1
#define kmem_cache_alloc(obj, gfp)	malloc(sizeof(struct ubi_wl_entry))
#define kmem_cache_free(obj, size)	free(size)
#define kmem_cache_destroy(...)

#define cond_resched()			do { } while (0)
#define yield()				do { } while (0)

#define KERN_WARNING
#define KERN_ERR
#define KERN_NOTICE
#define KERN_DEBUG
#define WARN_ON(x) if (x) {printf("WARNING in %s line %d\n", __func__, __LINE__);}

#define GFP_KERNEL			0
#define GFP_NOFS			1

#define __user
#define __init
#define __exit
#define __initdata
#define __iomem

#define kthread_create(...)	__builtin_return_address(0)
#define kthread_stop(...)	do { } while (0)
#define wake_up_process(...)	do { } while (0)

struct rw_semaphore { int i; };
struct mutex { int i; };
struct kernel_param { int i; };

struct cdev {
	int owner;
	dev_t dev;
};
#define cdev_init(...)		do { } while (0)
#define cdev_add(...)		0
#define cdev_del(...)		do { } while (0)

#define MAX_ERRNO		4095
#define IS_ERR_VALUE(x)		((x) >= (unsigned long)-MAX_ERRNO)

static inline void *ERR_PTR(long error)
{
	return (void *) error;
}

static inline long PTR_ERR(const void *ptr)
{
	return (long) ptr;
}

static inline long IS_ERR(const void *ptr)
{
	return IS_ERR_VALUE((unsigned long)ptr);
}

/* module */
#define THIS_MODULE		0
#define try_module_get(...)	1
#define module_put(...)		do { } while (0)
#define module_init(...)
#define module_exit(...)
#define EXPORT_SYMBOL(...)
#define EXPORT_SYMBOL_GPL(...)
#define module_param_call(...)
#define MODULE_PARM_DESC(...)
#define MODULE_VERSION(...)
#define MODULE_DESCRIPTION(...)
#define MODULE_AUTHOR(...)
#define MODULE_LICENSE(...)
#define MODULE_ALIAS(...)

enum irqreturn {
	IRQ_NONE,
	IRQ_HANDLED,
};

typedef enum irqreturn irqreturn_t;

/* completion */
struct completion {
	unsigned int done;
};

#define COMPLETION_INITIALIZER(work) \
	{ 0 }

/**
 * DECLARE_COMPLETION - declare and initialize a completion structure
 * @work:  identifier for the completion structure
 *
 * This macro declares and initializes a completion structure. Generally used
 * for static declarations. You should use the _ONSTACK variant for automatic
 * variables.
 */
#define DECLARE_COMPLETION(work) \
	struct completion work = COMPLETION_INITIALIZER(work)

#define DECLARE_COMPLETION_ONSTACK(work) DECLARE_COMPLETION(work)

static inline void init_completion(struct completion *x) __attribute__((always_inline));
static inline void init_completion(struct completion *x)
{
	x->done = 0;
}

static inline void complete(struct completion *x) __attribute__((always_inline));
static inline void complete(struct completion *x)
{
	x->done++;
}

/* semaphore */
struct semaphore {
	unsigned int		count;
	unsigned int		wait_list;
};

static inline void down(struct semaphore *sem)
{
	if (sem->count > 0)
		sem->count--;
	else
		sem->wait_list++;
}

static inline void up(struct semaphore *sem)
{
	if (sem->wait_list == 0)
		sem->count++;
	else
		sem->wait_list--;
}

#endif
