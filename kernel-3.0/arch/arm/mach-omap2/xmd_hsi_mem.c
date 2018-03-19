/*
 * xmd_hsi_mem.c
 *
 * Copyright (C) 2011-2012 Intel Mobile Communications GmbH
 *
 * Author: Chaitanya <Chaitanya.Khened@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include "xmd-ch.h"
#include "xmd_hsi_mem.h"
#include "xmd-hsi-ll-cfg.h"

#ifdef HSI_MEM_ENABLE_LOGS
#define xmd_mem_dbg(fmt, args...)  printk("\nhsi_mem: " fmt "\n", ## args)
#else
#define xmd_mem_dbg(fmt, args...)  do { } while (0)
#endif

#if defined (HSI_PRE_ALLOC_BUFFERS)
u32 *hsi_mem_blk0_ptr[HSI_MEM_NUM_OF_BLK0];
u32 *hsi_mem_blk1_ptr[HSI_MEM_NUM_OF_BLK1];
u32 *hsi_mem_blk2_ptr[HSI_MEM_NUM_OF_BLK2];
u32 *hsi_mem_blk3_ptr[HSI_MEM_NUM_OF_BLK3];

#if !defined (HSI_MEM_USE_DMA_BUFS)
/*Declared as int to make sure that buffers are word aligned */
u32 hsi_mem_blk0[HSI_MEM_NUM_OF_BLK0][HSI_MEM_BLOCK0_SIZE/4];
u32 hsi_mem_blk1[HSI_MEM_NUM_OF_BLK1][HSI_MEM_BLOCK1_SIZE/4];
u32 hsi_mem_blk2[HSI_MEM_NUM_OF_BLK2][HSI_MEM_BLOCK2_SIZE/4];
u32 hsi_mem_blk3[HSI_MEM_NUM_OF_BLK3][HSI_MEM_BLOCK3_SIZE/4];
#else
u32 *hsi_mem_blk0[HSI_MEM_NUM_OF_BLK0];
u32 *hsi_mem_blk1[HSI_MEM_NUM_OF_BLK1];
u32 *hsi_mem_blk2[HSI_MEM_NUM_OF_BLK2];
u32 *hsi_mem_blk3[HSI_MEM_NUM_OF_BLK3];
#endif

u8 *hsi_mem_fb_block[HSI_MEM_NUM_OF_FB_BLK] = {NULL};
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
/* After hsi_mem_reinit() is added, memory allocation and free need to be taken more care of */
u8 *hsi_mem_fb_block_backup[HSI_MEM_NUM_OF_FB_BLK] = {NULL};
#endif
#endif
static spinlock_t hsi_mem_lock;

/*****************************************************************************/
/* Function:... hsi_mem_mem_init                                             */
/* Description: Initialization of the memory pools.                          */
/*****************************************************************************/
int hsi_mem_init(void)
{
	static int is_hsi_mem_init;

	if(!is_hsi_mem_init) {
#if defined (HSI_PRE_ALLOC_BUFFERS) && defined (HSI_MEM_USE_DMA_BUFS)
	u32 i;

	for(i=0; i < HSI_MEM_NUM_OF_BLK0; i++) {
		hsi_mem_blk0[i] =  (u32*) kmalloc(HSI_MEM_BLOCK0_SIZE,
											GFP_DMA|GFP_KERNEL);
		if(hsi_mem_blk0[i] == NULL)
			xmd_mem_dbg("Failed to alloc HSI_MEM_BLOCK0  memory.i=%d",i);
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK1; i++) {
		hsi_mem_blk1[i] =  (u32*) kmalloc(HSI_MEM_BLOCK1_SIZE,
											GFP_DMA|GFP_KERNEL);
		if(hsi_mem_blk1[i] == NULL)
			xmd_mem_dbg("Failed to alloc HSI_MEM_BLOCK1  memory.i=%d",i);
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK2; i++) {
		hsi_mem_blk2[i] =  (u32*) kmalloc(HSI_MEM_BLOCK2_SIZE,
											GFP_DMA|GFP_KERNEL);
		if(hsi_mem_blk2[i] == NULL)
			xmd_mem_dbg("Failed to alloc HSI_MEM_BLOCK2  memory.i=%d",i);
	}

	for(i=0; i < HSI_MEM_NUM_OF_BLK3; i++) {
		hsi_mem_blk3[i] =  (u32*) kmalloc(HSI_MEM_BLOCK3_SIZE,
											GFP_DMA|GFP_KERNEL);
		if(hsi_mem_blk3[i] == NULL)
			xmd_mem_dbg("Failed to alloc HSI_MEM_BLOCK3  memory.i=%d",i);
	}
#endif
	spin_lock_init(&hsi_mem_lock);
	is_hsi_mem_init = 1;
	}
	return 0;
}

/*****************************************************************************/
/* Function:... hsi_mem_uninit                                               */
/* Description: Freeing of memory pools                                      */
/*****************************************************************************/
int hsi_mem_uninit(void)
{
	return 0;
}

#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
/*****************************************************************************/
/* Function:... hsi_mem_reinit                                               */
/* Description: Initialization of the memory pools                           */
/*****************************************************************************/
int hsi_mem_reinit(void)
{
#if defined (HSI_PRE_ALLOC_BUFFERS)
	unsigned int i;

	spin_lock_bh(&hsi_mem_lock);

	xmd_mem_dbg("hsi_mem_reinit().");

	for(i = 0; i < HSI_MEM_NUM_OF_BLK0; i++) {
		hsi_mem_blk0_ptr[i] = NULL;
	}

	for(i = 0; i < HSI_MEM_NUM_OF_BLK1; i++) {
		hsi_mem_blk1_ptr[i] = NULL;
	}

	for(i = 0; i < HSI_MEM_NUM_OF_BLK2; i++) {
		hsi_mem_blk2_ptr[i] = NULL;
	}

	for(i = 0; i < HSI_MEM_NUM_OF_BLK3; i++) {
		hsi_mem_blk3_ptr[i] = NULL;
	}

	/* free fallback memory */
	for (i = 0; i < HSI_MEM_NUM_OF_FB_BLK; i++) {
		if (hsi_mem_fb_block[i] != NULL) {
			kfree(hsi_mem_fb_block[i]);
			hsi_mem_fb_block[i] = NULL;
		}
	}

	/*TODO : Free allocated buffer from BUF retry WQ.*/

	spin_unlock_bh(&hsi_mem_lock);
#endif
	return 0;
}
#endif

/*****************************************************************************/
/* Function:... hsi_mem_alloc                                                */
/* Description: allocates a block with the requested size. If no memory      */
/* block of the requested size available, NULL will be returned. The         */
/* returned pointer will be word aligned                                     */
/*****************************************************************************/
void *hsi_mem_alloc(int size)
{
	void *buf = NULL;
#if defined (HSI_PRE_ALLOC_BUFFERS)
	u32 i;
	spin_lock_bh(&hsi_mem_lock);

	if(size == 0) {
		goto quit_mem_alloc;
	}

	if(size <= HSI_MEM_BLOCK0_SIZE) {
		for(i = 0; i < HSI_MEM_NUM_OF_BLK0; i++) {
			if (hsi_mem_blk0_ptr[i] == NULL) {
				hsi_mem_blk0_ptr[i] = hsi_mem_blk0[i];
				buf = (void *)hsi_mem_blk0_ptr[i];
#ifdef HSI_MEM_LOW_MEMORY_THRESHOLD
				if ((HSI_MEM_NUM_OF_BLK0 - i) < HSI_MEM_LOW_MEMORY_THRESHOLD)
					xmd_mem_dbg("WARN: HSI_MEM_BLOCK0 used/total=%d/%d.", i + 1, HSI_MEM_NUM_OF_BLK0);
#endif
				goto quit_mem_alloc;
			}
		}
		xmd_mem_dbg("Running out of HSI_MEM_BLOCK0 memory.");
	}

	if((size > HSI_MEM_BLOCK0_SIZE) && (size <= HSI_MEM_BLOCK1_SIZE)) {
		for(i = 0; i < HSI_MEM_NUM_OF_BLK1; i++) {
			if (hsi_mem_blk1_ptr[i] == NULL) {
				hsi_mem_blk1_ptr[i] = hsi_mem_blk1[i];
				buf = (void *)hsi_mem_blk1_ptr[i];
#ifdef HSI_MEM_LOW_MEMORY_THRESHOLD
				if ((HSI_MEM_NUM_OF_BLK1 - i) < HSI_MEM_LOW_MEMORY_THRESHOLD)
					xmd_mem_dbg("WARN: HSI_MEM_BLOCK1 used/total=%d/%d.", i + 1, HSI_MEM_NUM_OF_BLK1);
#endif
				goto quit_mem_alloc;
			}
		}
		xmd_mem_dbg("Running out of HSI_MEM_BLOCK1 memory.");
	}

	if((size > HSI_MEM_BLOCK1_SIZE) && (size <= HSI_MEM_BLOCK2_SIZE)) {
		for(i = 0; i < HSI_MEM_NUM_OF_BLK2; i++) {
			if (hsi_mem_blk2_ptr[i] == NULL) {
				hsi_mem_blk2_ptr[i] = hsi_mem_blk2[i];
				buf = (void *)hsi_mem_blk2_ptr[i];
#ifdef HSI_MEM_LOW_MEMORY_THRESHOLD
				if ((HSI_MEM_NUM_OF_BLK2 - i) < HSI_MEM_LOW_MEMORY_THRESHOLD)
					xmd_mem_dbg("WARN: HSI_MEM_BLOCK2 used/total=%d/%d.", i + 1, HSI_MEM_NUM_OF_BLK2);
#endif
				goto quit_mem_alloc;
			}
		}
		xmd_mem_dbg("Running out of HSI_MEM_BLOCK2 memory.");
	}

	if((size > HSI_MEM_BLOCK2_SIZE) && (size <= HSI_MEM_BLOCK3_SIZE)) {
		for(i = 0; i < HSI_MEM_NUM_OF_BLK3; i++) {
			if (hsi_mem_blk3_ptr[i] == NULL) {
				hsi_mem_blk3_ptr[i] = hsi_mem_blk3[i];
				buf = (void *)hsi_mem_blk3_ptr[i];
#ifdef HSI_MEM_LOW_MEMORY_THRESHOLD
				if ((HSI_MEM_NUM_OF_BLK3 - i) < HSI_MEM_LOW_MEMORY_THRESHOLD)
					xmd_mem_dbg("WARN: HSI_MEM_BLOCK3 used/total=%d/%d.", i + 1, HSI_MEM_NUM_OF_BLK3);
#endif
				goto quit_mem_alloc;
			}
		}
		xmd_mem_dbg("Running out of HSI_MEM_BLOCK3 memory.");
	}

	if(size >= HSI_MEM_BLOCK3_SIZE) {
		xmd_mem_dbg("Requesting Fall back memory of size %d.", size);
	}

	/* fallback to kernel for memory */
	for (i=0; i < HSI_MEM_NUM_OF_FB_BLK; i++) {
		if (hsi_mem_fb_block[i] == NULL) {
			hsi_mem_fb_block[i] = (u8*) kmalloc(size,
												GFP_DMA|GFP_ATOMIC);
			if(hsi_mem_fb_block[i] == NULL) {
				xmd_mem_dbg("Failed to alloc fall-back mem,returning NULL.");
				buf = NULL;
			} else {
				buf = (void *) hsi_mem_fb_block[i];
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
				/* After hsi_mem_reinit() is added, memory allocation and free need to be taken more care of */
				if (hsi_mem_fb_block_backup[i] != NULL) {
					xmd_mem_dbg("memory %p->%p.", hsi_mem_fb_block_backup[i], buf);
				}
				hsi_mem_fb_block_backup[i] = buf;
#endif
#ifdef HSI_MEM_LOW_MEMORY_THRESHOLD
				if ((HSI_MEM_NUM_OF_FB_BLK - i) < HSI_MEM_LOW_MEMORY_THRESHOLD)
					xmd_mem_dbg("WARN: HSI_MEM_FB_BLK used/total=%d/%d.", i + 1, HSI_MEM_NUM_OF_FB_BLK);
#endif
			}
			goto quit_mem_alloc;
		}
	}
quit_mem_alloc:
#else
	spin_lock_bh(&hsi_mem_lock);
	buf = kmalloc(size, GFP_DMA | GFP_ATOMIC);
#endif
	if(buf == NULL)
		xmd_mem_dbg("Failed to alloc mem returning NULL. Mem exhausted.");
	spin_unlock_bh(&hsi_mem_lock);
	return buf;
}

/*****************************************************************************/
/* Function:... hsi_mem_free                                                 */
/* Description: Frees a memory block, which was allocated before with        */
/*              hsi_mem_alloc.                                               */
/*****************************************************************************/
void hsi_mem_free(void* buf)
{
#if defined (HSI_PRE_ALLOC_BUFFERS)
	u32 i;
	u32 *mem = (u32 *) buf;
	u8 *fb_mem = (u8 *) buf;

	spin_lock_bh(&hsi_mem_lock);

	if(mem == NULL) {
		goto quit_mem_free;
	}

	for(i = 0; i < HSI_MEM_NUM_OF_BLK0; i++) {
		if (mem == hsi_mem_blk0_ptr[i]) {
			hsi_mem_blk0_ptr[i] = NULL;
			goto quit_mem_free;
		}
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
		else if (buf == hsi_mem_blk0[i]) {
			xmd_mem_dbg("WARN: memory %p in HSI_MEM_BLK0 was freed already.", buf);
			goto quit_mem_free;
		}
#endif
	}

	for(i = 0; i < HSI_MEM_NUM_OF_BLK1; i++) {
		if (mem == hsi_mem_blk1_ptr[i]) {
			hsi_mem_blk1_ptr[i] = NULL;
			goto quit_mem_free;
		}
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
		/* After hsi_mem_reinit() is added, memory allocation and free need to be taken more care of */
		else if (buf == hsi_mem_blk1[i]) {
			xmd_mem_dbg("WARN: memory %p in HSI_MEM_BLK1 was freed already.", buf);
			goto quit_mem_free;
		}
#endif
	}

	for(i = 0; i < HSI_MEM_NUM_OF_BLK2; i++) {
		if (mem == hsi_mem_blk2_ptr[i]) {
			hsi_mem_blk2_ptr[i] = NULL;
			goto quit_mem_free;
		}
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
		/* After hsi_mem_reinit() is added, memory allocation and free need to be taken more care of */
		else if (buf == hsi_mem_blk2[i]) {
			xmd_mem_dbg("WARN: memory %p in HSI_MEM_BLK2 was freed already.", buf);
			goto quit_mem_free;
		}
#endif
	}

	for(i = 0; i < HSI_MEM_NUM_OF_BLK3; i++) {
		if (mem == hsi_mem_blk3_ptr[i]) {
			hsi_mem_blk3_ptr[i] = NULL;
			goto quit_mem_free;
		}
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
		/* After hsi_mem_reinit() is added, memory allocation and free need to be taken more care of */
		else if (buf == hsi_mem_blk3[i]) {
			xmd_mem_dbg("WARN: memory %p in HSI_MEM_BLK3 was freed already.", buf);
			goto quit_mem_free;
		}
#endif
	}

	/* free fallback memory */
	for (i = 0; i < HSI_MEM_NUM_OF_FB_BLK; i++) {
		if (hsi_mem_fb_block[i] == fb_mem) {
			kfree(fb_mem);
			hsi_mem_fb_block[i] = NULL;
			goto quit_mem_free;
		}
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
		/* After hsi_mem_reinit() is added, memory allocation and free need to be taken more care of */
		else if (buf == hsi_mem_fb_block_backup[i]) {
			xmd_mem_dbg("WARN: memory %p in HSI_MEM_FB_BLK was freed already.", buf);
			goto quit_mem_free;
		}
#endif
	}

	/*Reached here ? Then this should be allocated from BUF retry WQ.*/
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
	xmd_mem_dbg("WARN: Freeing memory %p not managed by xmd_hsi_mem", buf);
	if (buf)
#endif
	kfree(buf);

quit_mem_free:
#else
	spin_lock_bh(&hsi_mem_lock);
	if(buf != NULL) {
		kfree(buf);
	}
#endif
	spin_unlock_bh(&hsi_mem_lock);
}
