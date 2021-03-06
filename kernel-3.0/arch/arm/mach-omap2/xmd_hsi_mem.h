/*
 * xmd_hsi_mem.h
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

#if !defined(XMD_HSI_MEM__H)
#define XMD_HSI_MEM__H

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/

/*****************************************************************************/
/* DEFINES                                                                   */
/*****************************************************************************/
/*  If this is defined, preallocated buffers are used else memory is allocated
	only when required by kmalloc.*/
#if !defined (CONFIG_EN_HSI_EDLP)
#define HSI_PRE_ALLOC_BUFFERS

/*  If this is defined, preallocated mem is obtained from dma_alloc_coherent
	else static pools are used. */
#define HSI_MEM_USE_DMA_BUFS
#endif

#if 0
/* #define HSI_MEM_ENABLE_LOGS */
#else
#define HSI_MEM_ENABLE_LOGS
#endif

/*TODO: Fine tune below memory configuration as per requirement. */

#define HSI_MEM_BLOCK0_SIZE   512
#define HSI_MEM_BLOCK1_SIZE	 (1024*2)
#define HSI_MEM_BLOCK2_SIZE	 (1024*8)
#if 0
#define HSI_MEM_BLOCK3_SIZE	 (1024*14)
#else
#define HSI_MEM_BLOCK3_SIZE	 (1024*15)
#endif

#define HSI_MEM_LARGE_BLOCK_SIZE  HSI_MEM_BLOCK3_SIZE

#if 0
#define HSI_MEM_NUM_OF_BLK0		60
#define HSI_MEM_NUM_OF_BLK1	 	20
#define HSI_MEM_NUM_OF_BLK2	 	20
#define HSI_MEM_NUM_OF_BLK3 	60
#define HSI_MEM_NUM_OF_FB_BLK	120
#else
#define HSI_MEM_LOW_MEMORY_THRESHOLD	10
#define HSI_MEM_NUM_OF_BLK0		160
#define HSI_MEM_NUM_OF_BLK1	 	80
#define HSI_MEM_NUM_OF_BLK2	 	20
#define HSI_MEM_NUM_OF_BLK3 	60
#define HSI_MEM_NUM_OF_FB_BLK	120
#endif

/*****************************************************************************/
/* TYPE DEFINITIONS                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* PROTOTYPES                                                                */
/*****************************************************************************/
int hsi_mem_init(void);
int hsi_mem_uninit(void);
void* hsi_mem_alloc(int size);
void hsi_mem_free(void* ptr);
#ifdef XMD_DLP_RECOVERY_ENHANCEMENT
int hsi_mem_reinit(void);
#endif
#endif /* XMD_HSI_MEM__H */
