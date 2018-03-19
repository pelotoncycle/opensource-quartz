/*
 *  linux/fs/ext4/bitmap.c
 *
 * Copyright (C) 1992, 1993, 1994, 1995
 * Remy Card (card@masi.ibp.fr)
 * Laboratoire MASI - Institut Blaise Pascal
 * Universite Pierre et Marie Curie (Paris VI)
 */

#include <linux/buffer_head.h>
#include <linux/jbd2.h>
#include "ext4.h"

#ifdef EXT4FS_DEBUG

static const int nibblemap[] = {4, 3, 3, 2, 3, 2, 2, 1, 3, 2, 2, 1, 2, 1, 1, 0};

/** https://android.googlesource.com/kernel/common/+/6ff2c41b81bd0778aa44ffcfce0ea623fa660887
 * ext4: pass a char * to ext4_count_free() instead of a buffer_head ptr
 */
#if 0
unsigned int ext4_count_free(struct buffer_head *map, unsigned int numchars)
{
	unsigned int i, sum = 0;

	if (!map)
		return 0;
	for (i = 0; i < numchars; i++)
		sum += nibblemap[map->b_data[i] & 0xf] +
			nibblemap[(map->b_data[i] >> 4) & 0xf];
	return sum;
}
#else
unsigned int ext4_count_free(char *bitmap, unsigned int numchars)
{
	unsigned int i, sum = 0;

	for (i = 0; i < numchars; i++)
		sum += nibblemap[bitmap[i] & 0xf] +
			nibblemap[(bitmap[i] >> 4) & 0xf];
	return sum;
}
#endif

#endif  /*  EXT4FS_DEBUG  */

