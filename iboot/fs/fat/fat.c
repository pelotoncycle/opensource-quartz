/*
 * fat.c
 *
 * R/O (V)FAT 12/16/32 filesystem implementation by Marcus Sundberg
 *
 * 2002-07-28 - rjones@nexus-tech.net - ported to ppcboot v1.1.6
 * 2003-03-10 - kharris@nexus-tech.net - ported to uboot
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*#define DEBUG*/

#include <common.h>
#include <config.h>
#include <exports.h>
#include <fat.h>
#include <asm/byteorder.h>
#include <part.h>
#include <malloc.h>
#include <linux/compiler.h>

/*
 * Convert a string to lowercase.
 */
static void downcase (char *str)
{
	while (*str != '\0') {
		TOLOWER(*str);
		str++;
	}
}

static block_dev_desc_t *cur_dev;
static unsigned int cur_part_nr;
static disk_partition_t cur_part_info;

#define DOS_BOOT_MAGIC_OFFSET	0x1fe
#define DOS_FS_TYPE_OFFSET	0x36
#define DOS_FS32_TYPE_OFFSET	0x52

static int disk_read(__u32 block, __u32 nr_blocks, void *buf)
{
	if (!cur_dev || !cur_dev->block_read)
		return -1;

	return cur_dev->block_read(cur_dev->dev,
			cur_part_info.start + block, nr_blocks, buf);
}

int fat_register_device (block_dev_desc_t * dev_desc, int part_no)
{
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, buffer, dev_desc->blksz);

	/* First close any currently found FAT filesystem */
	cur_dev = NULL;

#if (defined(CONFIG_CMD_IDE) || \
     defined(CONFIG_CMD_SATA) || \
     defined(CONFIG_CMD_SCSI) || \
     defined(CONFIG_CMD_USB) || \
     defined(CONFIG_MMC) || \
     defined(CONFIG_SYSTEMACE) )

	/* Read the partition table, if present */
	if (!get_partition_info(dev_desc, part_no, &cur_part_info)) {
		cur_dev = dev_desc;
		cur_part_nr = part_no;
	}
#endif

	/* Otherwise it might be a superfloppy (whole-disk FAT filesystem) */
	if (!cur_dev) {
		if (part_no != 1) {
#if 0
			printf("** Partition %d not valid on device %d **\n",
					part_no, dev_desc->dev);
#else
			printf("** bad part %d on dev %d **\n",
					part_no, dev_desc->dev);
#endif
			return -1;
		}

		cur_dev = dev_desc;
		cur_part_nr = 1;
		cur_part_info.start = 0;
		cur_part_info.size = dev_desc->lba;
		cur_part_info.blksz = dev_desc->blksz;
		memset(cur_part_info.name, 0, sizeof(cur_part_info.name));
		memset(cur_part_info.type, 0, sizeof(cur_part_info.type));
	}

	/* Make sure it has a valid FAT header */
	if (disk_read(0, 1, buffer) != 1) {
		cur_dev = NULL;
		return -1;
	}

	/* Check if it's actually a DOS volume */
	if (memcmp(buffer + DOS_BOOT_MAGIC_OFFSET, "\x55\xAA", 2)) {
		cur_dev = NULL;
		return -1;
	}

	/* Check for FAT12/FAT16/FAT32 filesystem */
	if (!memcmp(buffer + DOS_FS_TYPE_OFFSET, "FAT", 3))
		return 0;
	if (!memcmp(buffer + DOS_FS32_TYPE_OFFSET, "FAT32", 5))
		return 0;

	cur_dev = NULL;
	return -1;
}


#ifndef CONFIG_SYS_MINI_FAT
/*
 * Get the first occurence of a directory delimiter ('/' or '\') in a string.
 * Return index into string if found, -1 otherwise.
 */
static int dirdelim (char *str)
{
	char *start = str;

	while (*str != '\0') {
		if (ISDIRDELIM(*str))
			return str - start;
		str++;
	}
	return -1;
}
#endif /* !CONFIG_SYS_MINI_FAT */

/*
 * Extract zero terminated short name from a directory entry.
 */
static void get_name (dir_entry *dirent, char *s_name)
{
	char *ptr;

	memcpy(s_name, dirent->name, 8);
	s_name[8] = '\0';
	ptr = s_name;
	while (*ptr && *ptr != ' ')
		ptr++;
	if (dirent->ext[0] && dirent->ext[0] != ' ') {
		*ptr = '.';
		ptr++;
		memcpy(ptr, dirent->ext, 3);
		ptr[3] = '\0';
		while (*ptr && *ptr != ' ')
			ptr++;
	}
	*ptr = '\0';
	if (*s_name == DELETED_FLAG)
		*s_name = '\0';
	else if (*s_name == aRING)
		*s_name = DELETED_FLAG;
	downcase(s_name);
}

/*
 * Get the entry at index 'entry' in a FAT (12/16/32) table.
 * On failure 0x00 is returned.
 */
static __u32 get_fatent (fsdata *mydata, __u32 entry)
{
	__u32 bufnum;
	__u32 off16, offset;
	__u32 ret = 0x00;
	__u16 val1, val2;

	switch (mydata->fatsize) {
	case 32:
		bufnum = entry / FAT32BUFSIZE;
		offset = entry - bufnum * FAT32BUFSIZE;
		break;
	case 16:
		bufnum = entry / FAT16BUFSIZE;
		offset = entry - bufnum * FAT16BUFSIZE;
		break;
	case 12:
		bufnum = entry / FAT12BUFSIZE;
		offset = entry - bufnum * FAT12BUFSIZE;
		break;

	default:
		/* Unsupported FAT size */
		return ret;
	}

	debug("FAT%d: entry: 0x%04x = %d, offset: 0x%04x = %d\n",
	       mydata->fatsize, entry, entry, offset, offset);

	/* Read a new block of FAT entries into the cache. */
	if (bufnum != mydata->fatbufnum) {
		__u32 getsize = FATBUFBLOCKS;
		__u8 *bufptr = mydata->fatbuf;
		__u32 fatlength = mydata->fatlength;
		__u32 startblock = bufnum * FATBUFBLOCKS;

		if (getsize > fatlength)
			getsize = fatlength;

		fatlength *= mydata->sect_size;	/* We want it in bytes now */
		startblock += mydata->fat_sect;	/* Offset from start of disk */

		if (disk_read(startblock, getsize, bufptr) < 0) {
			debug("Error reading FAT blocks\n");
			return ret;
		}
		mydata->fatbufnum = bufnum;
	}

	/* Get the actual entry from the table */
	switch (mydata->fatsize) {
	case 32:
		ret = FAT2CPU32(((__u32 *) mydata->fatbuf)[offset]);
		break;
	case 16:
		ret = FAT2CPU16(((__u16 *) mydata->fatbuf)[offset]);
		break;
	case 12:
		off16 = (offset * 3) / 4;

		switch (offset & 0x3) {
		case 0:
			ret = FAT2CPU16(((__u16 *) mydata->fatbuf)[off16]);
			ret &= 0xfff;
			break;
		case 1:
			val1 = FAT2CPU16(((__u16 *)mydata->fatbuf)[off16]);
			val1 &= 0xf000;
			val2 = FAT2CPU16(((__u16 *)mydata->fatbuf)[off16 + 1]);
			val2 &= 0x00ff;
			ret = (val2 << 4) | (val1 >> 12);
			break;
		case 2:
			val1 = FAT2CPU16(((__u16 *)mydata->fatbuf)[off16]);
			val1 &= 0xff00;
			val2 = FAT2CPU16(((__u16 *)mydata->fatbuf)[off16 + 1]);
			val2 &= 0x000f;
			ret = (val2 << 8) | (val1 >> 8);
			break;
		case 3:
			ret = FAT2CPU16(((__u16 *)mydata->fatbuf)[off16]);
			ret = (ret & 0xfff0) >> 4;
			break;
		default:
			break;
		}
		break;
	}
	debug("FAT%d: ret: %08x, offset: %04x\n",
	       mydata->fatsize, ret, offset);

	return ret;
}

/*
 * Read at most 'size' bytes from the specified cluster into 'buffer'.
 * Return 0 on success, -1 otherwise.
 */
static int
get_cluster (fsdata *mydata, __u32 clustnum, __u8 *buffer,
	     unsigned long size)
{
	__u32 idx = 0;
	__u32 startsect;
	__u32 nr_sect;
	int ret;

	if (clustnum > 0) {
		startsect = mydata->data_begin +
				clustnum * mydata->clust_size;
	} else {
		startsect = mydata->rootdir_sect;
	}

	debug("gc - clustnum: %u, startsect: %u\n", clustnum, startsect);

	nr_sect = size / mydata->sect_size;
#if 0
	ret = disk_read(startsect, nr_sect, buffer);
	if (ret != nr_sect) {
		debug("Error reading data (got %d)\n", ret);
		return -1;
	}
#else
	if (nr_sect > 0) {
		debug("[cluster] number of sectors: %u (%u bytes)\n", nr_sect, (uint32_t)(nr_sect * mydata->sect_size));
		ret = disk_read(startsect, nr_sect, buffer);
		if (ret != nr_sect) {
			printf("ERR: read data 1 (got %d)\n", ret); /* James Wu */
			return -1;
		}
	}
#endif
	if (size % mydata->sect_size) {
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
		__u8 *tmpbuf = (__u8*)CONFIG_SPL_FAT_TMP_BUFFER_ADDR;
#else
		ALLOC_CACHE_ALIGN_BUFFER(__u8, tmpbuf, mydata->sect_size);
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */

		idx = size / mydata->sect_size;
		debug("[cluster] start sector: %u, padding: %u bytes\n", startsect + idx, (uint32_t)(size % mydata->sect_size));
		ret = disk_read(startsect + idx, 1, tmpbuf);
		if (ret != 1) {
			printf("ERR: read data 2 (got %d)\n", ret); /* James Wu */
			return -1;
		}
		buffer += idx * mydata->sect_size;

		memcpy(buffer, tmpbuf, size % mydata->sect_size);
		return 0;
	}

	return 0;
}

/*
 * Read at most 'maxsize' bytes from the file associated with 'dentptr'
 * into 'buffer'.
 * Return the number of bytes read or -1 on fatal errors.
 */
static long
get_contents (fsdata *mydata, dir_entry *dentptr, __u8 *buffer,
	      unsigned long maxsize)
{
#if 0
	unsigned long filesize = FAT2CPU32(dentptr->size), gotsize = 0;
	unsigned int bytesperclust = mydata->clust_size * mydata->sect_size;
	__u32 curclust = START(dentptr);
	__u32 endclust, newclust;
	unsigned long actsize;

	debug("Filesize: %ld bytes\n", filesize);

	if (maxsize > 0 && filesize > maxsize)
		filesize = maxsize;

	debug("%ld bytes\n", filesize);

	actsize = bytesperclust;
	endclust = curclust;

	do {
		/* search for consecutive clusters */
		while (actsize < filesize) {
			newclust = get_fatent(mydata, endclust);
			if ((newclust - 1) != endclust)
				goto getit;
			if (CHECK_CLUST(newclust, mydata->fatsize)) {
				debug("curclust: 0x%x\n", newclust);
				debug("Invalid FAT entry\n");
				return gotsize;
			}
			endclust = newclust;
			actsize += bytesperclust;
		}

		/* actsize >= file size */
		actsize -= bytesperclust;

		/* get remaining clusters */
		if (get_cluster(mydata, curclust, buffer, (int)actsize) != 0) {
			printf("Error reading cluster\n");
			return -1;
		}

		/* get remaining bytes */
		gotsize += (int)actsize;
		filesize -= actsize;
		buffer += actsize;
		actsize = filesize;
		if (get_cluster(mydata, endclust, buffer, (int)actsize) != 0) {
			printf("Error reading cluster\n");
			return -1;
		}
		gotsize += actsize;
		return gotsize;
getit:
		if (get_cluster(mydata, curclust, buffer, (int)actsize) != 0) {
			printf("Error reading cluster\n");
			return -1;
		}
		gotsize += (int)actsize;
		filesize -= actsize;
		buffer += actsize;

		curclust = get_fatent(mydata, endclust);
		if (CHECK_CLUST(curclust, mydata->fatsize)) {
			debug("curclust: 0x%x\n", curclust);
			printf("Invalid FAT entry\n");
			return gotsize;
		}
		actsize = bytesperclust;
		endclust = curclust;
	} while (1);
#else
	unsigned long filesize = FAT2CPU32(dentptr->size), gotsize = 0;
	unsigned int bytesperclust = mydata->clust_size * mydata->sect_size;
	__u32 curclust = START(dentptr);
	__u32 endclust, newclust;
	unsigned long actsize;

	debug("%s: Filesize: %lu bytes\n", __func__, filesize);
	debug("mydata: 0x%p, dentptr: 0x%p, curclust %u\n", mydata, dentptr, curclust);

	if (maxsize > 0 && filesize > maxsize)
		filesize = maxsize;

	debug("Reading %ld bytes\n", filesize);

	do {
		endclust = curclust;
		if ((filesize - gotsize) > bytesperclust)
			actsize = bytesperclust;
		else
			actsize = filesize - gotsize;

		do {
			/* Over the limitation */
			if ((gotsize + actsize) >= filesize) {
				actsize = filesize - gotsize;
				break;
			}

			newclust = get_fatent(mydata, endclust);

			/* EOF, Error or Not consecutive */
			if ((newclust - 1) != endclust)
				break;
			
			if (CHECK_CLUST(newclust, mydata->fatsize)) {
				printf("bad FAT entry (0x%x)(0x%x)\n", curclust, endclust);
				break;
			}

			endclust = newclust;
			actsize += bytesperclust;
		} while (1);

		debug("get clusters: 0x%x, %lu\n", curclust, actsize);
		if (get_cluster(mydata, curclust, buffer, (int)actsize) != 0) {
			printf("ERR: read cluster (0x%x)\n", curclust);
			return -1;
		}

		/* get remaining bytes */
		gotsize += (int)actsize;
		buffer += actsize;
		curclust = newclust;		
	} while ((gotsize < filesize) && (!CHECK_CLUST(curclust, mydata->fatsize)));

	return gotsize;
#endif
}

#ifdef CONFIG_SUPPORT_VFAT
/*
 * Extract the file name information from 'slotptr' into 'l_name',
 * starting at l_name[*idx].
 * Return 1 if terminator (zero byte) is found, 0 otherwise.
 */
static int slot2str (dir_slot *slotptr, char *l_name, int *idx)
{
	int j;

	for (j = 0; j <= 8; j += 2) {
		l_name[*idx] = slotptr->name0_4[j];
		if (l_name[*idx] == 0x00)
			return 1;
		(*idx)++;
	}
	for (j = 0; j <= 10; j += 2) {
		l_name[*idx] = slotptr->name5_10[j];
		if (l_name[*idx] == 0x00)
			return 1;
		(*idx)++;
	}
	for (j = 0; j <= 2; j += 2) {
		l_name[*idx] = slotptr->name11_12[j];
		if (l_name[*idx] == 0x00)
			return 1;
		(*idx)++;
	}

	return 0;
}

/*
 * Extract the full long filename starting at 'retdent' (which is really
 * a slot) into 'l_name'. If successful also copy the real directory entry
 * into 'retdent'
 * Return 0 on success, -1 otherwise.
 */
__u8 get_vfatname_block[MAX_CLUSTSIZE]
	__aligned(ARCH_DMA_MINALIGN);

static int
get_vfatname (fsdata *mydata, int curclust, __u8 *cluster,
	      dir_entry *retdent, char *l_name)
{
	dir_entry *realdent;
	dir_slot *slotptr = (dir_slot *)retdent;
	__u8 *buflimit = cluster + mydata->sect_size * ((curclust == 0) ?
							PREFETCH_BLOCKS :
							mydata->clust_size);
	__u8 counter = (slotptr->id & ~LAST_LONG_ENTRY_MASK) & 0xff;
	int idx = 0;

	if (counter > VFAT_MAXSEQ) {
		debug("Error: VFAT name is too long\n");
		return -1;
	}

	while ((__u8 *)slotptr < buflimit) {
		if (counter == 0)
			break;
		if (((slotptr->id & ~LAST_LONG_ENTRY_MASK) & 0xff) != counter)
			return -1;
		slotptr++;
		counter--;
	}

	if ((__u8 *)slotptr >= buflimit) {
		dir_slot *slotptr2;

		if (curclust == 0)
			return -1;
		curclust = get_fatent(mydata, curclust);
		if (CHECK_CLUST(curclust, mydata->fatsize)) {
			debug("curclust: 0x%x\n", curclust);
			printf("Invalid FAT entry\n");
			return -1;
		}

		if (get_cluster(mydata, curclust, get_vfatname_block,
				mydata->clust_size * mydata->sect_size) != 0) {
			debug("Error: reading directory block\n");
			return -1;
		}

		slotptr2 = (dir_slot *)get_vfatname_block;
		while (counter > 0) {
			if (((slotptr2->id & ~LAST_LONG_ENTRY_MASK)
			    & 0xff) != counter)
				return -1;
			slotptr2++;
			counter--;
		}

		/* Save the real directory entry */
		realdent = (dir_entry *)slotptr2;
		while ((__u8 *)slotptr2 > get_vfatname_block) {
			slotptr2--;
			slot2str(slotptr2, l_name, &idx);
		}
	} else {
		/* Save the real directory entry */
		realdent = (dir_entry *)slotptr;
	}

	do {
		slotptr--;
		if (slot2str(slotptr, l_name, &idx))
			break;
	} while (!(slotptr->id & LAST_LONG_ENTRY_MASK));

	l_name[idx] = '\0';
	if (*l_name == DELETED_FLAG)
		*l_name = '\0';
	else if (*l_name == aRING)
		*l_name = DELETED_FLAG;
	downcase(l_name);

	/* Return the real directory entry */
	memcpy(retdent, realdent, sizeof(dir_entry));

	return 0;
}

/* Calculate short name checksum */
static __u8 mkcksum (const char *str)
{
	int i;

	__u8 ret = 0;

	for (i = 0; i < 11; i++) {
		ret = (((ret & 1) << 7) | ((ret & 0xfe) >> 1)) + str[i];
	}

	return ret;
}
#endif	/* CONFIG_SUPPORT_VFAT */

#ifndef CONFIG_SYS_MINI_FAT
/*
 * Get the directory entry associated with 'filename' from the directory
 * starting at 'startsect'
 */
__u8 get_dentfromdir_block[MAX_CLUSTSIZE]
	__aligned(ARCH_DMA_MINALIGN);

static dir_entry *get_dentfromdir (fsdata *mydata, int startsect,
				   char *filename, dir_entry *retdent,
				   int dols)
{
#ifdef CONFIG_SUPPORT_VFAT
	__u16 prevcksum = 0xffff;
#endif /* CONFIG_SUPPORT_VFAT */
	__u32 curclust = START(retdent);
	int files = 0, dirs = 0;

	debug("get_dentfromdir: %s\n", filename);

	while (1) {
		dir_entry *dentptr;

		int i;

		if (get_cluster(mydata, curclust, get_dentfromdir_block,
				mydata->clust_size * mydata->sect_size) != 0) {
			debug("Error: reading directory block\n");
			return NULL;
		}

		dentptr = (dir_entry *)get_dentfromdir_block;

		for (i = 0; i < DIRENTSPERCLUST; i++) {
#ifdef CONFIG_SUPPORT_VFAT
			char s_name[14], l_name[VFAT_MAXLEN_BYTES];

		    s_name[0] = '\0';
			l_name[0] = '\0';
#else
			char s_name[14];

		    s_name[0] = '\0';
#endif /* CONFIG_SUPPORT_VFAT */
#if 0
			if (dentptr->name[0] == DELETED_FLAG) {
				dentptr++;
				continue;
			}
#else
			switch (dentptr->name[0]) {
			case DELETED_FLAG:	/* 0xE5 DELETE */
			case aRING:			/* 0x05 KANJI */
				dentptr++;
				continue;
			case 0x2E:			/* DOT entry */
				if (dols == LS_NO) {
					dentptr++;
					continue;
				}
				break;
			default:
				break;
			}
#endif
			if ((dentptr->attr & ATTR_VOLUME)) {
#ifdef CONFIG_SUPPORT_VFAT
				if ((dentptr->attr & ATTR_VFAT) == ATTR_VFAT &&
				    (dentptr->name[0] & LAST_LONG_ENTRY_MASK)) {
					prevcksum = ((dir_slot *)dentptr)->alias_checksum;
					get_vfatname(mydata, curclust,
						     get_dentfromdir_block,
						     dentptr, l_name);
					if (dols) {
						int isdir;
						char dirc;
						int doit = 0;

						isdir = (dentptr->attr & ATTR_DIR);

						if (isdir) {
							dirs++;
							dirc = '/';
							doit = 1;
						} else {
							dirc = ' ';
							if (l_name[0] != 0) {
								files++;
								doit = 1;
							}
						}
						if (doit) {
							if (dirc == ' ') {
								printf(" %8ld   %s%c\n",
									(long)FAT2CPU32(dentptr->size),
									l_name,
									dirc);
							} else {
								printf("            %s%c\n",
									l_name,
									dirc);
							}
						}
						dentptr++;
						continue;
					}
					debug("vfatname: |%s|\n", l_name);
				} else
#endif
				{
					/* Volume label or VFAT entry */
					dentptr++;
					continue;
				}
			}
			if (dentptr->name[0] == 0) {
				if (dols) {
					printf("\n%d file(s), %d dir(s)\n\n",
						files, dirs);
				}
				debug("Dentname == NULL - %d\n", i);
				return NULL;
			}
#ifdef CONFIG_SUPPORT_VFAT
			if (dols && mkcksum(dentptr->name) == prevcksum) {
				prevcksum = 0xffff;
				dentptr++;
				continue;
			}
#endif
			get_name(dentptr, s_name);
			if (dols) {
				int isdir = (dentptr->attr & ATTR_DIR);
				char dirc;
				int doit = 0;

				if (isdir) {
					dirs++;
					dirc = '/';
					doit = 1;
				} else {
					dirc = ' ';
					if (s_name[0] != 0) {
						files++;
						doit = 1;
					}
				}

				if (doit) {
					if (dirc == ' ') {
						printf(" %8ld   %s%c\n",
							(long)FAT2CPU32(dentptr->size),
							s_name, dirc);
					} else {
						printf("            %s%c\n",
							s_name, dirc);
					}
				}

				dentptr++;
				continue;
			}

#ifdef CONFIG_SUPPORT_VFAT
			if (strcmp(filename, s_name)
			    && strcmp(filename, l_name)) {
				debug("Mismatch: |%s|%s|\n", s_name, l_name);
#else
			if (strcmp(filename, s_name)) {
				debug("Mismatch: |%s|\n", s_name);
#endif /* CONFIG_SUPPORT_VFAT */
				dentptr++;
				continue;
			}

			memcpy(retdent, dentptr, sizeof(dir_entry));

			debug("DentName: %s", s_name);
			debug(", start: 0x%x", START(dentptr));
			debug(", size:  0x%x %s\n",
			      FAT2CPU32(dentptr->size),
			      (dentptr->attr & ATTR_DIR) ? "(DIR)" : "");

			return retdent;
		}

		curclust = get_fatent(mydata, curclust);
		if (CHECK_CLUST(curclust, mydata->fatsize)) {
			debug("curclust: 0x%x\n", curclust);
			printf("Invalid FAT entry\n");
			return NULL;
		}
	}

	return NULL;
}
#endif /* !CONFIG_SYS_MINI_FAT */

/*
 * Read boot sector and volume info from a FAT filesystem
 */
static int
read_bootsectandvi (boot_sector *bs, volume_info *volinfo, int *fatsize)
{
	__u8 *block;
	volume_info *vistart;
	int ret = 0;

	if (cur_dev == NULL) {
		debug("Error: no device selected\n");
		return -1;
	}

#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
	block = (__u8*)CONFIG_SPL_FAT_BLOCK_ADDR;
#else
	block = memalign(ARCH_DMA_MINALIGN, cur_dev->blksz);
	if (block == NULL) {
		debug("Error: allocating block\n");
		return -1;
	}
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */

	if (disk_read (0, 1, block) < 0) {
		debug("Error: reading block\n");
		goto fail;
	}

	memcpy(bs, block, sizeof(boot_sector));
	bs->reserved = FAT2CPU16(bs->reserved);
	bs->fat_length = FAT2CPU16(bs->fat_length);
	bs->secs_track = FAT2CPU16(bs->secs_track);
	bs->heads = FAT2CPU16(bs->heads);
	bs->total_sect = FAT2CPU32(bs->total_sect);

	debug("nr of sectors per FAT for FAT12/FAT16 (fat length): %d\n", bs->fat_length);
	debug("nr of sectors per cluster: %d\n", bs->cluster_size);
	debug("nr of reserved sectors: %d\n", bs->reserved);
	debug("nr of FAT: %d\n", bs->fats);
	debug("nr of sectors per track: %d\n", bs->secs_track);
	debug("nr of heads: %d\n", bs->heads);
	debug("nr of hidden sectors: %d\n",  FAT2CPU32(bs->hidden));
	debug("total sectors: %d\n", bs->total_sect);

	/* FAT32 entries */
	if (bs->fat_length == 0) {
		/* Assume FAT32 */
		bs->fat32_length = FAT2CPU32(bs->fat32_length);
		bs->flags = FAT2CPU16(bs->flags);
		bs->root_cluster = FAT2CPU32(bs->root_cluster);
		bs->info_sector = FAT2CPU16(bs->info_sector);
		bs->backup_boot = FAT2CPU16(bs->backup_boot);
		debug("fat32_length: %u, flags: 0x%08X\n",
				bs->fat32_length, bs->flags);
		debug("Active FAT: 0x%04X, FAT Mirroring: %d\n",
				(bs->flags & 0x1F), (bs->flags & 0x080) >> 7);
		debug("root_cluster: %u, info_sector: %d, backup_boot: %d\n",
				bs->root_cluster, bs->info_sector, bs->backup_boot);
		vistart = (volume_info *)(block + sizeof(boot_sector));
		*fatsize = 32;
	} else {
		vistart = (volume_info *)&(bs->fat32_length);
		*fatsize = 0;
	}
	memcpy(volinfo, vistart, sizeof(volume_info));

	if (*fatsize == 32) {
		if (strncmp(FAT32_SIGN, vistart->fs_type, SIGNLEN) == 0)
			goto exit;
	} else {
		if (strncmp(FAT12_SIGN, vistart->fs_type, SIGNLEN) == 0) {
			*fatsize = 12;
			goto exit;
		}
		if (strncmp(FAT16_SIGN, vistart->fs_type, SIGNLEN) == 0) {
			*fatsize = 16;
			goto exit;
		}
	}

	debug("Error: broken fs_type sign\n");
fail:
	ret = -1;
exit:
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
#else
	free(block);
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */
	return ret;
}

__u8 do_fat_read_block[MAX_CLUSTSIZE]
	__aligned(ARCH_DMA_MINALIGN);

static fsdata datablock __attribute__ ((section(".data")));

#ifndef CONFIG_SYS_MINI_FAT
static long /* James Wu */
do_fat_read (const char *filename, void *buffer, unsigned long maxsize,
	     int dols)
{
#ifdef CONFIG_SUPPORT_VFAT
	char fnamecopy[2048];
#else
	char fnamecopy[256];
#endif /* CONFIG_SUPPORT_VFAT */
	boot_sector bs;
	volume_info volinfo;
#if 0
	fsdata datablock;
#endif
	fsdata *mydata = &datablock;
	dir_entry *dentptr;
#ifdef CONFIG_SUPPORT_VFAT
	__u16 prevcksum = 0xffff;
#endif /* CONFIG_SUPPORT_VFAT */
	char *subname = "";
	__u32 cursect;
	int idx, isdir = 0;
	int files = 0, dirs = 0;
	long ret = -1;
	int firsttime;
	__u32 root_cluster = 0;
	int rootdir_size = 0;
	int j;

	memset(&datablock, 0, sizeof(datablock));

	if (read_bootsectandvi(&bs, &volinfo, &mydata->fatsize)) {
		debug("Error: reading boot sector\n");
		return -1;
	}

	if (mydata->fatsize == 32) {
		root_cluster = bs.root_cluster;
		mydata->fatlength = bs.fat32_length;
	} else {
		mydata->fatlength = bs.fat_length;
	}

	mydata->fat_sect = bs.reserved;

#if 0
	cursect = mydata->rootdir_sect
		= mydata->fat_sect + mydata->fatlength * bs.fats;
#endif

	mydata->sect_size = (bs.sector_size[1] << 8) + bs.sector_size[0];
	mydata->clust_size = bs.cluster_size;
	if (mydata->sect_size != cur_part_info.blksz) {
#if 0
		printf("Error: FAT sector size mismatch (fs=%hu, dev=%lu)\n",
				mydata->sect_size, cur_part_info.blksz);
#else
		printf("ERR: bad FAT sector size(fs=%hu, dev=%lu)\n",
				mydata->sect_size, cur_part_info.blksz);
#endif
		return -1;
	}

	if (mydata->fatsize == 32) {
#if 0
		mydata->data_begin = mydata->rootdir_sect -
					(mydata->clust_size * 2);
#else
		int fat_end = mydata->fat_sect + mydata->fatlength * bs.fats;

		mydata->data_begin = fat_end - (mydata->clust_size * 2);

		cursect = mydata->rootdir_sect
			= fat_end + (bs.root_cluster - 2) * mydata->clust_size;
#endif
	} else {
		cursect = mydata->rootdir_sect
			= mydata->fat_sect + mydata->fatlength * bs.fats;
		rootdir_size = ((bs.dir_entries[1]  * (int)256 +
				 bs.dir_entries[0]) *
				 sizeof(dir_entry)) /
				 mydata->sect_size;
		mydata->data_begin = mydata->rootdir_sect +
					rootdir_size -
					(mydata->clust_size * 2);
	}

	mydata->fatbufnum = -1;
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
	mydata->fatbuf = (__u8*)CONFIG_SPL_FAT_BUFFER_ADDR;
#else
	mydata->fatbuf = memalign(ARCH_DMA_MINALIGN, FATBUFSIZE);
	if (mydata->fatbuf == NULL) {
		debug("Error: allocating memory\n");
		return -1;
	}
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */

#ifdef CONFIG_SUPPORT_VFAT
	debug("VFAT Support enabled\n");
#endif
	debug("FAT%d, fat_sect: %u, fatlength: %u\n",
	       mydata->fatsize, mydata->fat_sect, mydata->fatlength);
	debug("Rootdir begins at cluster: %u, sector: %u, offset: %x\n"
	       "Data begins at: %d\n",
	       root_cluster,
	       mydata->rootdir_sect,
	       mydata->rootdir_sect * mydata->sect_size, mydata->data_begin);
	debug("Sector size: %d, cluster size: %d\n", mydata->sect_size,
	      mydata->clust_size);

	/* "cwd" is always the root... */
	while (ISDIRDELIM(*filename))
		filename++;

	/* Make a copy of the filename and convert it to lowercase */
	strcpy(fnamecopy, filename);
	downcase(fnamecopy);

	if (*fnamecopy == '\0') {
		if (!dols)
			goto exit;

		dols = LS_ROOT;
	} else if ((idx = dirdelim(fnamecopy)) >= 0) {
		isdir = 1;
		fnamecopy[idx] = '\0';
		subname = fnamecopy + idx + 1;

		/* Handle multiple delimiters */
		while (ISDIRDELIM(*subname))
			subname++;
	} else if (dols) {
		isdir = 1;
	}

	j = 0;
	while (1) {
		int i;

		debug("FAT read sect=%d, clust_size=%d, DIRENTSPERBLOCK=%zd\n",
			cursect, mydata->clust_size, DIRENTSPERBLOCK);

		if (disk_read(cursect,
				(mydata->fatsize == 32) ?
				(mydata->clust_size) :
				PREFETCH_BLOCKS,
				do_fat_read_block) < 0) {
			debug("Error: reading rootdir block\n");
			goto exit;
		}

		dentptr = (dir_entry *) do_fat_read_block;

		for (i = 0; i < DIRENTSPERBLOCK; i++) {
#ifdef CONFIG_SUPPORT_VFAT
			char s_name[14], l_name[VFAT_MAXLEN_BYTES];

		    s_name[0] = '\0';
			l_name[0] = '\0';
#else
			char s_name[14];

		    s_name[0] = '\0';
#endif /* CONFIG_SUPPORT_VFAT */
#if 0
			if (dentptr->name[0] == DELETED_FLAG) {
				dentptr++;
				continue;
			}
#else
			switch (dentptr->name[0]) {
			case DELETED_FLAG:	/* 0xE5 DELETE */
			case aRING:			/* 0x05 KANJI */
				dentptr++;
				continue;
			case 0x2E:			/* DOT entry */
				if (dols == LS_NO) {
					dentptr++;
					continue;
				}
				break;
			default:
				break;
			}
#endif
			if ((dentptr->attr & ATTR_VOLUME)) {
#ifdef CONFIG_SUPPORT_VFAT
				if ((dentptr->attr & ATTR_VFAT) == ATTR_VFAT &&
				    (dentptr->name[0] & LAST_LONG_ENTRY_MASK)) {
					prevcksum =
						((dir_slot *)dentptr)->alias_checksum;

					get_vfatname(mydata,
						     root_cluster,
						     do_fat_read_block,
						     dentptr, l_name);

					if (dols == LS_ROOT) {
						char dirc;
						int doit = 0;
						int isdir =
							(dentptr->attr & ATTR_DIR);

						if (isdir) {
							dirs++;
							dirc = '/';
							doit = 1;
						} else {
							dirc = ' ';
							if (l_name[0] != 0) {
								files++;
								doit = 1;
							}
						}
						if (doit) {
							if (dirc == ' ') {
								printf(" %8ld   %s%c\n",
									(long)FAT2CPU32(dentptr->size),
									l_name,
									dirc);
							} else {
								printf("            %s%c\n",
									l_name,
									dirc);
							}
						}
						dentptr++;
						continue;
					}
					debug("Rootvfatname: |%s|\n",
					       l_name);
				} else
#endif
				{
					/* Volume label or VFAT entry */
					dentptr++;
					continue;
				}
			} else if (dentptr->name[0] == 0) {
				debug("RootDentname == NULL - %d\n", i);
				if (dols == LS_ROOT) {
					printf("\n%d file(s), %d dir(s)\n\n",
						files, dirs);
					ret = 0;
				}
				goto exit;
			}
#ifdef CONFIG_SUPPORT_VFAT
			else if (dols == LS_ROOT &&
				 mkcksum(dentptr->name) == prevcksum) {
				prevcksum = 0xffff;
				dentptr++;
				continue;
			}
#endif
			get_name(dentptr, s_name);

			if (dols == LS_ROOT) {
				int isdir = (dentptr->attr & ATTR_DIR);
				char dirc;
				int doit = 0;

				if (isdir) {
					dirc = '/';
					if (s_name[0] != 0) {
						dirs++;
						doit = 1;
					}
				} else {
					dirc = ' ';
					if (s_name[0] != 0) {
						files++;
						doit = 1;
					}
				}
				if (doit) {
					if (dirc == ' ') {
						printf(" %8ld   %s%c\n",
							(long)FAT2CPU32(dentptr->size),
							s_name, dirc);
					} else {
						printf("            %s%c\n",
							s_name, dirc);
					}
				}
				dentptr++;
				continue;
			}

#ifdef CONFIG_SUPPORT_VFAT
			if (strcmp(fnamecopy, s_name)
			    && strcmp(fnamecopy, l_name)) {
				debug("RootMismatch: |%s|%s|\n", s_name,
				       l_name);
#else
			if (strcmp(fnamecopy, s_name)) {
				debug("RootMismatch: |%s|\n", s_name);
#endif /* CONFIG_SUPPORT_VFAT */
				dentptr++;
				continue;
			}

			if (isdir && !(dentptr->attr & ATTR_DIR))
				goto exit;

#if 0
			debug("RootName: %s", s_name);
			debug(", start: 0x%x", START(dentptr));
			debug(", size:  0x%x %s\n",
			       FAT2CPU32(dentptr->size),
			       isdir ? "(DIR)" : "");
#else
			debug("RootName: %s, start: 0x%x, size:  0x%x (%u) %s\n",
					s_name, START(dentptr),
					FAT2CPU32(dentptr->size), FAT2CPU32(dentptr->size),
					isdir ? "(DIR)" : "");
#endif

			goto rootdir_done;	/* We got a match */
		}
		debug("END LOOP: j=%d   clust_size=%d\n", j,
		       mydata->clust_size);

		/*
		 * On FAT32 we must fetch the FAT entries for the next
		 * root directory clusters when a cluster has been
		 * completely processed.
		 */
		++j;
		int fat32_end = 0;
		if ((mydata->fatsize == 32) && (j == mydata->clust_size)) {
			int nxtsect = 0;
			int nxt_clust = 0;

			nxt_clust = get_fatent(mydata, root_cluster);
			fat32_end = CHECK_CLUST(nxt_clust, 32);

			nxtsect = mydata->data_begin +
				(nxt_clust * mydata->clust_size);

			root_cluster = nxt_clust;

			cursect = nxtsect;
			j = 0;
		} else {
			cursect++;
		}

		/* If end of rootdir reached */
		if ((mydata->fatsize == 32 && fat32_end) ||
		    (mydata->fatsize != 32 && j == rootdir_size)) {
			if (dols == LS_ROOT) {
				printf("\n%d file(s), %d dir(s)\n\n",
				       files, dirs);
				ret = 0;
			}
			goto exit;
		}
	}
rootdir_done:

	firsttime = 1;

	while (isdir) {
		int startsect = mydata->data_begin
			+ START(dentptr) * mydata->clust_size;
		dir_entry dent;
		char *nextname = NULL;

		dent = *dentptr;
		dentptr = &dent;

		idx = dirdelim(subname);

		if (idx >= 0) {
			subname[idx] = '\0';
			nextname = subname + idx + 1;
			/* Handle multiple delimiters */
			while (ISDIRDELIM(*nextname))
				nextname++;
			if (dols && *nextname == '\0')
				firsttime = 0;
		} else {
			if (dols && firsttime) {
				firsttime = 0;
			} else {
				isdir = 0;
			}
		}

		if (get_dentfromdir(mydata, startsect, subname, dentptr,
				     isdir ? 0 : dols) == NULL) {
			if (dols && !isdir)
				ret = 0;
			goto exit;
		}

		if (idx >= 0) {
			if (!(dentptr->attr & ATTR_DIR))
				goto exit;
			subname = nextname;
		}
	}

	ret = get_contents(mydata, dentptr, buffer, maxsize);
	debug("Size: %d, got: %ld\n", FAT2CPU32(dentptr->size), ret);

exit:
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
#else
	free(mydata->fatbuf);
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */
	return ret;
}
#endif /* !CONFIG_SYS_MINI_FAT */

/**
 * Returns:
 *	0:		Found the file and remember to free "mydata->fatbuf" manually
 *	-1:		File is not found
 */
static int
file_fat_root_find (const char *filename, fsdata **r_mydata, dir_entry **r_dentptr)
{
#ifdef CONFIG_SUPPORT_VFAT
	char fnamecopy[256];
#else
	char fnamecopy[15];
#endif /* CONFIG_SUPPORT_VFAT */
	boot_sector bs;
	volume_info volinfo;
	fsdata *mydata = &datablock;
	dir_entry *dentptr;
	__u32 cursect;
	long ret = -1;
	__u32 root_cluster = 0;
	int rootdir_size = 0;
	int j;

	memset(&datablock, 0, sizeof(datablock));

	if (read_bootsectandvi(&bs, &volinfo, &mydata->fatsize)) {
		debug("Error: reading boot sector\n");
		return -1;
	}

	if (mydata->fatsize == 32) {
		root_cluster = bs.root_cluster;
		mydata->fatlength = bs.fat32_length;
	} else {
		mydata->fatlength = bs.fat_length;
	}

	mydata->fat_sect = bs.reserved;

#if 0
	cursect = mydata->rootdir_sect
		= mydata->fat_sect + mydata->fatlength * bs.fats;
#endif

	mydata->sect_size = (bs.sector_size[1] << 8) + bs.sector_size[0];
	mydata->clust_size = bs.cluster_size;
	if (mydata->sect_size != cur_part_info.blksz) {
#if 0
		printf("Error: FAT sector size mismatch (fs=%hu, dev=%lu)\n",
				mydata->sect_size, cur_part_info.blksz);
#else
		printf("ERR: bad FAT sector size (fs=%hu, dev=%lu)\n",
				mydata->sect_size, cur_part_info.blksz);
#endif
		return -1;
	}

	if (mydata->fatsize == 32) {
#if 0
		mydata->data_begin = mydata->rootdir_sect -
					(mydata->clust_size * 2);
#else
		int fat_end = mydata->fat_sect + mydata->fatlength * bs.fats;

		mydata->data_begin = fat_end - (mydata->clust_size * 2);

		cursect = mydata->rootdir_sect
			= fat_end + (bs.root_cluster - 2) * mydata->clust_size;
#endif
	} else {
		cursect = mydata->rootdir_sect
			= mydata->fat_sect + mydata->fatlength * bs.fats;
		rootdir_size = ((bs.dir_entries[1]  * (int)256 +
				 bs.dir_entries[0]) *
				 sizeof(dir_entry)) /
				 mydata->sect_size;
		mydata->data_begin = mydata->rootdir_sect +
					rootdir_size -
					(mydata->clust_size * 2);
	}

	mydata->fatbufnum = -1;
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
	mydata->fatbuf = (__u8*)CONFIG_SPL_FAT_BUFFER_ADDR;
#else
	mydata->fatbuf = malloc(FATBUFSIZE);
	if (mydata->fatbuf == NULL) {
		debug("Error: allocating memory\n");
		return -1;
	}
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */

#ifdef CONFIG_SUPPORT_VFAT
	debug("VFAT Support enabled\n");
#endif
	debug("FAT%d, fat_sect: %d, fatlength: %d\n",
	       mydata->fatsize, mydata->fat_sect, mydata->fatlength);
	debug("Rootdir begins at cluster: %u, sector: %u, offset: %x\n"
	       "Data begins at: %d\n",
	       root_cluster,
	       mydata->rootdir_sect,
	       mydata->rootdir_sect * mydata->sect_size, mydata->data_begin);
	debug("Sector size: %d, cluster size: %d\n", mydata->sect_size,
	      mydata->clust_size);

	/* "cwd" is always the root... */
	while (ISDIRDELIM(*filename))
		filename++;

	/* Make a copy of the filename and convert it to lowercase */
#if 0
	strcpy(fnamecopy, filename);
#else
#ifdef CONFIG_SUPPORT_VFAT
	strncpy(fnamecopy, filename, 255);
#else
	strncpy(fnamecopy, filename, 14);
#endif /* CONFIG_SUPPORT_VFAT */
#endif
	downcase(fnamecopy);

	if (*fnamecopy == '\0')
		goto exit;

	j = 0;
	while (1) {
		int i;

		debug("FAT read sect=%d, clust_size=%d, DIRENTSPERBLOCK=%zd\n",
			cursect, mydata->clust_size, DIRENTSPERBLOCK);

		if (disk_read(cursect,
				(mydata->fatsize == 32) ?
				(mydata->clust_size) :
				PREFETCH_BLOCKS,
				do_fat_read_block) < 0) {
			debug("Error: reading rootdir block\n");
			goto exit;
		}

		dentptr = (dir_entry *) do_fat_read_block;

		for (i = 0; i < DIRENTSPERBLOCK; i++) {
#ifdef CONFIG_SUPPORT_VFAT
			char s_name[14], l_name[VFAT_MAXLEN_BYTES];

		    s_name[0] = '\0';
			l_name[0] = '\0';
#else
			char s_name[14];

		    s_name[0] = '\0';
#endif /* CONFIG_SUPPORT_VFAT */
#if 0
			if (dentptr->name[0] == DELETED_FLAG) {
				dentptr++;
				continue;
			}
#else
			switch (dentptr->name[0]) {
			case DELETED_FLAG:	/* 0xE5 DELETE */
			case aRING:			/* 0x05 KANJI */
				dentptr++;
				continue;
			case 0x2E:			/* DOT entry */
#ifdef DEBUG
				get_name (dentptr, s_name);
				debug("RootSkip (DOT): |%s|\n", s_name);
#endif /* DEBUG */
				dentptr++;
				continue;
			default:
				break;
			}
#endif
			if ((dentptr->attr & ATTR_VOLUME)) {
#ifdef CONFIG_SUPPORT_VFAT
				if ((dentptr->attr & ATTR_VFAT) == ATTR_VFAT &&
				    (dentptr->name[0] & LAST_LONG_ENTRY_MASK)) {
					get_vfatname(mydata,
						     root_cluster,
						     do_fat_read_block,
						     dentptr, l_name);
				} else
#endif
				{
					/* Volume label or VFAT entry */
					dentptr++;
					continue;
				}
			} else if (dentptr->attr & ATTR_DIR) {
#ifdef DEBUG
				get_name (dentptr, s_name);
				debug("RootSkip (DIR): |%s|\n", s_name);
#endif /* DEBUG */
				/* Directory entry */
				dentptr++;
				continue;
			} else if (dentptr->name[0] == 0) {
				debug("RootDentname == NULL - %d\n", i);
				goto exit;
			}
			get_name(dentptr, s_name);

#ifdef CONFIG_SUPPORT_VFAT
			if (strcmp(fnamecopy, s_name)
			    && strcmp(fnamecopy, l_name)) {
				debug("RootMismatch: |%s|%s|\n", s_name,
				       l_name);
#else
			if (strcmp(fnamecopy, s_name)) {
				debug("RootMismatch: |%s|\n", s_name);
#endif /* CONFIG_SUPPORT_VFAT */
				dentptr++;
				continue;
			}

			debug("RootName: %s, start: 0x%x, size:  0x%x (%u)\n",
					s_name, START(dentptr),
					FAT2CPU32(dentptr->size), FAT2CPU32(dentptr->size));

			/* We got a match */
			*r_mydata = mydata;
			*r_dentptr = dentptr;
			debug("r_mydata: 0x%p, r_dentptr: 0x%p\n", *r_mydata, *r_dentptr);

			ret = 0;
			goto exit;
		}
		debug("END LOOP: j=%d   clust_size=%d\n", j,
		       mydata->clust_size);

		/*
		 * On FAT32 we must fetch the FAT entries for the next
		 * root directory clusters when a cluster has been
		 * completely processed.
		 */
		++j;
		int fat32_end = 0;
		if ((mydata->fatsize == 32) && (j == mydata->clust_size)) {
			int nxtsect = 0;
			int nxt_clust = 0;

			nxt_clust = get_fatent(mydata, root_cluster);
			fat32_end = CHECK_CLUST(nxt_clust, 32);

			nxtsect = mydata->data_begin +
				(nxt_clust * mydata->clust_size);

			root_cluster = nxt_clust;

			cursect = nxtsect;
			j = 0;
		} else {
			cursect++;
		}

		/* If end of rootdir reached */
		if ((mydata->fatsize == 32 && fat32_end) ||
		    (mydata->fatsize != 32 && j == rootdir_size)) {
			goto exit;
		}
	}
exit:
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
#else
	if (ret != 0) {
		free(mydata->fatbuf);
		mydata->fatbuf = NULL;
	}
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */
	return ret;
}

#ifndef CONFIG_SYS_MINI_FAT
int file_fat_detectfs (void)
{
	boot_sector bs;
	volume_info volinfo;
	int fatsize;
	char vol_label[12];

	if (cur_dev == NULL) {
		printf("No current device\n");
		return 1;
	}

#if defined(CONFIG_CMD_IDE) || \
    defined(CONFIG_CMD_SATA) || \
    defined(CONFIG_CMD_SCSI) || \
    defined(CONFIG_CMD_USB) || \
    defined(CONFIG_MMC)
	printf("Interface:  ");
	switch (cur_dev->if_type) {
	case IF_TYPE_IDE:
		printf("IDE");
		break;
	case IF_TYPE_SATA:
		printf("SATA");
		break;
	case IF_TYPE_SCSI:
		printf("SCSI");
		break;
	case IF_TYPE_ATAPI:
		printf("ATAPI");
		break;
	case IF_TYPE_USB:
		printf("USB");
		break;
	case IF_TYPE_DOC:
		printf("DOC");
		break;
	case IF_TYPE_MMC:
		printf("MMC");
		break;
	default:
		printf("Unknown");
	}

	printf("\n  Device %d: ", cur_dev->dev);
	dev_print(cur_dev);
#endif

	if (read_bootsectandvi(&bs, &volinfo, &fatsize)) {
		printf("\nNo valid FAT fs found\n");
		return 1;
	}

	memcpy(vol_label, volinfo.volume_label, 11);
	vol_label[11] = '\0';
	volinfo.fs_type[5] = '\0';

	printf("Partition %d: Filesystem: %s \"%s\"\n", cur_part_nr,
		volinfo.fs_type, vol_label);

	return 0;
}
#endif /* !CONFIG_SYS_MINI_FAT */

#ifndef CONFIG_SYS_MINI_FAT
int file_fat_ls (const char *dir)
{
	return do_fat_read(dir, NULL, 0, LS_YES);
}
#endif /* !CONFIG_SYS_MINI_FAT */

long file_fat_read (const char *filename, void *buffer, unsigned long maxsize)
{
#if 0
	printf("reading %s\n", filename);
	return do_fat_read(filename, buffer, maxsize, LS_NO);
#else
	long ret = -1;
	fsdata *mydata = NULL;
	dir_entry *dentptr = NULL;

	debug("reading %s\n", filename);

	if (cur_dev == NULL) {
		printf("no FAT dev\n");
		return -1;
	}

	if (file_fat_root_find(filename, &mydata, &dentptr)) {
		debug("'%s' not found\n", filename);
	} else if (mydata && dentptr) {
		ret = get_contents(mydata, dentptr, buffer, maxsize);
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
#else
		if (mydata->fatbuf) {
			free(mydata->fatbuf);
			mydata->fatbuf = NULL;
		}
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */
		debug("Size: %d, got: %ld\n", FAT2CPU32 (dentptr->size), ret);
	} else {
		debug("bad FAT\n");
	}

	return ret;
#endif
}

long file_fat_read_init (fat_read_info *fat, const char *filename, unsigned long minsize)
{
	long ret = -1;
	fsdata *mydata = NULL;
	dir_entry *dentptr = NULL;

	debug("reading %s, \n", filename);

	if (fat == NULL || cur_dev == NULL) {
		printf("no FAT dev\n");
		return -1;
	}

	if (file_fat_root_find(filename, &mydata, &dentptr)) {
		debug("'%s' not found\n", filename);
		ret = -1;
	} else if (mydata && dentptr) {
		unsigned long filesize = FAT2CPU32(dentptr->size);

		debug("Filesize: %lu bytes (%lu)\n", filesize, minsize);

		if (filesize < 0x7FFFFFFF && filesize > minsize) {
			file_fat_read_reset(fat);
			ret = (long)filesize;
		} else
			ret = -2;
	} else {
		debug("bad FAT\n");
		ret = -1;
	}

	fat->mydata = mydata;
	fat->dentptr = dentptr;

	return ret;
}

void file_fat_read_reset (fat_read_info *fat)
{
	fat->curr_cluster = 0;
	fat->curr_clust_offset = 0;
	debug("## [reset] current cluster: %u, current file cluster offset: %u\n",
			fat->curr_cluster, fat->curr_clust_offset);
}

/*
 * Read at most 'size' bytes from the specified cluster into 'buffer'.
 * Return 0 on success, -1 otherwise.
 */
static int
fat_cluster_read (fat_read_info *fat, uint32_t offset, void *buffer, uint32_t size)
{
	fsdata *mydata = fat->mydata;
	__u32 startsect;
	uint32_t curr_sect_offset;
	long leading;
	uint32_t padding;
	__u32 nr_sects;
	int ret;

	if (fat->curr_cluster == 0) {
		debug("[cluster_read] start file offset: %u, start clust: %u, start sect: %u, reading size: %u\n",
				offset, fat->curr_cluster, startsect, size);
		return -1;
	}

	/* The sector of the current cluster */
	startsect = mydata->data_begin + fat->curr_cluster * mydata->clust_size;

	debug("[cluster_read] start file offset: %u, start clust: %u, start sect: %u, reading size: %u\n",
			offset, fat->curr_cluster, startsect, size);

	/* The sector of 'offset' */
	leading = (offset - fat->curr_clust_offset) / mydata->sect_size;
	startsect += leading;
	curr_sect_offset = fat->curr_clust_offset + leading * mydata->sect_size;
	debug("[cluster_read] start sector for the start file offset: %u, start file sector offset: %u bytes\n",
			startsect, curr_sect_offset);

	leading = offset - curr_sect_offset;
	debug("[cluster_read] leading: %ld\n", leading);
	if (leading > 0) {
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
		__u8 *tmpbuf = (__u8*)CONFIG_SPL_FAT_TMP_BUFFER_ADDR;
#else
		__u8 tmpbuf[mydata->sect_size];
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */

		debug("[cluster_read] start sector: %u, leading: %u\n",
				startsect, (uint32_t)(mydata->sect_size - leading));

		ret = disk_read(startsect, 1, tmpbuf);
		if (ret != 1) {
			printf("ERR: read data 1 (got %d)\n", ret);
			return -1;
		}

		memcpy(buffer, &(tmpbuf[leading]), (mydata->sect_size - leading));

		if (size > (mydata->sect_size - leading))
			size -= (mydata->sect_size - leading);
		else
			return 0;

		buffer += (mydata->sect_size - leading);
		startsect++;
	}

	nr_sects = size / mydata->sect_size;
	padding = size % mydata->sect_size;
	if (nr_sects > 0) {
		debug("[cluster_read] number of sectors: %u (%u bytes), padding: %u\n",
				nr_sects, (uint32_t)(nr_sects * mydata->sect_size), padding);
		ret = disk_read(startsect, nr_sects, (__u8*)buffer);
		if (ret != nr_sects) {
			printf("ERR: read data 2 (got %d)\n", ret);
			return -1;
		}
	}

	if (padding) {
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
		__u8 *tmpbuf = (__u8*)CONFIG_SPL_FAT_TMP_BUFFER_ADDR;
#else
		__u8 tmpbuf[mydata->sect_size];
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */

		debug("[cluster_read] start sector: %u, padding: %u\n",
				startsect + nr_sects, padding);

		ret = disk_read(startsect + nr_sects, 1, tmpbuf);
		if (ret != 1) {
			printf("ERR: read data 3 (got %d)\n", ret);
			return -1;
		}
		buffer += (nr_sects * mydata->sect_size);
		memcpy(buffer, tmpbuf, padding);
	}

	return 0;
}

long file_fat_read_data (fat_read_info *fat, uint32_t offset, void *buffer, uint32_t size)
{
	fsdata *mydata = fat->mydata;
	dir_entry *dentptr = fat->dentptr;
	unsigned long filesize = FAT2CPU32(dentptr->size), gotsize = 0;
	unsigned int bytesperclust = mydata->clust_size * mydata->sect_size;
	__u32 curclust, curr_clust_offset;
	__u32 endclust, newclust = 0;
	unsigned long actsize, legacyOffset;

	debug("%s: Filesize: %lu bytes\n", __func__, filesize);

	if (size > 0 && filesize > size) filesize = size;

	debug("Reading size: %ld bytes, offset %u bytes\n", filesize, offset);
	debug("Cluster size: %u bytes, Sector size: %d bytes\n", bytesperclust, mydata->sect_size);
	debug("curr_clust: %u, curr_clust_offset: %u\n", fat->curr_cluster, fat->curr_clust_offset);

	/* Find the cluster of 'offset' */
	if ((offset == 0) || (fat->curr_cluster == 0) ||
			(fat->curr_clust_offset == 0) || (offset < fat->curr_clust_offset)) {
		/* Workaround: the backward file offset will start from the begining of the file. */
		curclust = START(dentptr);
		curr_clust_offset = 0;
	} else {
		curclust = fat->curr_cluster;
		curr_clust_offset = fat->curr_clust_offset;
	}

	while ((curr_clust_offset + bytesperclust) <= offset) {
		newclust = get_fatent(mydata, curclust);
		if (CHECK_CLUST(newclust, mydata->fatsize)) {
			debug("new cluster#: 0x%x, new cluster offset: %u\n", newclust, curr_clust_offset + bytesperclust);
			printf("bad FAT entry (0x%x)(0x%x)\n", newclust, curclust);
			return -1;
		}
		curclust = newclust;
		curr_clust_offset += bytesperclust;
	}

	legacyOffset = offset - curr_clust_offset;

	do {
		endclust = curclust;
		fat->curr_cluster = curclust;
		fat->curr_clust_offset = curr_clust_offset;

		debug("curr_clust: %u, curr_clust_offset: %u\n", fat->curr_cluster, fat->curr_clust_offset);

		if (legacyOffset)
			actsize = (filesize > (bytesperclust - legacyOffset)) ? (bytesperclust - legacyOffset) : filesize;
		else if ((filesize - gotsize) > bytesperclust)
			actsize = bytesperclust;
		else
			actsize = filesize - gotsize;

		do {
			/* Over the limitation */
			if ((gotsize + actsize) >= filesize) {
				actsize = filesize - gotsize;
				break;
			}

			newclust = get_fatent(mydata, endclust);

			/* EOF,Error or Not consecutive */
			if ((newclust - 1) != endclust)
				break;

			if (CHECK_CLUST(newclust, mydata->fatsize)) {
				printf("bad FAT entry (0x%x)(0x%x)\n", newclust, endclust);
				break;
			}

			endclust = newclust;
			actsize += bytesperclust;
			curr_clust_offset += bytesperclust;
		} while(1);

		if (fat_cluster_read(fat, (fat->curr_clust_offset + legacyOffset), buffer, actsize) != 0) {
			printf("ERR: read cluster (0x%x)\n", fat->curr_cluster);
			return -1;
		}	

		/* get remaining bytes */
		gotsize += (int)actsize;
		buffer += actsize;
		curclust = newclust;
		curr_clust_offset += bytesperclust;
		legacyOffset = 0;
	} while ((gotsize < filesize) && (!CHECK_CLUST(curclust, mydata->fatsize)));

	return gotsize;
}

void file_fat_read_exit (fat_read_info *fat)
{
	if (fat && fat->mydata) {
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SYS_SPL_MALLOC_SIZE)
#else
		if (fat->mydata->fatbuf) {
			free(fat->mydata->fatbuf);
			fat->mydata->fatbuf = NULL;
		}
#endif /* CONFIG_SPL_BUILD, CONFIG_SYS_SPL_MALLOC_SIZE */
	}
}
