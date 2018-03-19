/*
 * Copyright (C) 2008 RuggedCom, Inc.
 * Richard Retanubun <RichardRetanubun@RuggedCom.com>
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

/*
 * Problems with CONFIG_SYS_64BIT_LBA:
 *
 * struct disk_partition.start in include/part.h is sized as ulong.
 * When CONFIG_SYS_64BIT_LBA is activated, lbaint_t changes from ulong to uint64_t.
 * For now, it is cast back to ulong at assignment.
 *
 * This limits the maximum size of addressable storage to < 2 Terra Bytes
 */
#include <common.h>
#include <command.h>
#include <ide.h>
#include <malloc.h>
#include "part_efi.h"
#include <linux/ctype.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_CMD_IDE) || \
    defined(CONFIG_CMD_SATA) || \
    defined(CONFIG_CMD_SCSI) || \
    defined(CONFIG_CMD_USB) || \
    defined(CONFIG_MMC) || \
    defined(CONFIG_SYSTEMACE)

/**
 * efi_crc32() - EFI version of crc32 function
 * @buf: buffer to calculate crc32 of
 * @len - length of buf
 *
 * Description: Returns EFI-style CRC32 value for @buf
 */
static inline u32 efi_crc32(const void *buf, u32 len)
{
	return crc32(0, buf, len);
}

/*
 * Private function prototypes
 */

static int pmbr_part_valid(struct partition *part);
static int is_pmbr_valid(legacy_mbr * mbr);
static int is_gpt_valid(block_dev_desc_t * dev_desc, unsigned long long lba,
				gpt_header * pgpt_head, gpt_entry ** pgpt_pte);
static gpt_entry *alloc_read_gpt_entries(block_dev_desc_t * dev_desc,
				gpt_header * pgpt_head);
static int is_pte_valid(gpt_entry * pte);

static char *print_efiname(gpt_entry *pte)
{
	static char name[PARTNAME_SZ + 1];
	int i;
	for (i = 0; i < PARTNAME_SZ; i++) {
		u8 c;
		c = pte->partition_name[i] & 0xff;
		c = (c && !isprint(c)) ? '.' : c;
		name[i] = c;
	}
	name[PARTNAME_SZ] = 0;
	return name;
}

#ifndef CONFIG_SYS_MINI_FAT
static void uuid_string(unsigned char *uuid, char *str)
{
	static const u8 le[16] = {3, 2, 1, 0, 5, 4, 7, 6, 8, 9, 10, 11,
				  12, 13, 14, 15};
	int i;

	for (i = 0; i < 16; i++) {
		sprintf(str, "%02x", uuid[le[i]]);
		str += 2;
		switch (i) {
		case 3:
		case 5:
		case 7:
		case 9:
			*str++ = '-';
			break;
		}
	}
}

static efi_guid_t system_guid = PARTITION_SYSTEM_GUID;

static inline int is_bootable(gpt_entry *p)
{
	return p->attributes.fields.legacy_bios_bootable ||
		!memcmp(&(p->partition_type_guid), &system_guid,
			sizeof(efi_guid_t));
}
#endif /* !CONFIG_SYS_MINI_FAT */

/*
 * Public Functions (include/part.h)
 */

#ifndef CONFIG_SYS_MINI_FAT
void print_part_efi(block_dev_desc_t * dev_desc)
{
	ALLOC_CACHE_ALIGN_BUFFER(gpt_header, gpt_head, 1);
	gpt_entry *gpt_pte = NULL;
	int i = 0;
	char uuid[37];

	if (!dev_desc) {
		printf("%s: Invalid Argument(s)\n", __func__);
		return;
	}
	/* This function validates AND fills in the GPT header and PTE */
#if 0
	if (is_gpt_valid(dev_desc, GPT_PRIMARY_PARTITION_TABLE_LBA,
			 gpt_head, &gpt_pte) != 1) {
		printf("%s: *** ERROR: Invalid GPT ***\n", __func__);
		return;
	}
#else
{
	int ret = is_gpt_valid(dev_desc, GPT_PRIMARY_PARTITION_TABLE_LBA,
			gpt_head, &gpt_pte);
	if (ret != 1) {
		if (ret != 0) {
			printf("%s: Primary GPT is invalid, using alternate GPT\n", __func__);
			ret = is_gpt_valid(dev_desc, (unsigned long long)(dev_desc->lba - 1),
						gpt_head, &gpt_pte);
		}
		if (ret != 1) {
			printf("%s: *** ERROR: Invalid GPT ***\n", __func__);
			return;
		}
	}
}
#endif

	debug("%s: gpt-entry at %p\n", __func__, gpt_pte);

	printf("Part\tStart LBA\tEnd LBA\t\tName\n");
	printf("\tAttributes\n");
	printf("\tType UUID\n");
	printf("\tPartition UUID\n");

	for (i = 0; i < le32_to_cpu(gpt_head->num_partition_entries); i++) {
		/* Stop at the first non valid PTE */
		if (!is_pte_valid(&gpt_pte[i]))
			break;

		printf("%3d\t0x%08llx\t0x%08llx\t\"%s\"\n", (i + 1),
			le64_to_cpu(gpt_pte[i].starting_lba),
			le64_to_cpu(gpt_pte[i].ending_lba),
			print_efiname(&gpt_pte[i]));
		printf("\tattrs:\t0x%016llx\n", gpt_pte[i].attributes.raw);
		uuid_string(gpt_pte[i].partition_type_guid.b, uuid);
		printf("\ttype:\t%s\n", uuid);
		uuid_string(gpt_pte[i].unique_partition_guid.b, uuid);
		printf("\tuuid:\t%s\n", uuid);
	}

	/* Remember to free pte */
	free(gpt_pte);
	return;
}
#endif /* !CONFIG_SYS_MINI_FAT */

int get_partition_info_efi(block_dev_desc_t * dev_desc, int part,
				disk_partition_t * info)
{
	ALLOC_CACHE_ALIGN_BUFFER(gpt_header, gpt_head, 1);
	gpt_entry *gpt_pte = NULL;

	/* "part" argument must be at least 1 */
	if (!dev_desc || !info || part < 1) {
		printf("%s: Invalid Argument(s)\n", __func__);
		return -1;
	}

	/* This function validates AND fills in the GPT header and PTE */
#if 0
	if (is_gpt_valid(dev_desc, GPT_PRIMARY_PARTITION_TABLE_LBA,
			gpt_head, &gpt_pte) != 1) {
		printf("%s: *** ERROR: Invalid GPT ***\n", __func__);
		return -1;
	}
#else
{
	int ret = is_gpt_valid(dev_desc, GPT_PRIMARY_PARTITION_TABLE_LBA,
			gpt_head, &gpt_pte);
	if (ret != 1) {
		if (ret != 0) {
			printf("%s: Primary GPT is invalid, using alternate GPT\n", __func__);
			ret = is_gpt_valid(dev_desc, (unsigned long long)(dev_desc->lba - 1),
						gpt_head, &gpt_pte);
		}
		if (ret != 1) {
			printf("%s: *** ERROR: Invalid GPT ***\n", __func__);
			return -1;
		}
	}
}
#endif

	if (part > le32_to_cpu(gpt_head->num_partition_entries) ||
	    !is_pte_valid(&gpt_pte[part - 1])) {
		printf("%s: *** ERROR: Invalid partition number %d ***\n",
			__func__, part);
		return -1;
	}

	/* The ulong casting limits the maximum disk size to 2 TB */
	info->start = (u64)le64_to_cpu(gpt_pte[part - 1].starting_lba);
	/* The ending LBA is inclusive, to calculate size, add 1 to it */
	info->size = ((u64)le64_to_cpu(gpt_pte[part - 1].ending_lba) + 1)
		     - info->start;
	info->blksz = GPT_BLOCK_SIZE;

	sprintf((char *)info->name, "%s",
			print_efiname(&gpt_pte[part - 1]));
#if 0
	sprintf((char *)info->type, "U-Boot");
#endif

	debug("%s: start 0x%lX, size 0x%lX, name %s", __func__,
		info->start, info->size, info->name);

	/* Remember to free pte */
	free(gpt_pte);
	return 0;
}

#define GPT_HEADER_SIZE						92UL
#define GPT_PRIMARY_PARTITION_ENTRY_LBA		2ULL
#define GPT_SECONDARY_PARTITION_ENTRY_LBA	34ULL
#define GPT_FIRST_USABLE_LBA				66ULL

static const efi_guid_t partition_type = {
	.b = {
		0xa2, 0xa0, 0xd0, 0xeb, 0xe5, 0xb9, 0x33, 0x44,
		0x87, 0xc0, 0x68, 0xb6, 0xb7, 0x26, 0x99, 0xc7,
	},
};

static const efi_guid_t random_uuid = {
	.b = {
		0xff, 0x1f, 0xf2, 0xf9, 0xd4, 0xa8, 0x0e, 0x5f,
		0x97, 0x46, 0x59, 0x48, 0x69, 0xae, 0xc3, 0x4e,
	},
};

static void recover_primary_gpt(block_dev_desc_t * dev_desc,
			gpt_header *pgpt_head, gpt_entry *pgpt_pte)
{
	ALLOC_CACHE_ALIGN_BUFFER(gpt_header, primary_gpt_head, 1);
	u32 calc_crc32;
	unsigned long writes;
	unsigned long count;

	/* Primary Partition Header */
	memcpy(primary_gpt_head, pgpt_head, sizeof(gpt_header));

	/* location of the primary header */
	primary_gpt_head->my_lba = cpu_to_le64(GPT_PRIMARY_PARTITION_TABLE_LBA);
	/* location of the alternate header */
	/* The last LBA is reserved for the alternate GPT header */
	primary_gpt_head->alternate_lba = cpu_to_le64(dev_desc->lba - 1ULL);
	/* Partition entries starting LBA (always 2 in primary copy) */
	primary_gpt_head->partition_entry_lba = cpu_to_le64(GPT_PRIMARY_PARTITION_ENTRY_LBA);

	/* the GUID Partition Table CRC */
	primary_gpt_head->header_crc32 = cpu_to_le32(0UL);
	calc_crc32 = efi_crc32((const unsigned char *)primary_gpt_head, GPT_HEADER_SIZE);
	primary_gpt_head->header_crc32 = cpu_to_le32(calc_crc32);

	/* Write Primary GPT Header to device */
	debug("%s: start %llu, count %u, buffer 0x%08x\n", __func__, GPT_PRIMARY_PARTITION_TABLE_LBA, 1, (u32)primary_gpt_head);
	writes = dev_desc->block_write(dev_desc->dev, GPT_PRIMARY_PARTITION_TABLE_LBA, 1, primary_gpt_head);
	if (writes != 1) {
		printf("GPT: restore primary header write err: %lu\n", writes);
		return;
	}

	/* Write Primary GPT Entries to device */
	/* GPT_BLOCK_SIZE aligned partition number */
	count = (unsigned long)(le32_to_cpu(pgpt_head->num_partition_entries) *
			le32_to_cpu(pgpt_head->sizeof_partition_entry));
	count = ALIGN(count, GPT_BLOCK_SIZE) / GPT_BLOCK_SIZE;
	debug("%s: start %llu, count %lu, buffer 0x%08x\n", __func__, GPT_PRIMARY_PARTITION_ENTRY_LBA, count, (u32)pgpt_pte);
	writes = dev_desc->block_write(dev_desc->dev, GPT_PRIMARY_PARTITION_ENTRY_LBA, (lbaint_t)count, pgpt_pte);
	if (writes != count) {
		printf("GPT: restore primary entries write err: %lu<->%lu\n", writes, count);
		return;
	}

	puts("Primary GPT is recovered\n");
}

int load_part_efi(block_dev_desc_t * dev_desc,
		int (*part_import)(block_dev_desc_t *dev_desc, int part, disk_partition_t *info))
{
	ALLOC_CACHE_ALIGN_BUFFER(gpt_header, gpt_head, 1);
	int ret = 0;
	gpt_entry *gpt_pte = NULL;
	efi_char16_t *name = NULL;
	disk_partition_t info;
	int part_num = 0;
	int i = 0, j = 0;

	if (!dev_desc || !part_import) {
		printf("%s: Invalid Argument(s)\n", __func__);
		return -1;
	}
	/* This function validates AND fills in the GPT header and PTE */
	memset(gpt_head, 0, sizeof(gpt_head));
	ret = is_gpt_valid(dev_desc, GPT_PRIMARY_PARTITION_TABLE_LBA,
			 gpt_head, &gpt_pte);
	if (ret != 1) {
		if (ret != 0) {
			puts("Primary GPT is invalid, using alternate GPT\n");
			ret = is_gpt_valid(dev_desc, (unsigned long long)(dev_desc->lba - 1),
						gpt_head, &gpt_pte);
		}
		if (ret != 1) {
			printf("%s: *** ERROR: Invalid GPT ***\n", __func__);
			return -1;
		} else {
			/* Recover Primary GPT */
			recover_primary_gpt(dev_desc, gpt_head, gpt_pte);
		}
	}

	part_num = (int)le32_to_cpu(gpt_head->num_partition_entries);
	for (i = 0; i < part_num; i++) {
		if (is_pte_valid(&(gpt_pte[i]))) {
			/* The ulong casting limits the maximum disk size to 2 TB */
			info.start = (ulong)le64_to_cpu(gpt_pte[i].starting_lba);
			/* The ending LBA is inclusive, to calculate size, add 1 to it */
			info.size = ((ulong)le64_to_cpu(gpt_pte[i].ending_lba) + 1) - info.start;

			info.blksz = GPT_BLOCK_SIZE;

			name = gpt_pte[i].partition_name;
			for (j = 0; (j < sizeof(info.name)) && *name; j++)
				info.name[j] = *name++;
			info.name[j] = '\0';

			ret = part_import(dev_desc, i, &info);
			if (ret) {
				debug("%s: failed to load part %d\n", __func__, ret);
				break;
			}
		} else {
			break;	/* Stop at the first non valid PTE */
		}
	}

	/* Remember to free pte */
	free(gpt_pte);
	return ret;
}

/* Initiate the protective MBR */
void init_pmbr(legacy_mbr *mbr, u32 nr_lbas)
{
	struct partition *part;

	memset(mbr, 0, sizeof(legacy_mbr));

	mbr->signature = cpu_to_le16(MSDOS_MBR_SIGNATURE);

	part = &(mbr->partition_record[0]);

	part->boot_ind = 0x00; /* non-bootable */
	part->head = 0xFF; /* bogus CHS */
	part->sector = 0xFF; /* bogus CHS */
	part->cyl = 0xFF; /* bogus CHS */
	part->sys_ind = EFI_PMBR_OSTYPE_EFI_GPT; /* GPT partition */
	part->end_head = 0xFF; /* bogus CHS */
	part->end_sector = 0xFF; /* bogus CHS */
	part->end_cyl = 0xFF; /* bogus CHS */
	part->start_sect = cpu_to_le32(1UL);
	part->nr_sects = cpu_to_le32(nr_lbas);
}

/* Initiate the primary GPT header */
void init_def_gpt_header(gpt_header *pgpt_head, int part_num, u64 nr_lbas)
{
	memset(pgpt_head, 0, sizeof(gpt_header));

	/* Signature */
	pgpt_head->signature = cpu_to_le64(GPT_HEADER_SIGNATURE);

	/* Revision */
	pgpt_head->revision = cpu_to_le32(GPT_HEADER_REVISION_V1);

	/* Header size */
	pgpt_head->header_size = cpu_to_le32(GPT_HEADER_SIZE);

	/* Disk GUID */
	memcpy(&pgpt_head->disk_guid, &random_uuid, sizeof(efi_guid_t));

	/* Number of partition entries */
	pgpt_head->num_partition_entries = cpu_to_le32(part_num);

	/* Size of a partition entry (usually 128) */
	pgpt_head->sizeof_partition_entry = cpu_to_le32(sizeof(gpt_entry));

	/* Primary Partition Header */

	/* location of the primary header */
	pgpt_head->my_lba = cpu_to_le64(GPT_PRIMARY_PARTITION_TABLE_LBA);

	/* location of the alternate header */
	/* The last LBA is reserved for the alternate GPT header */
	pgpt_head->alternate_lba = cpu_to_le64(nr_lbas - 1ULL);

	/* first usable LBA for partitions (primiary partition table last LBA + 1) */
	pgpt_head->first_usable_lba = cpu_to_le64(GPT_FIRST_USABLE_LBA);

	/* last usable LBA for partitions (secondary partition table first LBA - 1) */
	/* The last LBA is reserved for the alternate GPT header */
	pgpt_head->last_usable_lba = cpu_to_le64(nr_lbas - 2ULL);

	/* Partition entries starting LBA (always 2 in primary copy) */
	pgpt_head->partition_entry_lba = cpu_to_le64(GPT_PRIMARY_PARTITION_ENTRY_LBA);
}

int save_part_efi(block_dev_desc_t * dev_desc, int part_num,
		int (*part_export)(block_dev_desc_t *dev_desc, int part, disk_partition_t *info))
{
	int ret = 0;
	unsigned long writes;
	unsigned long count;
	disk_partition_t info;
	ALLOC_CACHE_ALIGN_BUFFER(legacy_mbr, legacymbr, 1);
	ALLOC_CACHE_ALIGN_BUFFER(gpt_header, gpt_head, 1);
	gpt_entry *gpt_pte = NULL;
	u32 calc_crc32;
	char *name = NULL;
	int i = 0, j = 0;

	if (!dev_desc || !part_export || part_num <= 0 || part_num > 128) {
		printf("%s: Invalid Argument(s) (%d)\n", __func__, part_num);
		return -1;
	}

	init_def_gpt_header(gpt_head, part_num, (u64)dev_desc->lba);

	count = ALIGN(sizeof(gpt_entry) * part_num, GPT_BLOCK_SIZE);
	gpt_pte = memalign(ARCH_DMA_MINALIGN, count);
	if (!gpt_pte) {
		printf("%s: out of memory (%lu)\n", __func__, count);
		return -1;
	}
	memset(gpt_pte, 0, count);

	for (i = 0; i < part_num; i++) {
		ret = part_export(dev_desc, i, &info);
		if (ret) {
			debug("%s: failed to get part %d\n", __func__, ret);
			goto exit0;
		}
		/* The maximum disk size is 2 TB */
		gpt_pte[i].starting_lba = cpu_to_le64((u64)info.start);
		gpt_pte[i].ending_lba = cpu_to_le64((u64)(info.start + info.size - 1));

		memcpy(&(gpt_pte[i].partition_type_guid), &partition_type, sizeof(efi_guid_t));
		memcpy(&(gpt_pte[i].unique_partition_guid), &random_uuid, sizeof(efi_guid_t));
		gpt_pte[i].unique_partition_guid.b[0] = (unsigned char)i;

		name = (char*)info.name;
		for (j = 0; (j < sizeof(info.name)) && *name; j++)
			gpt_pte[i].partition_name[j] = *name++;
		gpt_pte[i].partition_name[j] = '\0';
	}

	/* Primary Partition Header */
	/* the GUID Partition Table Entry Array CRC */
	calc_crc32 = efi_crc32((const unsigned char *)gpt_pte, sizeof(gpt_entry) * part_num);
	gpt_head->partition_entry_array_crc32 = cpu_to_le32(calc_crc32);
	/* the GUID Partition Table CRC */
	gpt_head->header_crc32 = cpu_to_le32(0UL);
	calc_crc32 = efi_crc32((const unsigned char *)gpt_head, GPT_HEADER_SIZE);
	gpt_head->header_crc32 = cpu_to_le32(calc_crc32);

	init_pmbr(legacymbr, (u32)dev_desc->lba);

	/* Write legacy MBR to device block 0 */
	writes = dev_desc->block_write(dev_desc->dev, 0, 1, legacymbr);
	if (writes != 1) {
		printf("GPT: MBR write err: %lu\n", writes);
		ret = -1;
		goto exit0;
	}

	/* Write Primary GPT Header to device */
	writes = dev_desc->block_write(dev_desc->dev, GPT_PRIMARY_PARTITION_TABLE_LBA, 1, gpt_head);
	if (writes != 1) {
		printf("GPT: primary header write err: %lu\n", writes);
		ret = -1;
		goto exit0;
	}

	/* Write Primary GPT Entries to device */
	count /= GPT_BLOCK_SIZE;
	writes = dev_desc->block_write(dev_desc->dev, GPT_PRIMARY_PARTITION_ENTRY_LBA, (lbaint_t)count, gpt_pte);
	if (writes != count) {
		printf("GPT: primary entries write err: %lu<->%lu\n", writes, count);
		ret = -1;
		goto exit0;
	}

	/* Secondary/Alternate Partition Header */
	/* location of the primary header */
	/* The last LBA is reserved for the alternate GPT header */
	gpt_head->my_lba = cpu_to_le64(dev_desc->lba - 1ULL);
	/* location of the alternate header */
	gpt_head->alternate_lba = cpu_to_le64(GPT_PRIMARY_PARTITION_TABLE_LBA);
	/* Partition entries starting LBA (always 2 in primary copy) */
	gpt_head->partition_entry_lba = cpu_to_le64(GPT_SECONDARY_PARTITION_ENTRY_LBA);
	/* the GUID Partition Table CRC */
	gpt_head->header_crc32 = cpu_to_le32(0UL);
	calc_crc32 = efi_crc32((const unsigned char *)gpt_head, GPT_HEADER_SIZE);
	gpt_head->header_crc32 = cpu_to_le32(calc_crc32);

	/* Write Secondary/Alternate GPT Header to the last LBA */
	writes = dev_desc->block_write(dev_desc->dev, (unsigned long)(dev_desc->lba - 1ULL), 1, gpt_head);
	if (writes != 1) {
		printf("GPT: alternate header write err: %lu\n", writes);
		ret = -1;
		goto exit0;
	}

	/* Write Secondary/Alternate GPT Entries to device */
	writes = dev_desc->block_write(dev_desc->dev, GPT_SECONDARY_PARTITION_ENTRY_LBA, (lbaint_t)count, gpt_pte);
	if (writes != count) {
		printf("GPT: alternate entries write err: %lu<->%lu\n", writes, count);
		ret = -1;
		goto exit0;
	}

exit0:
	/* Remember to free pte */
	if (gpt_pte != NULL)
		free(gpt_pte);

	return ret;
}

int test_part_efi(block_dev_desc_t * dev_desc)
{
	ALLOC_CACHE_ALIGN_BUFFER(legacy_mbr, legacymbr, 1);

	/* Read legacy MBR from block 0 and validate it */
	if ((dev_desc->block_read(dev_desc->dev, 0, 1, (ulong *)legacymbr) != 1)
		|| (is_pmbr_valid(legacymbr) != 1)) {
		return -1;
	}
	return 0;
}

/*
 * Private functions
 */
/*
 * pmbr_part_valid(): Check for EFI partition signature
 *
 * Returns: 1 if EFI GPT partition type is found.
 */
static int pmbr_part_valid(struct partition *part)
{
	if (part->sys_ind == EFI_PMBR_OSTYPE_EFI_GPT &&
		le32_to_cpu(part->start_sect) == 1UL) {
		return 1;
	}

	return 0;
}

/*
 * is_pmbr_valid(): test Protective MBR for validity
 *
 * Returns: 1 if PMBR is valid, 0 otherwise.
 * Validity depends on two things:
 *  1) MSDOS signature is in the last two bytes of the MBR
 *  2) One partition of type 0xEE is found, checked by pmbr_part_valid()
 */
static int is_pmbr_valid(legacy_mbr * mbr)
{
	int i = 0;

	if (!mbr || le16_to_cpu(mbr->signature) != MSDOS_MBR_SIGNATURE)
		return 0;

	for (i = 0; i < 4; i++) {
		if (pmbr_part_valid(&mbr->partition_record[i])) {
			return 1;
		}
	}
	return 0;
}

/**
 * is_gpt_valid() - tests one GPT header and PTEs for validity
 *
 * lba is the logical block address of the GPT header to test
 * gpt is a GPT header ptr, filled on return.
 * ptes is a PTEs ptr, filled on return.
 *
 * Description: returns 1 if valid,  0 on error.
 * If valid, returns pointers to PTEs.
 */
static int is_gpt_valid(block_dev_desc_t * dev_desc, unsigned long long lba,
			gpt_header * pgpt_head, gpt_entry ** pgpt_pte)
{
	u32 crc32_backup = 0;
	u32 calc_crc32;
	unsigned long long lastlba;

	if (!dev_desc || !pgpt_head) {
		printf("%s: Invalid Argument(s)\n", __func__);
		return 0;
	}

	/* Read GPT Header from device */
	if (dev_desc->block_read(dev_desc->dev, lba, 1, pgpt_head) != 1) {
		printf("*** ERROR: Can't read GPT header ***\n");
		return 0;
	}

	/* Check the GPT header signature */
	if (le64_to_cpu(pgpt_head->signature) != GPT_HEADER_SIGNATURE) {
		printf("GUID Partition Table Header signature is wrong:"
			"0x%llX != 0x%llX\n",
			le64_to_cpu(pgpt_head->signature),
			GPT_HEADER_SIGNATURE);
		return -1; /* -1: not I/O error */
	}

	/* Check the GUID Partition Table CRC */
	memcpy(&crc32_backup, &pgpt_head->header_crc32, sizeof(crc32_backup));
	memset(&pgpt_head->header_crc32, 0, sizeof(pgpt_head->header_crc32));

	calc_crc32 = efi_crc32((const unsigned char *)pgpt_head,
		le32_to_cpu(pgpt_head->header_size));

	memcpy(&pgpt_head->header_crc32, &crc32_backup, sizeof(crc32_backup));

	if (calc_crc32 != le32_to_cpu(crc32_backup)) {
		printf("GUID Partition Table Header CRC is wrong:"
			"0x%x != 0x%x\n",
		       le32_to_cpu(crc32_backup), calc_crc32);
		return -1; /* -1: not I/O error */
	}

	/* Check that the my_lba entry points to the LBA that contains the GPT */
	if (le64_to_cpu(pgpt_head->my_lba) != lba) {
		printf("GPT: my_lba incorrect: %llX != %llX\n",
			le64_to_cpu(pgpt_head->my_lba),
			lba);
		return -1; /* -1: not I/O error */
	}

	/* Check the first_usable_lba and last_usable_lba are within the disk. */
	lastlba = (unsigned long long)dev_desc->lba;
	if (le64_to_cpu(pgpt_head->first_usable_lba) > lastlba) {
		printf("GPT: first_usable_lba incorrect: %llX > %llX\n",
			le64_to_cpu(pgpt_head->first_usable_lba), lastlba);
		return -1; /* -1: not I/O error */
	}
	if (le64_to_cpu(pgpt_head->last_usable_lba) > lastlba) {
		printf("GPT: last_usable_lba incorrect: %llX > %llX\n",
			(u64) le64_to_cpu(pgpt_head->last_usable_lba), lastlba);
		return -1; /* -1: not I/O error */
	}

	debug("GPT: first_usable_lba: %llX last_usable_lba %llX last lba %llX\n",
		le64_to_cpu(pgpt_head->first_usable_lba),
		le64_to_cpu(pgpt_head->last_usable_lba), lastlba);

	/* Read and allocate Partition Table Entries */
	*pgpt_pte = alloc_read_gpt_entries(dev_desc, pgpt_head);
	if (*pgpt_pte == NULL) {
		printf("GPT: Failed to allocate memory for PTE\n");
		return -1; /* -1: not I/O error */
	}

	/* Check the GUID Partition Table Entry Array CRC */
	calc_crc32 = efi_crc32((const unsigned char *)*pgpt_pte,
		le32_to_cpu(pgpt_head->num_partition_entries) *
		le32_to_cpu(pgpt_head->sizeof_partition_entry));

	if (calc_crc32 != le32_to_cpu(pgpt_head->partition_entry_array_crc32)) {
		printf("GUID Partition Table Entry Array CRC is wrong:"
			"0x%x != 0x%x\n",
			le32_to_cpu(pgpt_head->partition_entry_array_crc32),
			calc_crc32);

		free(*pgpt_pte);
		*pgpt_pte = NULL;
		return -1; /* -1: not I/O error */
	}

	/* We're done, all's well */
	return 1;
}

/**
 * alloc_read_gpt_entries(): reads partition entries from disk
 * @dev_desc
 * @gpt - GPT header
 *
 * Description: Returns ptes on success,  NULL on error.
 * Allocates space for PTEs based on information found in @gpt.
 * Notes: remember to free pte when you're done!
 */
static gpt_entry *alloc_read_gpt_entries(block_dev_desc_t * dev_desc,
					 gpt_header * pgpt_head)
{
	size_t count = 0;
	gpt_entry *pte = NULL;

	if (!dev_desc || !pgpt_head) {
		printf("%s: Invalid Argument(s)\n", __func__);
		return NULL;
	}

	count = le32_to_cpu(pgpt_head->num_partition_entries) *
		le32_to_cpu(pgpt_head->sizeof_partition_entry);
	count = ALIGN(count, GPT_BLOCK_SIZE);

	debug("%s: count = %u * %u = %zu\n", __func__,
	      (u32) le32_to_cpu(pgpt_head->num_partition_entries),
	      (u32) le32_to_cpu(pgpt_head->sizeof_partition_entry), count);

	/* Allocate memory for PTE, remember to FREE */
	if (count != 0) {
		pte = memalign(ARCH_DMA_MINALIGN, count);
	}

	if (count == 0 || pte == NULL) {
		printf("%s: ERROR: Can't allocate 0x%zX "
		       "bytes for GPT Entries\n",
			__func__, count);
		return NULL;
	}

	/* Read GPT Entries from device */
	if (dev_desc->block_read (dev_desc->dev,
		le64_to_cpu(pgpt_head->partition_entry_lba),
		(lbaint_t) (count / GPT_BLOCK_SIZE), pte)
		!= (count / GPT_BLOCK_SIZE)) {

		printf("*** ERROR: Can't read GPT Entries ***\n");
		free(pte);
		return NULL;
	}
	return pte;
}

static const efi_guid_t unused_guid = {
	.b = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	},
};

/**
 * is_pte_valid(): validates a single Partition Table Entry
 * @gpt_entry - Pointer to a single Partition Table Entry
 *
 * Description: returns 1 if valid,  0 on error.
 */
static int is_pte_valid(gpt_entry * pte)
{
#if 0
	efi_guid_t unused_guid;
#endif

	if (!pte) {
		printf("%s: Invalid Argument(s)\n", __func__);
		return 0;
	}

	/* Only one validation for now:
	 * The GUID Partition Type != Unused Entry (ALL-ZERO)
	 */
#if 0
	memset(unused_guid.b, 0, sizeof(unused_guid.b));
#endif

	if (memcmp(pte->partition_type_guid.b, unused_guid.b,
		sizeof(unused_guid.b)) == 0) {

		debug("%s: Found an unused PTE GUID at 0x%08X\n", __func__,
		(unsigned int)pte);

		return 0;
	/* James Wu: we only support one partition type */
	} else if (memcmp(pte->partition_type_guid.b, partition_type.b,
			sizeof(partition_type.b))) {
		debug("%s: Found an invalid PTE GUID at 0x%08X\n", __func__,
			(unsigned int)pte);
		return 0;
	} else {
		return 1;
	}
}
#endif
