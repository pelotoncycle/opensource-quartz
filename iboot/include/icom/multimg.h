/* mkmultimg/multimg.h
**
** Copyright 2014, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License"); 
** you may not use this file except in compliance with the License. 
** You may obtain a copy of the License at 
**
**     http://www.apache.org/licenses/LICENSE-2.0 
**
** Unless required by applicable law or agreed to in writing, software 
** distributed under the License is distributed on an "AS IS" BASIS, 
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
** See the License for the specific language governing permissions and 
** limitations under the License.
*/

#ifndef _ICOM_MULTI_IMAGE_H_
#define _ICOM_MULTI_IMAGE_H_

typedef struct multi_img_hdr multi_img_hdr;
typedef struct multi_img_entry multi_img_entry;

#define MULTI_IMAGE_MAGIC "INNOCOMM"
#define MULTI_IMAGE_MAGIC_SIZE 8
#define MULTI_IMAGE_NAME_SIZE 16
#define MULTI_IMAGE_ENTRY_NUM 12
#define MULTI_IMAGE_ENTRY_NAME_SIZE 16
#define MULTI_IMAGE_ENTRY_TYPE_SIZE 8

struct multi_img_entry
{
    unsigned char name[MULTI_IMAGE_ENTRY_NAME_SIZE]; /* asciiz image name (\0 is excluded) */
    unsigned char type[MULTI_IMAGE_ENTRY_TYPE_SIZE]; /* future expansion: asciiz image type (\0 is excluded) */
    unsigned image_offset;    /* image offset */
    unsigned image_size;      /* image size in bytes */
    unsigned flag;            /* future expansion: should be 0 */
};

struct multi_img_hdr
{
    unsigned char magic[MULTI_IMAGE_MAGIC_SIZE];
    unsigned page_size;       /* flash page size we assume */
    unsigned header_checksum; /* header crc32 checksum */
    unsigned checksum[8];     /* sha1 checksum */
    unsigned next_header;     /* next header offset */
    unsigned char name[MULTI_IMAGE_NAME_SIZE]; /* asciiz image name (\0 is excluded) */
    unsigned unused[2];       /* future expansion: should be 0 */
    unsigned num_of_images;   /* the number of images */
    struct multi_img_entry images[MULTI_IMAGE_ENTRY_NUM];
};

/*
** +------------------------+ 
** | 1st multi image header | 1 page
** +------------------------+
** | first image            | n pages
** +------------------------+
** | second image           | m pages
** +------------------------+
** | third image            | o pages
** +------------------------+
** | ...                    | p pages
** +------------------------+
** | 12th image             | q pages
** +------------------------+
** | 2nd multi image header | 1 page
** +------------------------+
** | first image            | x pages
** +------------------------+
** | ...                    | y pages
** +------------------------+
**
** n = (first image size + page_size - 1) / page_size
** m = (second image size + page_size - 1) / page_size
** o = (third image size + page_size - 1) / page_size
**
** all entities are page_size aligned in flash
*/

#endif /* _ICOM_MULTI_IMAGE_H_ */
