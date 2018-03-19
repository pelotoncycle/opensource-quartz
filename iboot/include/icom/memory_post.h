/*
 * (C) Copyright 2014
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _MEMORY_POST_H
#define _MEMORY_POST_H

typedef void (*memory_post_log_t)(const char* fmt, ...);

void memory_post_init(memory_post_log_t func);
void memory_post_exit(void);

int memory_post_databus(unsigned long *address);
int memory_post_addrline2(unsigned long start, unsigned long size);

int memory_post_dataline(unsigned long long * pmem);
int memory_post_addrline(ulong *testaddr, ulong *base, ulong size);
int memory_post_test1(unsigned long start, unsigned long size, unsigned long val);
int memory_post_test2(unsigned long start, unsigned long size);
int memory_post_test3(unsigned long start, unsigned long size);
int memory_post_test4(unsigned long start, unsigned long size);

#endif /* _MEMORY_POST_H */
