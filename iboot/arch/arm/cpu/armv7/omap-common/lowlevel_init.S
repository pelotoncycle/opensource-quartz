/*
 * Board specific setup info
 *
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *	Aneesh V	<aneesh@ti.com>
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

#include <asm/arch/omap.h>
#include <linux/linkage.h>

ENTRY(save_boot_params)
	/*
	 * See if the rom code passed pointer is valid:
	 * It is not valid if it is not in non-secure SRAM
	 * This may happen if you are booting with the help of
	 * debugger
	 */
	ldr     r2, =NON_SECURE_SRAM_START
	cmp	r2, r0
	bgt	1f
	ldr	r2, =NON_SECURE_SRAM_END
	cmp	r2, r0
	blt	1f

	/*
	 * store the boot params passed from rom code or saved
	 * and passed by SPL
	 */
	cmp	r0, #0
	beq	1f
	ldr	r1, =boot_params
	str	r0, [r1]
#ifdef CONFIG_SPL_BUILD
	/* Store the boot device in omap_boot_device */
	ldrb	r2, [r0, #BOOT_DEVICE_OFFSET]	@ r1 <- value of boot device
	and	r2, #BOOT_DEVICE_MASK
	ldr	r3, =boot_params
	strb	r2, [r3, #BOOT_DEVICE_OFFSET]	@ omap_boot_device <- r1

	/* boot mode is passed only for devices that can raw/fat mode */
	cmp	r2, #2
	blt	2f
	cmp	r2, #7
	bgt	2f
	/* Store the boot mode (raw/FAT) in omap_boot_mode */
	ldr	r2, [r0, #DEV_DESC_PTR_OFFSET]	@ get the device descriptor ptr
	bic	r3, r2, #255
	cmp	r3, #0
	beq	1f
	ldr	r2, [r2, #DEV_DATA_PTR_OFFSET]	@ get the pDeviceData ptr
	bic	r3, r2, #255
	cmp	r3, #0
	beq	1f
	ldr	r2, [r2, #BOOT_MODE_OFFSET]	@ get the boot mode
	ldr	r3, =omap_bootmode
	str	r2, [r3]
/*#endif*/
2:
	ldrb	r2, [r0, #CH_FLAGS_OFFSET]
	ldr	r3, =boot_params
	strb	r2, [r3, #CH_FLAGS_OFFSET]
#endif /* CONFIG_SPL_BUILD */
1:
	bx	lr
ENDPROC(save_boot_params)

ENTRY(lowlevel_init)
	/*
	 * Setup a temporary stack
	 */
	ldr	sp, =LOW_LEVEL_SRAM_STACK

	/*
	 * Save the old lr(passed in ip) and the current lr to stack
	 */
	push	{ip, lr}

	/*
	 * go setup pll, mux, memory
	 */
	bl	s_init
	pop	{ip, pc}
ENDPROC(lowlevel_init)

#ifdef CONFIG_SPL_BUILD

.global spl_enable_irqs
.global spl_disable_irqs

spl_enable_irqs:
	cpsie I
	bx lr

spl_disable_irqs:
	cpsid I
	bx lr

#else

#if 0
ENTRY(set_pl310_ctrl_reg)
	PUSH	{r4-r11, lr}	@ save registers - ROM code may pollute
				@ our registers
	LDR	r12, =0x102	@ Set PL310 control register - value in R0
	.word	0xe1600070	@ SMC #0 - hand assembled because -march=armv5
				@ call ROM Code API to set control register
	POP	{r4-r11, pc}
ENDPROC(set_pl310_ctrl_reg)
#endif

#if 0
/**
 * omap_smc_rom() - This function is the entry point to services
 *		    exposed by ROM code
 * Prototype:
 * void omap_smc_rom(unsigned int rom_svc, unsigned int arg);
 * @rom_svc: Application ID of ROM api (input in r0 will move to r12)
 * @arg: Argument passed to ROM api (input in r1 will move to r0)
 */
ENTRY(omap_smc_rom)
	push {r2-r12, lr}	@ Save all registers from ROM code!
	mov	r12, r0	@ Copy the Service ID in R12
	mov	r0, r1	@ Copy parameter to R0
	mcr     p15, 0, r0, c7, c10, 4	@ DSB
/*	mcr     p15, 0, r0, c7, c5, 4	@ ISB*/
/*	mcr     p15, 0, r0, c7, c10, 5	@ DMB*/
	.word	0xe1600070	@ SMC #0 - hand assembled because -march=armv5
				@ call ROM Code API to set control register
	pop {r2-r12, pc}
ENDPROC(omap_smc_rom)
#else
/* Refer to Kernel "void omap_smc1(u32 fn, u32 arg)"
 * This is common routine to manage secure monitor API
 * used to modify the PL310 secure registers.
 * 'r0' contains the value to be modified and 'r12' contains
 * the monitor API number. It uses few CPU registers
 * internally and hence they need be backed up including
 * link register "lr".
 * Function signature : void omap_smc1(u32 fn, u32 arg)
 */
ENTRY(omap_smc_rom)
	stmfd   sp!, {r2-r12, lr}
	mov	r12, r0
	mov 	r0, r1
	dsb
	smc	#0
	ldmfd   sp!, {r2-r12, pc}
ENDPROC(omap_smc_rom)

/* Refer to Kernel "u32 omap_smc2(u32 id, u32 falg, u32 pargs)"
 * Low level common routine to manage secure
 * HAL APIs.
 * Function signature : u32 omap_smc2(u32 id, u32 falg, u32 pargs)
 * @id : Application ID of HAL APIs
 * @flag : Flag to indicate the criticality of operation
 * @pargs : Physical address of parameter list starting
 *	    with number of parametrs
 */
ENTRY(omap_smc2)
	stmfd   sp!, {r1-r12, lr}
	mov	r3, r2
	mov	r2, r1
	mov	r1, #0x0	@ Process ID
	mov	r6, #0xff
	mov	r12, #0x00	@ Secure Service ID
	mov	r7, #0
	mcr	p15, 0, r7, c7, c5, 6
	dsb
	dmb
	smc	#0
	ldmfd   sp!, {r1-r12, pc}
END(omap_smc2)
#endif

#endif /* CONFIG_SPL_BUILD */
