/*
 * (C) Copyright 2012
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

#ifndef _ICOM_PWM_H
#define _ICOM_PWM_H

#ifdef CONFIG_SYS_PWM
/**
 * id: PWM id
 * duty_cycle: 0% ~ 100%
 */
void pwm_set_duty(unsigned id, unsigned int duty_cycle);
#else
#define pwm_set_duty(id, duty_cycle) do {} while (0)
#endif /* CONFIG_SYS_PWM */

#endif /* _ICOM_PWM_H */
