/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/**
 * i.MX PWM LED driver
 * Copyright (C) 2020 unu GmbH <opensource@unumotors.com>
 * Written by Geoffrey Phillips.
 *
 * NOTE! This copyright does *not* cover user programs that use kernel
 *  services by normal system calls - this is merely considered normal use
 *  of the kernel, and does *not* fall under the heading of "derived work".
 *
 * This library is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PWM_LED_H
#define PWM_LED_H

#include <linux/ioctl.h>

/* clang-format off */
#define PWM_LED_IOC_TYPE   'u'
#define PWM_LED_CONFIGURE  _IO( PWM_LED_IOC_TYPE, 0x40        ) /* 0x00007540 */
#define PWM_LED_OPEN_FADE  _IO( PWM_LED_IOC_TYPE, 0x41        ) /* 0x00007541 */
#define PWM_LED_OPEN_CUE   _IO( PWM_LED_IOC_TYPE, 0x42        ) /* 0x00007542 */
#define PWM_LED_CLOSE_FADE _IO( PWM_LED_IOC_TYPE, 0x43        ) /* 0x00007543 */
#define PWM_LED_CLOSE_CUE  _IO( PWM_LED_IOC_TYPE, 0x44        ) /* 0x00007544 */
#define PWM_LED_PLAY_FADE  _IO( PWM_LED_IOC_TYPE, 0x45        ) /* 0x00007545 */
#define PWM_LED_PLAY_CUE   _IO( PWM_LED_IOC_TYPE, 0x46        ) /* 0x00007546 */
#define PWM_LED_STOP_FADE  _IO( PWM_LED_IOC_TYPE, 0x47        ) /* 0x00007547 */
#define PWM_LED_STOP_CUE   _IO( PWM_LED_IOC_TYPE, 0x48        ) /* 0x00007548 */
#define PWM_LED_SET_ACTIVE _IO( PWM_LED_IOC_TYPE, 0x49        ) /* 0x00007549 */
#define PWM_LED_SET_DUTY   _IO( PWM_LED_IOC_TYPE, 0x4A        ) /* 0x0000754A */
#define PWM_LED_GET_DUTY   _IOR(PWM_LED_IOC_TYPE, 0x4B, ushort) /* 0x8002754B */
#define PWM_LED_SET_ADAPT  _IO( PWM_LED_IOC_TYPE, 0x4C        ) /* 0x0000754C */

/** PWM_LED_CONFIGURE defines */
#define PWM_LED_CFG_BIT_PERIOD     0
#define PWM_LED_CFG_BIT_PRESCALER  16
#define PWM_LED_CFG_BIT_INVERT     28
#define PWM_LED_CFG_MASK_PERIOD    0x0000FFFF /* Range 2 to 65535 */
#define PWM_LED_CFG_MASK_PRESCALER 0x0FFF0000 /* N-1 value, range 0 to 4095 */
#define PWM_LED_CFG_MASK_INVERT    0x10000000

/** After a cue is opened using PWM_LED_OPEN_CUE, u32 'cue items' shall be
 *  written to the device in the following format. */
#define PWM_LED_CUE_ITEM_BIT_LED   0
#define PWM_LED_CUE_ITEM_BIT_TYPE  8
#define PWM_LED_CUE_ITEM_BIT_VAL   16
#define PWM_LED_CUE_ITEM_MASK_LED  0x0000001F
#define PWM_LED_CUE_ITEM_MASK_TYPE 0x00000F00 /* See PWM_LED_CUE_ITEM_TYPE_x */
#define PWM_LED_CUE_ITEM_MASK_VAL  0xFFFF0000
#define PWM_LED_CUE_ITEM_TYPE_FADE 0 /* Value=fade index */
#define PWM_LED_CUE_ITEM_TYPE_DUTY 1 /* Value=duty cycle */
/* clang-format on */

#endif /* PWM_LED_H */
