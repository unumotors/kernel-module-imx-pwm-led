; SPDX-License-Identifier: GPL-2.0-only
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; i.MX PWM LED driver
; Copyright (C) 2020 unu GmbH
; Written by Geoffrey Phillips.
;
; This library is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License version 2 as
; published by the Free Software Foundation.
;
; This library is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this library.  If not, see <http://www.gnu.org/licenses/>.
;
; Much of this code is adapted from the i.MX6 PWM audio driver from Glowforge:
; https://github.com/Glowforge/kernel-module-imx-pwm-audio/
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Register Allocation:
; r0
; r1 - $one         // Constant of one
; r2 - $duty_cycle  // Read by CPU
; r3 - $temp
; r4
; r5
; r6 - $active      // Written by CPU
; r7 - $end_address // Written by CPU

start:
    ldi r2, 0    ; $duty_cycle = 0;           // Start with zero duty
    ldi r1, 1    ; $one = 1;
    stf r1, 0xc8 ; EPITSR = $one;             // Clear the EPIT event

update:
    done 4       ;                            // Wait for the next EPIT event
    stf r1, 0xc8 ; EPITSR = $one;             // Clear the EPIT event
    ldf r3, 0x00 ; $temp = MSA;               // Get the current read address
    cmphs r3, r7 ; if ($temp >= $end_address) // If playing has finished:
    bt alldone   ;     goto alldone;          // Jump to alldone
    ldf r2, 0x0a ; $duty_cycle = *MSA++;      // Get the next duty cycle (u16)
    cmpeqi r6, 0 ; if ($active == 0)          // If inactive:
    bt inactive  ;     goto inactive;         // Jump to inactive
    stf r2, 0x2b ; PWMSAR = $duty             // Set the new PWM duty
    btsti r1, 0  ; if ($one & 0x01)           // If true: No relative jmp exists
    bt update    ;     goto update;           // Jump to update

inactive:
    stf r6, 0x2b ; PWMSAR = $active;          // Set the PWM duty to zero
    btsti r1, 0  ; if ($one & 0x01)           // If true:
    bt update    ;     goto update;           // Jump to update

alldone:
    done 3       ;                            // Generate SDMA interrupt
    btsti r1, 0  ; if ($one & 0x01)           // If true:
    bt update    ;     goto update;           // Jump to update
