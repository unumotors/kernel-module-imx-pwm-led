/* SPDX-License-Identifier: GPL-2.0-only */
/*******************************************************************************
 * i.MX PWM LED driver
 * Copyright (C) 2020 unu GmbH <opensource@unumotors.com>
 * Written by Geoffrey Phillips.
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
 *
 * Much of this code is adapted from the i.MX6 PWM audio driver from Glowforge:
 * https://github.com/Glowforge/kernel-module-imx-pwm-audio/
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/

#include "pwm_led.h"
#include <asm/uaccess.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_data/dma-imx-sdma.h>
#include <linux/platform_data/epit-imx.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>

// FIXME: Locking of fade buffers when playing

/*******************************************************************************
 * MACROS
 ******************************************************************************/

/* clang-format off */
#define DEVICE_PATH           "/dev/"
#define DEVICE_NAME           "pwm_led"
#define ARG_DUTY              2 /* SDMA output arg 'duty': register 2 */
#define ARG_ACTIVE            6 /* SDMA input arg 'active': register 6 */
#define ARG_END               7 /* SDMA input arg 'end': register 7 */
#define DEFAULT_ACTIVE        1 /* Default 'active' value: true */
#define DEFAULT_ADAPTIVE      0 /* Default 'adaptive' value: false */
#define MAX_PWM_PRESCALER     4095

/** PWM Control Register */
#define PWMCR                 0x00
/** PWM Status Register */
#define PWMSR                 0x04
/** PWM Interrupt Register */
#define PWMIR                 0x08
/** PWM Sample Register */
#define PWMSAR                0x0C
/** PWM Period Register */
#define PWMPR                 0x10
/** PWM Counter Register */
#define PWMCNR                0x14

#define PWMCR_FWM(x)          (((x) & 0x3) << 26)
#define PWMCR_DOZEEN          (1 << 24)
#define PWMCR_WAITEN          (1 << 23)
#define PWMCR_DBGEN           (1 << 22)
#define PWMCR_POUTC(x)        (((x) & 0x3) << 18)
#define PWMCR_CLKSRC_IPG_HIGH (2 << 16)
#define PWMCR_CLKSRC_IPG      (1 << 16)
#define PWMCR_PRESCALER(x)    (((x) & 0xFFF) << 4)
#define PWMCR_SWR             (1 << 3)
#define PWMCR_REPEAT_1        (0 << 1)
#define PWMCR_REPEAT_2        (1 << 1)
#define PWMCR_REPEAT_4        (2 << 1)
#define PWMCR_REPEAT_8        (3 << 1)
#define PWMCR_EN              (1 << 0)

#define PWMSR_FWE             (1 << 6)
#define PWMSR_CMP             (1 << 5)
#define PWMSR_ROV             (1 << 4)
#define PWMSR_FE              (1 << 3)
#define PWMSR_FIFOAV_MASK     0x7

#define PWMIR_CIE             (1 << 2)
#define PWMIR_RIE             (1 << 1)
#define PWMIR_FIE             (1 << 0)
/* clang-format on */

/** Helper macro to get the 'dev' and 'self' from a 'filp'.
 *  miscdevice sets filp's private_data pointer to itself */
#define DEV_SELF_FROM_FILP(filp)                                   \
	struct device *dev =                                       \
		((struct miscdevice *)filp->private_data)->parent; \
	struct imx_pwm_led *self = dev_get_drvdata(dev)

/*******************************************************************************
 * TYPES
 ******************************************************************************/

/* clang-format off */
/** Open object enum */
enum open_object_type {
	OPEN_OBJECT_TYPE_NONE,
	OPEN_OBJECT_TYPE_FADE,
	OPEN_OBJECT_TYPE_CUE
};

/** Fade direction enum */
enum fade_dir {
	FADE_DIR_UNKNOWN,
	FADE_DIR_INCREASING,
	FADE_DIR_DECREASING,
	FADE_DIR_NON_MONOTONIC
};
/* clang-format on */

/** LED device data struct */
struct imx_pwm_led {
	/** Pointer to parent device */
	struct device *dev;
	/** Lock to guard access */
	struct mutex lock;
	/** Peripheral clock */
	struct clk *clk_per;
	/** IPG clock */
	struct clk *clk_ipg;
	/** Base address of PWM registers */
	void __iomem *mmio_base;
	/** Physical address of PWM register base */
	u32 mmio_base_phys;
	/** Userspace interface */
	struct miscdevice led_dev;
	/** SDMA channel number */
	u32 sdma_ch_num;
	/** Pointer to the SDMA channel used by the driver */
	struct sdma_channel *sdmac;
	/** Device index */
	uint led_idx;
	/** Device name */
	char *dev_name;
	/** PID of owner */
	pid_t owner_pid;
	/** Type of currently open object */
	enum open_object_type open_object_type;
	/** Index of currently open object */
	uint open_object_idx;
	/** True if writing is ongoing */
	int writing;
	/** True if adaptive mode is enabled */
	int adaptive;
};

/** Fade struct */
struct led_fade {
	/** Buffer of u16 samples */
	void *buf;
	/** Physical address of buffer for the SDMA */
	dma_addr_t buf_phys;
	/** Length of fade data in **bytes** */
	uint len;
	/** Lock to guard access */
	struct mutex lock;
	/** PID of owner */
	pid_t owner_pid;
	/** Direction */
	enum fade_dir dir;
};

/** Cue item struct */
struct led_cue_item {
	/** Index of LED */
	u8 led_idx;
	/** Type, see PWM_LED_CUE_ITEM_TYPE_* */
	u8 type;
	/** Value */
	u16 val;
};

/** Cue struct */
struct led_cue {
	/** Buffer of cue items */
	struct led_cue_item *items;
	/** Length of item data in **bytes** */
	uint len;
	/** Lock to guard access */
	struct mutex lock;
	/** PID of owner */
	pid_t owner_pid;
};

/** Module global data struct */
struct glb_data {
	/** Array of leds */
	struct imx_pwm_led **leds;
	/** Array of fades */
	struct led_fade *fades;
	/** Array of cues */
	struct led_cue *cues;
	/** Buffer of duty cycles (max_cues x max_leds) */
	u16 *duty_buf;
	/** Physical address of duty buffer for the SDMA */
	dma_addr_t duty_buf_phys;
	/** Number of configured leds */
	uint num_leds;
	/** Pointer to the timer used to drive the DMA engine */
	struct epit *epit;
	/** Pointer to the SDMA engine */
	struct sdma_engine *sdma;
	/** SDMA script load address (SDMA address in words) */
	u32 sdma_script_origin;
};

/*******************************************************************************
 * VARIABLES & CONSTANTS
 ******************************************************************************/

/* clang-format off */
/** Module Parameters */
static u32 sample_rate      = 100; /* Fade sample rate in Hz */
static u32 pwm_period       = 24000; /* 24 MHz IPG clk / 24000 = 1 kHz */
static u32 pwm_prescaler    = 0; /* N-1 value, allowed range: 0 to 4095 */
static u32 pwm_invert       = 0; /* 0=non-inverted, 1=inverted */
static u32 pwm_phase_offset = 0; /* PWM channel phase offset in microseconds */
static u32 max_fades        = 16;
static u32 max_cues         = 16;
static u32 max_leds         = 8; /* imx6ul has 8 PWMs */
static u32 max_fade_size    = 4096; /* DMA memory block size is 4KB */
static u32 sdma_priority    = 5; /* Allowed range: 1 to 6 */
module_param(sample_rate,      uint, 0);
module_param(pwm_period,       uint, 0);
module_param(pwm_prescaler,    uint, 0);
module_param(pwm_invert,       uint, 0);
module_param(pwm_phase_offset, uint, 0);
module_param(max_fades,        uint, 0);
module_param(max_cues,         uint, 0);
module_param(max_leds,         uint, 0);
module_param(max_fade_size,    uint, 0);
module_param(sdma_priority,    uint, 0);
/* clang-format on */

/** Device global data, shared between all devices */
static struct glb_data glb_data;

/** SDMA script compiled from imx_pwm_led_sdma.asm using sdma_asm.pl from
 *  http://billauer.co.il/blog/2011/10/imx-sdma-howto-assembler-linux/ */
static const u32 sdma_script[] = {
	0x0a000901, 0x69c80400, 0x69c86300, 0x03df7d09, 0x620a4e00,
	0x7c036a2b, 0x01607df5, 0x6e2b0160, 0x7df20300, 0x01607def,
};

/*******************************************************************************
 * UTILITY FUNCTIONS
 ******************************************************************************/

static inline pid_t get_my_pid(void)
{
	return task_tgid_nr(current);
}

static inline int am_not_owner(pid_t owner_pid)
{
	return (owner_pid != 0) && (owner_pid != get_my_pid());
}

static inline uint get_max_cue_size(void)
{
	return max_leds * sizeof(struct led_cue_item);
}

static uint get_duty_buf_size(void)
{
	return sizeof(u16) * max_leds * (max_cues + 1);
}

static void lock_leds(uint led_mask)
{
	uint i;
	for (i = 0; i < glb_data.num_leds; i++) {
		if (led_mask & (1 << i)) {
			mutex_lock(&glb_data.leds[i]->lock);
		}
	}
}

static void unlock_leds(uint led_mask)
{
	uint i;
	for (i = 0; i < glb_data.num_leds; i++) {
		if (led_mask & (1 << i)) {
			mutex_unlock(&glb_data.leds[i]->lock);
		}
	}
}

static u32 convert_mask_led_to_sdma(uint led_mask)
{
	uint i;
	u32 sdma_mask = 0;
	for (i = 0; i < glb_data.num_leds; i++) {
		if (led_mask & (1 << i)) {
			sdma_mask |= 1 << glb_data.leds[i]->sdma_ch_num;
		}
	}
	return sdma_mask;
}

static u32 convert_mask_sdma_to_led(uint sdma_mask)
{
	uint i;
	u32 led_mask = 0;
	for (i = 0; i < glb_data.num_leds; i++) {
		if (sdma_mask & (1 << glb_data.leds[i]->sdma_ch_num)) {
			led_mask |= 1 << i;
		}
	}
	return led_mask;
}

/*******************************************************************************
 * PWM FUNCTIONS
 ******************************************************************************/

static int config_pwm(struct imx_pwm_led *self, u32 period, u32 prescaler,
		      u32 invert)
{
	u32 cr;
	uint clk;

	/* Enable clocks */
	int ret = 0;

	if ((ret = clk_prepare_enable(self->clk_per))) {
		dev_err(self->dev, "%s: failed to enable PER clock", __func__);
		return ret;
	}
	if ((ret = clk_prepare_enable(self->clk_ipg))) {
		dev_err(self->dev, "%s: failed to enable IPG clock", __func__);
		return ret;
	}

	cr = PWMCR_REPEAT_1 | PWMCR_DOZEEN | PWMCR_WAITEN | PWMCR_DBGEN |
	     PWMCR_CLKSRC_IPG_HIGH | PWMCR_PRESCALER(prescaler) |
	     PWMCR_POUTC(invert);

	/* Software reset */
	__raw_writel(PWMCR_SWR, self->mmio_base + PWMCR);
	udelay(10);
	/* Activate & de-activate PWM (seems to be necessary after a reset) */
	__raw_writel(PWMCR_EN, self->mmio_base + PWMCR);
	__raw_writel(0, self->mmio_base + PWMCR);

	/* Silence */
	__raw_writel(0, self->mmio_base + PWMSAR);

	clk = clk_get_rate(self->clk_ipg);
	clk /= prescaler + 1;
	dev_dbg(self->dev, "%s: setting PWMPR: 0x%08x (%u.%03u Hz)", __func__,
		period - 2, clk / period,
		(((clk % period) * 1000) + (period / 2)) / period);
	__raw_writel(period - 2, self->mmio_base + PWMPR);

	/* Configure */
	dev_dbg(self->dev, "%s: setting PWMCR: 0x%08x", __func__, cr);
	__raw_writel(cr, self->mmio_base + PWMCR);
	return 0;
}

static void enable_pwm(struct imx_pwm_led *self)
{
	u32 cr = __raw_readl(self->mmio_base + PWMCR);
	cr |= PWMCR_EN;
	__raw_writel(cr, self->mmio_base + PWMCR);
}

static void disable_pwm(struct imx_pwm_led *self)
{
	u32 cr = 0;
	dev_dbg(self->dev, "%s", __func__);
	/* Disable PWM */
	__raw_writel(cr, self->mmio_base + PWMCR);
	/* Release clocks */
	clk_disable_unprepare(self->clk_per);
	clk_disable_unprepare(self->clk_ipg);
}

static inline void set_pwm_duty(struct imx_pwm_led *self, uint duty)
{
	__raw_writel(duty, self->mmio_base + PWMSAR);
}

/*******************************************************************************
 * FADE / CUE FUNCTIONS
 ******************************************************************************/

static int open_fade(struct imx_pwm_led *self, uint fade_idx)
{
	int ret = 0;
	struct led_fade *fade;
	if (fade_idx >= max_fades) {
		dev_warn(self->dev, "%s: %u >= max_fades %u", __func__,
			 fade_idx, max_fades);
		return -EINVAL;
	}
	fade = &glb_data.fades[fade_idx];
	mutex_lock(&fade->lock);
	if (am_not_owner(fade->owner_pid)) {
		dev_warn(self->dev, "%s: fade %u is owned by pid %d", __func__,
			 fade_idx, fade->owner_pid);
		ret = -EPERM;
		goto open_fade_exit;
	}
	fade->owner_pid = get_my_pid();
open_fade_exit:
	mutex_unlock(&fade->lock);
	return ret;
}

static int open_cue(struct imx_pwm_led *self, uint cue_idx)
{
	int ret = 0;
	struct led_cue *cue;
	if (cue_idx >= max_cues) {
		dev_warn(self->dev, "%s: %u >= max_cues %u", __func__, cue_idx,
			 max_cues);
		return -EINVAL;
	}
	cue = &glb_data.cues[cue_idx];
	mutex_lock(&cue->lock);
	if (am_not_owner(cue->owner_pid)) {
		dev_warn(self->dev, "%s: cue %u is owned by pid %d", __func__,
			 cue_idx, cue->owner_pid);
		ret = -EPERM;
		goto open_cue_exit;
	}
	cue->owner_pid = get_my_pid();
open_cue_exit:
	mutex_unlock(&cue->lock);
	return ret;
}

static int close_fade(struct imx_pwm_led *self, uint fade_idx)
{
	int ret = 0;
	struct led_fade *fade;
	if (fade_idx >= max_fades) {
		dev_warn(self->dev, "%s: %u >= max_fades %u", __func__,
			 fade_idx, max_fades);
		return -EINVAL;
	}
	fade = &glb_data.fades[fade_idx];
	mutex_lock(&fade->lock);
	if (am_not_owner(fade->owner_pid)) {
		dev_warn(self->dev, "%s: fade %u is owned by pid %d", __func__,
			 fade_idx, fade->owner_pid);
		ret = -EPERM;
		goto close_fade_exit;
	}
	fade->owner_pid = 0;
close_fade_exit:
	mutex_unlock(&fade->lock);
	return ret;
}

static int close_cue(struct imx_pwm_led *self, uint cue_idx)
{
	int ret = 0;
	struct led_cue *cue;
	if (cue_idx >= max_cues) {
		dev_warn(self->dev, "%s: %u >= max_cues %u", __func__, cue_idx,
			 max_cues);
		return -EINVAL;
	}
	cue = &glb_data.cues[cue_idx];
	mutex_lock(&cue->lock);
	if (am_not_owner(cue->owner_pid)) {
		dev_warn(self->dev, "%s: cue %u is owned by pid %d", __func__,
			 cue_idx, cue->owner_pid);
		ret = -EPERM;
		goto close_cue_exit;
	}
	cue->owner_pid = 0;
close_cue_exit:
	mutex_unlock(&cue->lock);
	return ret;
}

static int get_cue_led_mask(struct imx_pwm_led *self, uint cue_idx,
			    uint *led_mask)
{
	int ret;
	uint i;
	uint num_items;
	const struct led_cue *cue;

	if ((ret = open_cue(self, cue_idx))) {
		return ret;
	}
	cue = &glb_data.cues[cue_idx];

	if (cue->len == 0) {
		dev_warn(self->dev, "%s: cue %u: len == 0", __func__, cue_idx);
		return -ENODATA;
	}

	if ((cue->len % sizeof(struct led_cue_item)) != 0) {
		dev_warn(self->dev, "%s: cue %u: len %u %% %u != 0", __func__,
			 cue_idx, cue->len, sizeof(struct led_cue_item));
		return -EFAULT;
	}

	*led_mask = 0;
	num_items = cue->len / sizeof(struct led_cue_item);
	for (i = 0; i < num_items; i++) {
		const struct led_cue_item *item = &cue->items[i];
		if (item->led_idx >= glb_data.num_leds) {
			dev_warn(self->dev,
				 "%s: cue %u: item %u: "
				 "led_idx %u >= num_leds %u",
				 __func__, cue_idx, i, item->led_idx,
				 glb_data.num_leds);
			return -EFAULT;
		}
		*led_mask |= 1 << item->led_idx;
		item++;
	}
	return 0;
}

static int start_fade(struct imx_pwm_led *self)
{
	int old_enable;
	if (am_not_owner(self->owner_pid)) {
		dev_warn(self->dev, "%s: led %u is owned by pid %d", __func__,
			 self->led_idx, self->owner_pid);
		return -EPERM;
	}
	/* Enable the sdma channel */
	old_enable =
		sdma_event_enable(self->sdmac, epit_sdma_event(glb_data.epit));
	/* Set a nonzero priority to start the script */
	sdma_set_channel_priority(self->sdmac, sdma_priority);
	sdma_set_channel_pending(self->sdmac);
	/* If the channel was already enabled: */
	if (old_enable) {
		return 0;
	}
	dev_dbg(self->dev, "%s", __func__);
	return 0;
}

static int start_cue(struct imx_pwm_led *self, uint led_mask)
{
	uint i;
	u32 old_sdma_mask, sdma_mask, old_led_mask, started_led_mask;

	for (i = 0; i < glb_data.num_leds; i++) {
		if ((led_mask & (1 << i)) &&
		    am_not_owner(glb_data.leds[i]->owner_pid)) {
			dev_warn(self->dev, "%s: led %u is owned by pid %d",
				 __func__, i, glb_data.leds[i]->owner_pid);
			return -EPERM;
		}
	}

	sdma_mask = convert_mask_led_to_sdma(led_mask);

	/* Enable the sdma channels: */
	old_sdma_mask = sdma_event_enable_by_channel_mask(
		glb_data.sdma, sdma_mask, epit_sdma_event(glb_data.epit));
	// FIXME: It would be better to already have the priority enabled, but this doesn't work as the SDMA already starts running
	//        This must mean the EP[i] flag is already set before the EPIT event is even enabled. Is there a way to clear the
	//        EP[i] flags? The set pending sets the flags - is it one of the registers?
	for (i = 0; i < glb_data.num_leds; i++) {
		if (led_mask & (1 << i)) {
			/* Set a nonzero priority to start the script */
			sdma_set_channel_priority(glb_data.leds[i]->sdmac,
						  sdma_priority);
		}
	}
	sdma_set_channel_pending_by_mask(glb_data.sdma, sdma_mask);

	old_led_mask = convert_mask_sdma_to_led(old_sdma_mask);

	started_led_mask = (old_led_mask ^ led_mask) & led_mask;
	/* If all the channels were already enabled: */
	if (started_led_mask == 0) {
		return 0;
	}
	dev_dbg(self->dev, "%s: 0x%08x, started: 0x%08x", __func__, led_mask,
		started_led_mask);
	return 0;
}

static int stop_led(struct imx_pwm_led *self)
{
	/* Disable the sdma channel */
	int old_enable =
		sdma_event_disable(self->sdmac, epit_sdma_event(glb_data.epit));
	/* If the channel was already disabled: */
	if (!old_enable) {
		return 0;
	}
	dev_dbg(self->dev, "%s", __func__);
	return 0;
}

static int stop_cue(struct imx_pwm_led *self, uint led_mask)
{
	uint i;
	u32 old_sdma_mask, sdma_mask, old_led_mask, stopped_led_mask;

	for (i = 0; i < glb_data.num_leds; i++) {
		if ((led_mask & (1 << i)) &&
		    am_not_owner(glb_data.leds[i]->owner_pid)) {
			dev_warn(self->dev, "%s: led %u is owned by pid %d",
				 __func__, i, glb_data.leds[i]->owner_pid);
			return -EPERM;
		}
	}

	sdma_mask = convert_mask_led_to_sdma(led_mask);

	/* Disable the sdma channels */
	old_sdma_mask = sdma_event_disable_by_channel_mask(
		glb_data.sdma, sdma_mask, epit_sdma_event(glb_data.epit));

	old_led_mask = convert_mask_sdma_to_led(old_sdma_mask);

	stopped_led_mask = (old_led_mask ^ ~led_mask) & led_mask;

	/* If all the channels were already disabled: */
	if (stopped_led_mask == 0) {
		return 0;
	}
	dev_dbg(self->dev, "%s: 0x%08x, stopped: 0x%08x", __func__, led_mask,
		stopped_led_mask);
	return 0;
}

static void find_fade_dir(struct imx_pwm_led *self, struct led_fade *fade,
			  uint size)
{
	const u16 *itr, *end;
	if (fade->dir == FADE_DIR_NON_MONOTONIC) {
		return;
	}
	itr = &((const u16 *)fade->buf)[fade->len / sizeof(u16)];
	end = &((const u16 *)fade->buf)[(fade->len + size) / sizeof(u16)];
	if ((fade->len / sizeof(u16)) == 0) {
		itr++;
	}
	while (itr < end) {
		uint val = *itr;
		uint last_val = *(itr - 1);
		if (val > last_val) {
			if (fade->dir == FADE_DIR_UNKNOWN) {
				dev_dbg(self->dev, "%s: inc", __func__);
				fade->dir = FADE_DIR_INCREASING;
			} else if (fade->dir != FADE_DIR_INCREASING) {
				dev_dbg(self->dev,
					"%s: non-monotonic at idx %u", __func__,
					itr - (const u16 *)fade->buf);
				fade->dir = FADE_DIR_NON_MONOTONIC;
				break;
			} else {
				/* Still increasing */
			}
		} else if (val < last_val) {
			if (fade->dir == FADE_DIR_UNKNOWN) {
				dev_dbg(self->dev, "%s: dec", __func__);
				fade->dir = FADE_DIR_DECREASING;
			} else if (fade->dir != FADE_DIR_DECREASING) {
				dev_dbg(self->dev,
					"%s: non-monotonic at idx %u", __func__,
					itr - (const u16 *)fade->buf);
				fade->dir = FADE_DIR_NON_MONOTONIC;
				break;
			} else {
				/* Still decreasing */
			}
		} else {
			/* Same */
		}
		itr++;
	}
}

static uint adapt_fade(struct imx_pwm_led *self, const struct led_fade *fade,
		       uint current_duty)
{
	uint len = fade->len / sizeof(u16);
	uint l = 0;
	uint r = len;
	if (len == 0) {
		dev_warn(self->dev, "%s: no data", __func__);
		return 0;
	}
	/* If the direction is unknown or non-monotonic: */
	if ((fade->dir != FADE_DIR_INCREASING) &&
	    (fade->dir != FADE_DIR_DECREASING)) {
		dev_warn(self->dev, "%s: non-monotonic", __func__);
		return 0;
	}
	/* Binary search for leftmost value with the current duty: */
	while (l < r) {
		uint m = (l + r) / 2;
		uint val = ((const u16 *)fade->buf)[m];
		if ((fade->dir == FADE_DIR_INCREASING) ? (val < current_duty) :
							 (val > current_duty)) {
			l = m + 1;
		} else {
			r = m;
		}
	}
	/* If the current duty is greater than (for increasing), or less than
	 * (for decreasing), all of the fade values: */
	if (l >= len) {
		/* Use the last fade value: */
		l = len - 1;
	}
	/* If an adaptation is being made: */
	if (l != 0) {
		dev_dbg(self->dev, "%s: dir: %s, idx: %u, duty: %u", __func__,
			(fade->dir == FADE_DIR_INCREASING) ? "inc" : "dec",
			current_duty, l, ((const u16 *)fade->buf)[l]);
	}
	/* Return byte offset: */
	return l * sizeof(u16);
}

/*******************************************************************************
 * SDMA FUNCTIONS
 ******************************************************************************/

static int load_sdma_script(struct device *dev)
{
	int ret;
	const u32 *script = sdma_script;
	size_t script_len = sizeof(sdma_script);
	struct sdma_context_data initial_context = { { 0 } };

	/* write the script code to SDMA RAM */
	dev_dbg(dev, "%s: %d bytes", __func__, script_len);
	ret = sdma_write_datamem(glb_data.sdma, (void *)script, script_len,
				 glb_data.sdma_script_origin);
	if (ret) {
		dev_err(dev, "%s: failed to write data", __func__);
		return ret;
	}
	return 0;
}

/** Sets script program counter and initial arguments */
static int load_sdma_initial_context(struct imx_pwm_led *self)
{
	int ret;
	struct sdma_context_data context = { { 0 } };

	/* set the script arguments and initial PC;
	 * see the specific asm file for argument requirements;
	 * (in program space addressing - halfwords) */
	context.channel_state.pc = glb_data.sdma_script_origin * 2;
	context.gReg[ARG_ACTIVE] = DEFAULT_ACTIVE;
	context.mda = self->mmio_base_phys + PWMSAR;
	/* destination address frozen;
	 * source address postincrement;
	 * start in read mode */
	context.ms = 0x00100000;
	context.pda = epit_status_register_address(glb_data.epit);
	/* destination address frozen;
	 * 32-bit write size; start in write mode */
	context.ps = 0x000c0400;

	/* acquire the channel and load its context;
	 * it's triggered externally by the EPIT */
	sdma_setup_channel(self->sdmac, true);

	ret = sdma_load_partial_context(self->sdmac, &context, 0,
					sizeof(context));
	if (ret) {
		dev_err(self->dev, "%s: failed to load context", __func__);
		return ret;
	}
	return 0;
}

static int load_sdma_duty(struct imx_pwm_led *self, uint cue_idx, uint led_idx,
			  uint duty)
{
	int ret;
	struct sdma_context_data context = { { 0 } };
	uint duty_offset = (cue_idx * max_leds) + led_idx;
	dma_addr_t duty_phys =
		glb_data.duty_buf_phys + (duty_offset * sizeof(u16));
	dev_dbg(self->dev, "%s: led: %u, duty: %u", __func__, led_idx, duty);
	glb_data.duty_buf[duty_offset] = duty;

	/* Write the ARG_END and MSA; as well as the MDA,
	 * since it's in the middle */
	context.gReg[ARG_END] = duty_phys + sizeof(u16);
	context.mda = self->mmio_base_phys + PWMSAR;
	context.msa = duty_phys;

	/* This operation is run by channel 0, which is mutually exclusive with
	 * our scripts's channel. Since only one channel is running at a time,
	 * and channels cannot preempt each other, this ensures that the context
	 * is only updated between iterations of our script. (i.e. registers
	 * won't get suddenly updated while our script is running) */
	ret = sdma_load_partial_context(
		self->sdmac, (struct sdma_context_data *)&context.gReg[ARG_END],
		offsetof(struct sdma_context_data, gReg[ARG_END]),
		12); /* 3 * sizeof(u32) = 12 bytes */
	if (ret) {
		dev_err(self->dev, "%s: failed to load context", __func__);
		return ret;
	}
	return 0;
}

static int fetch_sdma_duty(struct imx_pwm_led *self, uint *duty)
{
	int ret = sdma_fetch_partial_context(self->sdmac, duty,
					     offsetof(struct sdma_context_data,
						      gReg[ARG_DUTY]),
					     4); /* sizeof(u32) = 4 bytes */
	if (ret) {
		dev_err(self->dev, "%s: failed to fetch context", __func__);
		return ret;
	}
	dev_dbg(self->dev, "%s: %u", __func__, *duty);
	return 0;
}

static int load_sdma_fade(struct imx_pwm_led *self, uint fade_idx)
{
	int ret;
	const struct led_fade *fade;
	struct sdma_context_data context = { { 0 } };
	u32 offset = 0;
	dev_dbg(self->dev, "%s: led: %u, fade: %u", __func__, self->led_idx,
		fade_idx);

	if ((ret = open_fade(self, fade_idx))) {
		return ret;
	}
	fade = &glb_data.fades[fade_idx];

	if (fade->len == 0) {
		dev_warn(self->dev, "%s: fade %u: len %u < %u", __func__,
			 fade_idx, fade->len, sizeof(u16));
		return -ENODATA;
	}

	if ((fade->len % sizeof(u16)) != 0) {
		dev_warn(self->dev, "%s: fade %u: len %u %% %u != 0", __func__,
			 fade_idx, fade->len, sizeof(u16));
		return -EFAULT;
	}

	/* If the adaptive behaviour is enabled (starting a new fade while an
	 * existing one is running results in the new fade starting from the
	 * nearest duty cycle to the current one): */
	if (self->adaptive) {
		int ret;
		uint duty;
		/* Fetch the current duty cycle from the SDMA, in case the LED
		 * is inactive, in which case the actual PWM duty will be 0%: */
		if ((ret = fetch_sdma_duty(self, &duty))) {
			return ret;
		}
		/* Adapt the starting offset of the fade: */
		offset = adapt_fade(self, fade, duty);
	}

	/* Write the ARG_END and MSA; as well as the MDA,
	 * since it's in the middle */
	context.gReg[ARG_END] = fade->buf_phys + fade->len;
	context.mda = self->mmio_base_phys + PWMSAR;
	context.msa = fade->buf_phys + offset;

	ret = sdma_load_partial_context(
		self->sdmac, (struct sdma_context_data *)&context.gReg[ARG_END],
		offsetof(struct sdma_context_data, gReg[ARG_END]),
		12); /* 3 * sizeof(u32) = 12 bytes */
	if (ret) {
		dev_err(self->dev, "%s: failed to load context", __func__);
		return ret;
	}
	return 0;
}

static int load_sdma_cue(struct imx_pwm_led *self, uint cue_idx)
{
	int ret;
	uint i;
	uint num_items;
	const struct led_cue *cue;
	dev_dbg(self->dev, "%s: %u", __func__, cue_idx);

	if ((ret = open_cue(self, cue_idx))) {
		return ret;
	}
	cue = &glb_data.cues[cue_idx];

	if (cue->len == 0) {
		dev_warn(self->dev, "%s: cue %u: len == 0", __func__, cue_idx);
		return -ENODATA;
	}

	if ((cue->len % sizeof(struct led_cue_item)) != 0) {
		dev_warn(self->dev, "%s: cue %u: len %u %% %u != 0", __func__,
			 cue_idx, cue->len, sizeof(struct led_cue_item));
		return -EFAULT;
	}

	/* Load all of the items listed in the cue: */
	num_items = cue->len / sizeof(struct led_cue_item);
	for (i = 0; i < num_items; i++) {
		const struct led_cue_item *item = &cue->items[i];
		int ret;
		uint led_idx = item->led_idx;
		uint type = item->type & 0x0F;
		struct imx_pwm_led *led;
		if (led_idx >= glb_data.num_leds) {
			dev_warn(self->dev,
				 "%s: cue %u: item %u: "
				 "led_idx %u >= num_leds %u",
				 __func__, cue_idx, i, led_idx,
				 glb_data.num_leds);
			return -EFAULT;
		}
		led = glb_data.leds[led_idx];
		if (am_not_owner(led->owner_pid)) {
			dev_warn(self->dev, "%s: led %u is owned by pid %d",
				 __func__, led_idx, led->owner_pid);
			return -EPERM;
		}
		switch (type) {
		case PWM_LED_CUE_ITEM_TYPE_DUTY: {
			if ((ret = load_sdma_duty(led, cue_idx, led_idx,
						  item->val))) {
				return ret;
			}
			break;
		}
		case PWM_LED_CUE_ITEM_TYPE_FADE:
			if ((ret = load_sdma_fade(led, item->val))) {
				return ret;
			}
			break;
		default:
			dev_warn(self->dev,
				 "%s: cue %u: item %u: unknown type %u",
				 __func__, cue_idx, i, type);
			return -EINVAL;
		}
		item++;
	}
	return 0;
}

static int set_sdma_active(struct imx_pwm_led *self, int active)
{
	int ret;
	struct sdma_context_data context = { { 0 } };
	dev_dbg(self->dev, "%s: %d", __func__, active);

	context.gReg[ARG_ACTIVE] = active;

	ret = sdma_load_partial_context(
		self->sdmac,
		(struct sdma_context_data *)&context.gReg[ARG_ACTIVE],
		offsetof(struct sdma_context_data, gReg[ARG_ACTIVE]),
		4); /* 1 * sizeof(u32) = 4 bytes */
	if (ret) {
		dev_err(self->dev, "%s: failed to load context", __func__);
	}
	return 0;
}

static int is_sdma_playing(struct imx_pwm_led *self)
{
	struct sdma_context_data context = { { 0 } };
	/* Fetch the current MSA and end addresses: */
	int ret = sdma_fetch_partial_context(
		self->sdmac, &context.gReg[ARG_END],
		offsetof(struct sdma_context_data, gReg[ARG_END]),
		12); /* 3 * sizeof(u32) = 12 bytes */
	if (ret) {
		dev_err(self->dev, "%s: failed to fetch context", __func__);
	}
	ret = context.msa < context.gReg[ARG_END];
	dev_dbg(self->dev, "%s: %d", __func__, ret);
	return ret;
}

/**
 * Called after the SDMA engine executes a "done 3" instruction, setting the
 * interrupt flag for our channel.
 * This callback executes in tasklet context.
 */
static void imx_pwm_led_sdma_callback(void *param)
{
	struct imx_pwm_led *self = (struct imx_pwm_led *)param;
	(void)stop_led(self);
}

/*******************************************************************************
 * USERSPACE API
 ******************************************************************************/

static int imx_pwm_led_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	pid_t pid = get_my_pid();
	DEV_SELF_FROM_FILP(filp);
	mutex_lock(&self->lock);
	/* Only one process may open the device at a time. */
	if (self->owner_pid != 0) {
		dev_warn(dev, "open: " DEVICE_PATH "%s is owned by pid %d",
			 self->dev_name, self->owner_pid);
		ret = -EBUSY;
		goto imx_pwm_led_dev_open_exit;
	}
	dev_dbg(dev, "open: " DEVICE_PATH "%s by pid %d", self->dev_name, pid);
	self->owner_pid = pid;
	self->writing = 0;
imx_pwm_led_dev_open_exit:
	mutex_unlock(&self->lock);
	return ret;
}

static int imx_pwm_led_dev_close(struct inode *inode, struct file *filp)
{
	uint i;
	DEV_SELF_FROM_FILP(filp);
	mutex_lock(&self->lock);
	dev_dbg(dev, "close: " DEVICE_PATH "%s", self->dev_name);
	/* Automatically close any fades and cues that were opened: */
	for (i = 0; i < max_fades; i++) {
		if (glb_data.fades[i].owner_pid == self->owner_pid) {
			close_fade(self, i);
		}
	}
	for (i = 0; i < max_cues; i++) {
		if (glb_data.cues[i].owner_pid == self->owner_pid) {
			close_cue(self, i);
		}
	}
	self->owner_pid = 0;
	mutex_unlock(&self->lock);
	return 0;
}

static ssize_t imx_pwm_led_dev_read(struct file *filp, char __user *data,
				    size_t count, loff_t *offp)
{
	/* Reads are not supported */
	return 0;
}

static ssize_t imx_pwm_led_dev_write(struct file *filp, const char __user *data,
				     size_t count, loff_t *offp)
{
	size_t bytes_free = 0;
	ssize_t nbytes = 0;
	DEV_SELF_FROM_FILP(filp);
	switch (self->open_object_type) {
	case OPEN_OBJECT_TYPE_CUE: {
		int ret;
		struct led_cue *cue;
		if ((ret = open_cue(self, self->open_object_idx))) {
			return ret;
		}
		cue = &glb_data.cues[self->open_object_idx];
		if (!self->writing) {
			self->writing = 1;
			cue->len = 0;
		}
		if (cue->len >= get_max_cue_size()) {
			dev_warn(dev, "write: cue %u: buffer full",
				 self->open_object_idx);
			return -ENOSPC;
		}
		bytes_free = get_max_cue_size() - cue->len;
		nbytes = min(count, bytes_free);
		if (copy_from_user(((void *)cue->items) + cue->len, data,
				   nbytes)) {
			dev_err(dev, "write: failed userspace copy");
			return -EFAULT;
		}
		cue->len += nbytes;
		dev_dbg(dev, "write: cue %u: %u bytes, total len %u bytes",
			self->open_object_idx, nbytes, cue->len);
		break;
	}
	case OPEN_OBJECT_TYPE_FADE: {
		int ret;
		struct led_fade *fade;
		if ((ret = open_fade(self, self->open_object_idx))) {
			return ret;
		}
		fade = &glb_data.fades[self->open_object_idx];
		if (!self->writing) {
			self->writing = 1;
			fade->len = 0;
			fade->dir = FADE_DIR_UNKNOWN;
		}
		if (fade->len >= max_fade_size) {
			dev_warn(dev, "write: fade %u: buffer full",
				 self->open_object_idx);
			return -ENOSPC;
		}
		bytes_free = max_fade_size - fade->len;
		nbytes = min(count, bytes_free);
		if (copy_from_user(((void *)fade->buf) + fade->len, data,
				   nbytes)) {
			dev_err(dev, "write: failed userspace copy");
			return -EFAULT;
		}
		find_fade_dir(self, fade, nbytes);
		fade->len += nbytes;
		dev_dbg(dev, "write: fade %u: %u bytes, total len %u bytes",
			self->open_object_idx, nbytes, fade->len);
		break;
	}
	case OPEN_OBJECT_TYPE_NONE:
	default:
		dev_warn(self->dev, "write: no open object");
		return -EPERM;
		break;
	}
	return nbytes;
}

static int imx_pwm_led_dev_fsync(struct file *filp, loff_t start, loff_t end,
				 int datasync)
{
	return 0;
}

static loff_t imx_pwm_led_dev_llseek(struct file *filp, loff_t off, int whence)
{
	DEV_SELF_FROM_FILP(filp);
	/* An lseek() to 0 can be used to clear the buffer. */
	if (off != 0 || whence != SEEK_SET) {
		return -EINVAL;
	}
	switch (self->open_object_type) {
	case OPEN_OBJECT_TYPE_CUE: {
		int ret;
		if ((ret = open_cue(self, self->open_object_idx))) {
			return ret;
		}
		dev_dbg(dev, "seek: cue %u", self->open_object_idx);
		glb_data.cues[self->open_object_idx].len = 0;
		break;
	}
	case OPEN_OBJECT_TYPE_FADE: {
		int ret;
		if ((ret = open_fade(self, self->open_object_idx))) {
			return ret;
		}
		dev_dbg(dev, "seek: fade %u", self->open_object_idx);
		glb_data.fades[self->open_object_idx].len = 0;
		break;
	}
	case OPEN_OBJECT_TYPE_NONE:
	default:
		dev_warn(self->dev, "seek: no open object");
		return -EPERM;
		break;
	}
	return 0;
}

static long ioctl_open_fade(struct imx_pwm_led *self, unsigned long arg)
{
	int ret;
	dev_dbg(self->dev, "ioctl: OPEN_FADE: %lu", arg);
	if ((ret = open_fade(self, arg))) {
		return ret;
	}
	self->open_object_type = OPEN_OBJECT_TYPE_FADE;
	self->open_object_idx = arg;
	return 0;
}

static long ioctl_open_cue(struct imx_pwm_led *self, unsigned long arg)
{
	int ret;
	dev_dbg(self->dev, "ioctl: OPEN_CUE: %lu", arg);
	if ((ret = open_cue(self, arg))) {
		return ret;
	}
	self->open_object_type = OPEN_OBJECT_TYPE_CUE;
	self->open_object_idx = arg;
	return 0;
}

static long ioctl_close_fade(struct imx_pwm_led *self, unsigned long arg)
{
	int ret;
	dev_dbg(self->dev, "ioctl: CLOSE_FADE: %lu", arg);
	if ((ret = close_fade(self, arg))) {
		return ret;
	}
	self->open_object_type = OPEN_OBJECT_TYPE_NONE;
	return 0;
}

static long ioctl_close_cue(struct imx_pwm_led *self, unsigned long arg)
{
	int ret;
	dev_dbg(self->dev, "ioctl: CLOSE_CUE: %lu", arg);
	if ((ret = close_cue(self, arg))) {
		return ret;
	}
	self->open_object_type = OPEN_OBJECT_TYPE_NONE;
	return 0;
}

static long ioctl_configure(struct imx_pwm_led *self, unsigned long arg)
{
	long ret;
	uint period = arg & PWM_LED_CFG_MASK_PERIOD;
	uint prescaler = (arg & PWM_LED_CFG_MASK_PRESCALER) >>
			 PWM_LED_CFG_BIT_PRESCALER;
	uint invert = (arg & PWM_LED_CFG_MASK_INVERT) >> PWM_LED_CFG_BIT_INVERT;
	dev_dbg(self->dev,
		"ioctl: CONFIGURE: period: %u, prescaler: %u, invert: %u",
		period, prescaler, invert);
	if ((period < 2) || (period > __UINT16_MAX__)) {
		dev_warn(self->dev, "ioctl: CONFIGURE: invalid period %u",
			 period);
		return -EINVAL;
	}
	if (prescaler > MAX_PWM_PRESCALER) {
		dev_warn(self->dev, "ioctl: CONFIGURE: invalid prescaler %u",
			 prescaler);
		return -EINVAL;
	}

	if ((ret = stop_led(self))) {
		return ret;
	}
	disable_pwm(self);
	if ((ret = config_pwm(self, period, prescaler, invert))) {
		return ret;
	}
	enable_pwm(self);
	return 0;
}

static long ioctl_set_duty(struct imx_pwm_led *self, unsigned long arg)
{
	long ret;
	dev_dbg(self->dev, "ioctl: SET_DUTY: %lu", arg);
	if (arg > __UINT16_MAX__) {
		dev_warn(self->dev, "ioctl: SET_DUTY: invalid duty %lu", arg);
		return -EINVAL;
	}

	if ((ret = stop_led(self))) {
		return ret;
	}
	/* Use the SDMA to load the duty rather than setting it directly.
	 * This will take into account whether the LED is active and allow
	 * adaptation of a following fade, even when inactive: */
	if ((ret = load_sdma_duty(self, max_cues, self->led_idx, arg))) {
		return ret;
	}
	return 0;
}

static long ioctl_get_duty(struct imx_pwm_led *self, unsigned long arg)
{
	long ret;
	uint duty;
	/* Fetch the current duty cycle from the SDMA, in case the LED is
	 * inactive, in which case the actual PWM duty will be 0%: */
	if ((ret = fetch_sdma_duty(self, &duty))) {
		return ret;
	}
	dev_dbg(self->dev, "ioctl: GET_DUTY: %u", duty);
	if ((ret = copy_to_user((void *)arg, &duty, sizeof(duty)))) {
		dev_warn(self->dev, "ioctl: failed userspace copy");
		return ret;
	}
	return 0;
}

static long ioctl_set_active(struct imx_pwm_led *self, unsigned long arg)
{
	uint duty;
	int ret;
	dev_dbg(self->dev, "ioctl: SET_ACTIVE: %lu", arg);
	if ((ret = set_sdma_active(self, arg != 0))) {
		return ret;
	}
	/* Fetch the playing status: */
	ret = is_sdma_playing(self);
	/* If there was an error getting the status: */
	if (ret < 0) {
		return ret;
	}
	/* If the LED is currently playing: */
	if (ret) {
		/* The SDMA will set the actual duty: */
		return 0;
	}

	/* Fetch the current duty: */
	if ((ret = fetch_sdma_duty(self, &duty))) {
		return ret;
	}
	/* Set the PWM duty to the current duty if active, otherwise zero: */
	set_pwm_duty(self, arg ? duty : 0);
	return 0;
}

static long ioctl_set_adapt(struct imx_pwm_led *self, unsigned long arg)
{
	dev_dbg(self->dev, "ioctl: SET_ADAPT: %lu", arg);
	self->adaptive = arg != 0;
	return 0;
}

static long ioctl_play_fade(struct imx_pwm_led *self, unsigned long arg)
{
	int ret;
	dev_dbg(self->dev, "ioctl: PLAY_FADE: %lu", arg);
	/* 1. Stop our LED (it was already locked) */
	if ((ret = stop_led(self))) {
		return ret;
	}
	/* 2. Load the fade */
	if ((ret = load_sdma_fade(self, arg))) {
		return ret;
	}
	/* 3. Start our LED (it will be unlocked later) */
	if ((ret = start_fade(self))) {
		return ret;
	}
	return 0;
}

static long ioctl_play_cue(struct imx_pwm_led *self, unsigned long arg)
{
	int ret;
	uint led_mask;
	dev_dbg(self->dev, "ioctl: PLAY_CUE: %lu", arg);
	/* 1. Get the leds in the cue */
	if ((ret = get_cue_led_mask(self, arg, &led_mask))) {
		return ret;
	}
	/* 2. Lock the leds except our own, which was already locked */
	lock_leds(led_mask & ~(1 << self->led_idx));
	/* 3. Stop the leds */
	if ((ret = stop_cue(self, led_mask))) {
		goto ioctl_play_cue_exit;
	}
	/* 4. Load the cue */
	if ((ret = load_sdma_cue(self, arg))) {
		goto ioctl_play_cue_exit;
	}
	/* 5. Start the leds */
	ret = start_cue(self, led_mask);
ioctl_play_cue_exit:
	/* 6. Unlock the leds except our own, which will be unlocked later */
	unlock_leds(led_mask & ~(1 << self->led_idx));
	return ret;
}

static long ioctl_stop_fade(struct imx_pwm_led *self, unsigned long arg)
{
	int ret;
	(void)arg;
	dev_dbg(self->dev, "ioctl: STOP_FADE");
	/* 1. Stop our LED (it was already locked) */
	if ((ret = stop_led(self))) {
		return ret;
	}
	return 0;
}

static long ioctl_stop_cue(struct imx_pwm_led *self, unsigned long arg)
{
	int ret;
	uint led_mask;
	dev_dbg(self->dev, "ioctl: STOP_CUE: %lu", arg);
	/* 1. Get the leds in the cue */
	if ((ret = get_cue_led_mask(self, arg, &led_mask))) {
		return ret;
	}
	/* 2. Lock the leds except our own, which was already locked */
	lock_leds(led_mask & ~(1 << self->led_idx));
	/* 3. Stop the leds */
	if ((ret = stop_cue(self, led_mask))) {
		goto ioctl_play_cue_exit;
	}
ioctl_play_cue_exit:
	/* 4. Unlock the leds except our own, which will be unlocked later */
	unlock_leds(led_mask & ~(1 << self->led_idx));
	return ret;
}

static long imx_pwm_led_dev_ioctl(struct file *filp, unsigned int cmd,
				  unsigned long arg)
{
	int ret;
	DEV_SELF_FROM_FILP(filp);
	/* Ignore unsupported ioctl types, which can be sent by the terminal: */
	if (_IOC_TYPE(cmd) != PWM_LED_IOC_TYPE) {
		return EPERM;
	}
	mutex_lock(&self->lock);
	switch (cmd) {
	/* clang-format off */
	case PWM_LED_CONFIGURE:  ret = ioctl_configure (self, arg); break;
	case PWM_LED_OPEN_FADE:  ret = ioctl_open_fade (self, arg); break;
	case PWM_LED_OPEN_CUE:   ret = ioctl_open_cue  (self, arg); break;
	case PWM_LED_CLOSE_FADE: ret = ioctl_close_fade(self, arg); break;
	case PWM_LED_CLOSE_CUE:  ret = ioctl_close_cue (self, arg); break;
	case PWM_LED_PLAY_FADE:  ret = ioctl_play_fade (self, arg); break;
	case PWM_LED_PLAY_CUE:   ret = ioctl_play_cue  (self, arg); break;
	case PWM_LED_STOP_FADE:  ret = ioctl_stop_fade (self, arg); break;
	case PWM_LED_STOP_CUE:   ret = ioctl_stop_cue  (self, arg); break;
	case PWM_LED_SET_ACTIVE: ret = ioctl_set_active(self, arg); break;
	case PWM_LED_SET_DUTY:   ret = ioctl_set_duty  (self, arg); break;
	case PWM_LED_GET_DUTY:   ret = ioctl_get_duty  (self, arg); break;
	case PWM_LED_SET_ADAPT:  ret = ioctl_set_adapt (self, arg); break;
	/* clang-format on */
	default:
		dev_warn(dev, "ioctl: invalid cmd 0x%02X, dir %u, size %u",
			 _IOC_NR(cmd), _IOC_DIR(cmd), _IOC_SIZE(cmd));
		ret = -EINVAL;
		goto imx_pwm_led_dev_ioctl_exit;
	}
	/* Clear the writing flag: */
	self->writing = 0;
imx_pwm_led_dev_ioctl_exit:
	mutex_unlock(&self->lock);
	return ret;
}

const struct file_operations imx_pwm_led_dev_fops = {
	/* clang-format off */
	.owner          = THIS_MODULE,
	.open           = imx_pwm_led_dev_open,
	.read           = imx_pwm_led_dev_read,
	.write          = imx_pwm_led_dev_write,
	.fsync          = imx_pwm_led_dev_fsync,
	.llseek         = imx_pwm_led_dev_llseek,
	.release        = imx_pwm_led_dev_close,
	.unlocked_ioctl = imx_pwm_led_dev_ioctl,
	/* clang-format on */
};

/*******************************************************************************
 * DEVICE PROBE / REMOVE
 ******************************************************************************/

static const struct of_device_id imx_pwm_led_dt_ids[] = {
	{ .compatible = "unu,imx-pwm-led" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_pwm_led_dt_ids);

static int probe_first_dev(struct platform_device *pdev)
{
	int ret;
	uint i;
	struct device_node *epit_np = NULL;

	/* Allocate global data and contiguous duty and fade buffers: */
	glb_data.leds =
		kzalloc(max_leds * sizeof(struct imx_pwm_led *), GFP_KERNEL);
	if (glb_data.leds == NULL) {
		dev_err(&pdev->dev, "probe: failed to allocate memory");
		return -ENOMEM;
	}
	glb_data.fades =
		kzalloc(max_fades * sizeof(struct led_fade), GFP_KERNEL);
	if (glb_data.fades == NULL) {
		dev_err(&pdev->dev, "probe: failed to allocate memory");
		ret = -ENOMEM;
		goto probe_failed_alloc_fades;
	}
	for (i = 0; i < max_fades; i++) {
		mutex_init(&glb_data.fades[i].lock);
	}
	glb_data.cues = kzalloc(max_cues * sizeof(struct led_cue), GFP_KERNEL);
	if (glb_data.cues == NULL) {
		dev_err(&pdev->dev, "probe: failed to allocate memory");
		ret = -ENOMEM;
		goto probe_failed_alloc_cues;
	}
	for (i = 0; i < max_cues; i++) {
		mutex_init(&glb_data.cues[i].lock);
	}
	for (i = 0; i < max_cues; i++) {
		glb_data.cues[i].items =
			kzalloc(get_max_cue_size(), GFP_KERNEL);
		if (glb_data.cues[i].items == NULL) {
			dev_err(&pdev->dev, "probe: failed to allocate memory");
			ret = -ENOMEM;
			goto probe_failed_cue_items_alloc;
		}
	}
	glb_data.duty_buf = dma_zalloc_coherent(&pdev->dev, get_duty_buf_size(),
						&glb_data.duty_buf_phys,
						GFP_KERNEL);
	if (glb_data.duty_buf == NULL) {
		dev_err(&pdev->dev, "probe: failed to allocate duty buffer");
		ret = -ENOMEM;
		goto probe_failed_fade_alloc;
	}
	dev_dbg(&pdev->dev, "probe: allocated %u byte duty buffer at 0x%08x",
		get_duty_buf_size(), glb_data.duty_buf_phys);
	for (i = 0; i < max_fades; i++) {
		struct led_fade *fade = &glb_data.fades[i];
		fade->buf = dma_zalloc_coherent(&pdev->dev, max_fade_size,
						&fade->buf_phys, GFP_KERNEL);
		if (fade->buf == NULL) {
			dev_err(&pdev->dev, "probe: failed to allocate fade "
					    "buffer");
			ret = -ENOMEM;
			goto probe_failed_fade_alloc;
		}
		dev_dbg(&pdev->dev,
			"probe: fade %u: allocated %u byte buffer "
			"at 0x%08x",
			i, max_fade_size, fade->buf_phys);
	}

	/* Set up the EPIT: */
	epit_np = of_parse_phandle(pdev->dev.of_node, "unu,timer", 0);
	if (IS_ERR(epit_np)) {
		dev_err(&pdev->dev, "probe: unu,timer property not specified");
		ret = -ENODEV;
		goto probe_failed_epit_init;
	}
	glb_data.epit = epit_get(epit_np);
	of_node_put(epit_np);
	if (!glb_data.epit) {
		dev_err(&pdev->dev, "probe: failed to get timer");
		ret = -ENODEV;
		goto probe_failed_epit_init;
	}
	ret = epit_init_freerunning(glb_data.epit, NULL, NULL);
	if (ret) {
		dev_err(&pdev->dev, "probe: failed to initialize timer");
		goto probe_failed_epit_init;
	}
	/* Start generating periodic EPIT events:
	 * (This won't cause sdma events yet, as all channels are disabled) */
	epit_start_hz(glb_data.epit, sample_rate);

	/* Setup the SDMA: */
	glb_data.sdma = sdma_engine_get();
	if (IS_ERR(glb_data.sdma)) {
		dev_err(&pdev->dev, "probe: failed to get sdma engine");
		ret = -ENODEV;
		goto probe_failed_sdma_init;
	}
	/* Read the SDMA script origin: */
	u32 sdma_script_origin;
	if (of_property_read_u32(pdev->dev.of_node, "unu,sdma-script-origin",
				 &sdma_script_origin)) {
		dev_err(&pdev->dev,
			"probe: unu,sdma-script-origin property not "
			"specified");
		goto probe_failed_sdma_init;
	}
	glb_data.sdma_script_origin = sdma_script_origin;
	/* Load the SDMA script: */
	if ((ret = load_sdma_script(&pdev->dev))) {
		goto probe_failed_load_sdma_script;
	}
	return 0;

probe_failed_load_sdma_script:
probe_failed_sdma_init:
	epit_stop(glb_data.epit);
probe_failed_epit_init:
probe_failed_fade_alloc:
	if (glb_data.duty_buf != NULL) {
		dma_free_coherent(&pdev->dev, get_duty_buf_size(),
				  glb_data.duty_buf, glb_data.duty_buf_phys);
	}
	for (i = 0; i < max_fades; i++) {
		const struct led_fade *fade = &glb_data.fades[i];
		if (fade->buf != NULL) {
			dma_free_coherent(&pdev->dev, max_fade_size, fade->buf,
					  fade->buf_phys);
		}
	}
probe_failed_cue_items_alloc:
	for (i = 0; i < max_cues; i++) {
		mutex_destroy(&glb_data.cues[i].lock);
		kfree(glb_data.cues[i].items);
	}
	kfree(glb_data.cues);
probe_failed_alloc_cues:
	for (i = 0; i < max_fades; i++) {
		mutex_destroy(&glb_data.fades[i].lock);
	}
	kfree(glb_data.fades);
probe_failed_alloc_fades:
	kfree(glb_data.leds);
	return ret;
}

static void remove_first_dev(struct platform_device *pdev)
{
	uint i;
	epit_stop(glb_data.epit);
	dma_free_coherent(&pdev->dev, get_duty_buf_size(), glb_data.duty_buf,
			  glb_data.duty_buf_phys);
	for (i = 0; i < max_fades; i++) {
		const struct led_fade *fade = &glb_data.fades[i];
		dma_free_coherent(&pdev->dev, max_fade_size, fade->buf,
				  fade->buf_phys);
	}
	for (i = 0; i < max_cues; i++) {
		mutex_destroy(&glb_data.cues[i].lock);
		kfree(glb_data.cues[i].items);
	}
	kfree(glb_data.cues);
	for (i = 0; i < max_fades; i++) {
		mutex_destroy(&glb_data.fades[i].lock);
	}
	kfree(glb_data.fades);
	kfree(glb_data.leds);
}

static int imx_pwm_led_probe(struct platform_device *pdev)
{
	uint i;
	uint led_idx = glb_data.num_leds;
	const struct of_device_id *of_id =
		of_match_device(imx_pwm_led_dt_ids, &pdev->dev);
	struct imx_pwm_led *self;
	struct resource *r;
	int ret;
	u32 sdma_channel;

	if (!of_id) {
		return -ENODEV;
	}

	if (led_idx >= max_leds) {
		dev_err(&pdev->dev,
			"probe: more than max_leds devices configured");
		return -EFAULT;
	}
	self = devm_kzalloc(&pdev->dev, sizeof(*self), GFP_KERNEL);
	if (self == NULL) {
		dev_err(&pdev->dev, "probe: failed to allocate memory");
		return -ENOMEM;
	}
	self->led_idx = led_idx;
	self->dev = &pdev->dev;
	platform_set_drvdata(pdev, self);

	/* If this is the first device: */
	if (led_idx == 0) {
		if ((ret = probe_first_dev(pdev))) {
			return ret;
		}
	}
	glb_data.leds[led_idx] = self;

	self->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(self->clk_per)) {
		dev_err(&pdev->dev, "probe: getting per clock failed with %ld",
			PTR_ERR(self->clk_per));
		ret = PTR_ERR(self->clk_per);
		goto probe_failed_get_clk;
	}
	dev_dbg(&pdev->dev, "probe: clk_per rate: %lu Hz",
		clk_get_rate(self->clk_per));

	self->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(self->clk_ipg)) {
		dev_err(&pdev->dev, "probe: getting ipg clock failed with %ld",
			PTR_ERR(self->clk_ipg));
		ret = PTR_ERR(self->clk_ipg);
		goto probe_failed_get_clk;
	}
	dev_dbg(&pdev->dev, "probe: clk_ipg rate: %lu Hz",
		clk_get_rate(self->clk_ipg));

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	self->mmio_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(self->mmio_base)) {
		ret = PTR_ERR(self->mmio_base);
		goto probe_failed_get_resource;
	}
	self->mmio_base_phys = r->start;

	/* Generate the device name */
	self->dev_name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
					DEVICE_NAME "%u", led_idx);
	if (!self->dev_name) {
		dev_err(&pdev->dev, "probe: failed to allocate device name");
		ret = -ENOMEM;
		goto probe_failed_dev_name_alloc;
	}

	/* Read SDMA channel number */
	if (of_property_read_u32(pdev->dev.of_node, "unu,sdma-channel",
				 &sdma_channel)) {
		dev_err(&pdev->dev,
			"probe: unu,sdma-channel property not specified");
		goto probe_failed_get_sdma_channel;
	}
	self->sdma_ch_num = sdma_channel;

	/* Set up SDMA and get a channel reference */
	self->sdmac = sdma_get_channel(glb_data.sdma, self->sdma_ch_num);
	if (IS_ERR(self->sdmac)) {
		dev_err(&pdev->dev, "probe: failed to get sdma channel");
		ret = -ENODEV;
		goto probe_failed_get_sdma_channel;
	}
	if ((ret = load_sdma_initial_context(self))) {
		goto probe_failed_load_sdma_context;
	}
	sdma_set_channel_interrupt_callback(self->sdmac,
					    imx_pwm_led_sdma_callback, self);

	mutex_init(&self->lock);

	self->led_dev.minor = MISC_DYNAMIC_MINOR;
	self->led_dev.name = self->dev_name;
	self->led_dev.fops = &imx_pwm_led_dev_fops;
	self->led_dev.parent = &pdev->dev;
	if ((ret = misc_register(&self->led_dev))) {
		dev_err(&pdev->dev,
			"probe: unable to register " DEVICE_PATH "%s",
			self->dev_name);
		goto probe_failed_dev_register;
	}

	/* Configure but don't yet enable the PWM channel: */
	if ((ret = config_pwm(self, pwm_period, pwm_prescaler, pwm_invert))) {
		goto probe_failed_pwm_enable;
	}
	glb_data.num_leds++;
	return 0;

probe_failed_pwm_enable:
	misc_deregister(&self->led_dev);
probe_failed_dev_register:
	mutex_destroy(&self->lock);
	sdma_set_channel_interrupt_callback(self->sdmac, NULL, NULL);
probe_failed_load_sdma_context:
probe_failed_get_sdma_channel:
probe_failed_dev_name_alloc:
probe_failed_get_resource:
probe_failed_get_clk:
	if (led_idx == 0) {
		remove_first_dev(pdev);
	}
	return ret;
}

static int imx_pwm_led_remove(struct platform_device *pdev)
{
	struct imx_pwm_led *self = platform_get_drvdata(pdev);
	(void)stop_led(self);
	disable_pwm(self);

	misc_deregister(&self->led_dev);
	mutex_destroy(&self->lock);
	sdma_set_channel_interrupt_callback(self->sdmac, NULL, NULL);
	if (self->led_idx == 0) {
		remove_first_dev(pdev);
	}
	return 0;
}

static struct platform_driver imx_pwm_led_driver = {
    .driver = {
        .name = "imx-pwm-led",
        .owner = THIS_MODULE,
        .of_match_table = imx_pwm_led_dt_ids,
    },
    .probe = imx_pwm_led_probe,
    .remove = imx_pwm_led_remove,
};

/*******************************************************************************
 * MODULE INIT / EXIT
 ******************************************************************************/

static int __init imx_pwm_led_init(void)
{
	int ret;
	uint i;

	pr_info("%s: pwm_period: %u, sample_rate: %u Hz\n", THIS_MODULE->name,
		pwm_period, sample_rate);

	/* Reject nonsense parameters */
	if (sample_rate == 0) {
		pr_err("%s: value %u is invalid for sample_rate param\n",
		       THIS_MODULE->name, sample_rate);
		return -EINVAL;
	}
	if ((pwm_period < 2) || (pwm_period > __UINT16_MAX__)) {
		pr_err("%s: value %u is invalid for pwm_period param\n",
		       THIS_MODULE->name, pwm_period);
		return -EINVAL;
	}
	if (pwm_prescaler > MAX_PWM_PRESCALER) {
		pr_err("%s: value %u is invalid for pwm_prescaler param\n",
		       THIS_MODULE->name, pwm_prescaler);
		return -EINVAL;
	}
	if (pwm_invert > 1) {
		pr_err("%s: value %u is invalid for pwm_invert param\n",
		       THIS_MODULE->name, pwm_invert);
		return -EINVAL;
	}
	if (pwm_phase_offset > (MAX_UDELAY_MS * 1000)) {
		pr_err("%s: value %u is invalid for pwm_phase_offset param\n",
		       THIS_MODULE->name, pwm_phase_offset);
		return -EINVAL;
	}
	if ((max_leds == 0) || (max_leds > 32)) {
		pr_err("%s: value %u is invalid for max_leds param\n",
		       THIS_MODULE->name, max_leds);
		return -EINVAL;
	}
	if ((max_fades == 0) || (max_fades > __UINT16_MAX__)) {
		pr_err("%s: value %u is invalid for max_fades param\n",
		       THIS_MODULE->name, max_fades);
		return -EINVAL;
	}
	if ((max_cues == 0) || (max_cues > __UINT16_MAX__)) {
		pr_err("%s: value %u is invalid for max_cues param\n",
		       THIS_MODULE->name, max_cues);
		return -EINVAL;
	}
	if ((max_fade_size == 0) || ((max_fade_size % sizeof(u16)) != 0)) {
		pr_err("%s: value %u is invalid for max_fade_size param\n",
		       THIS_MODULE->name, max_fade_size);
		return -EINVAL;
	}
	if ((sdma_priority == 0) || (sdma_priority > 6)) {
		pr_err("%s: value %u is invalid for sdma_priority param\n",
		       THIS_MODULE->name, sdma_priority);
		return -EINVAL;
	}

	if ((ret = platform_driver_register(&imx_pwm_led_driver))) {
		pr_err("%s: failed to initialize led driver\n",
		       THIS_MODULE->name);
		return ret;
	}

	/* Enable all PWM channels at the same time, inserting the phase offset
	 * delay when configured: */
	local_irq_disable();
	for (i = 0; i < glb_data.num_leds; i++) {
		if (i > 0) {
			udelay(pwm_phase_offset);
		}
		enable_pwm(glb_data.leds[i]);
	}
	local_irq_enable();

	return 0;
}
module_init(imx_pwm_led_init);

static void __exit imx_pwm_led_exit(void)
{
	platform_driver_unregister(&imx_pwm_led_driver);
	pr_info("%s: unloaded\n", THIS_MODULE->name);
}
module_exit(imx_pwm_led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("unu GmbH <opensource@unumotors.com>");
MODULE_DESCRIPTION("Freescale i.MX6 PWM LED interface");
