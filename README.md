# Freescale i.MX PWM LED Linux Kernel Module
Copyright (C) 2020 unu GmbH

Written by [Geoffrey Phillips](http://geoff.phillips.fm).

## License
```
This library is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this library.  If not, see <http://www.gnu.org/licenses/>.
```

## Overview
This Linux kernel module controls multiple PWM channels using the SDMA for
driving LEDs on Freescale (NXP) [i.MX](https://www.nxp.com/products/processors-and-microcontrollers/arm-processors/i-mx-applications-processors)
processors.

Much of this code is adapted from the [imx-pwm-audio](https://github.com/Glowforge/kernel-module-imx-pwm-audio/)
module from Glowforge and the [meta-openglow](https://github.com/ScottW514/meta-openglow)
kernel patches from OpenGlow.
The [i.MX SDMA assembler](http://billauer.co.il/blog/2011/10/imx-sdma-howto-assembler-linux/)
from Eli Billauer was also used.

## Requirements
The module requires kernel patches to expose driver APIs to the i.MX EPIT and
SDMA. See the `linux-imx-x.x.x` directory.

## User-Space API
The module creates devices for each PWM channel visible at `/dev/pwm_led0` to
`/dev/pwm_led7` for the iMX6 Ultralight.

### Definitions
- A PWM 'period' is defined as a 16-bit PWM clock divider. For example, if the
  PWM clock is 24 MHz, and the PWM period is 12000, the output PWM frequency
  will be 2 kHz.
- A PWM 'duty-cycle value' is defined as a 16-bit counter value at which the
  output changes from 1 to 0, which can range from zero to the PWM period value.
  For example, if the PWM period is 12000, and a duty-cycle value of 6000 is
  set, the output PWM duty cycle will be 50%.
- The module data model has a set of 'fades' and 'cues' that are shared between
  all channels and are defined as follows:
  - A 'fade' is a buffer of PWM duty-cycle values of fixed sample rate. A fade
    can be played on a PWM channel, so that the output fades between the
    starting duty-cycle value and the finishing duty-cycle value. At the end of
    the fade, the duty-cycle of the PWM channel is left at the last fade value.
  - A 'cue' is a list of actions to start on one or more channels at the same
    time. The supported actions are:
    - Starting playback of a fade on a given channel.
    - Setting a given duty-cycle directly on a given channel.

    **Note:** A cue may start the same fade on multiple channels at the same
    time.

    The format of each item in the cue list is as follows (as defined in
    `pwm_led.h`):

        #define PWM_LED_CUE_ITEM_BIT_LED   0
        #define PWM_LED_CUE_ITEM_BIT_TYPE  8
        #define PWM_LED_CUE_ITEM_BIT_VAL   16
        #define PWM_LED_CUE_ITEM_MASK_LED  0x0000001F
        #define PWM_LED_CUE_ITEM_MASK_TYPE 0x00000F00 /* See below */
        #define PWM_LED_CUE_ITEM_MASK_VAL  0xFFFF0000
        #define PWM_LED_CUE_ITEM_TYPE_FADE 0 /* Value=fade index */
        #define PWM_LED_CUE_ITEM_TYPE_DUTY 1 /* Value=duty cycle */

  - For each channel the module has the following modes:
    - 'Adaptive': This is a configuration option. It causes the channel to
      'adapt' the played fades. When enabled and a fade is started, the first
      duty-cycle that is nearest to the current duty-cycle of the channel is
      found, and then the fade is started at that point. For example, if the
      current duty-cycle of a channel is 50% and a fade of 100% to 0% is played,
      the fade will be started at the point closest to 50%, rather than first
      jumping to 100%.
    - 'Active': This can be changed at run time. When a channel is active, the
      values of a fade played on the channel will set be set as the channel's
      duty-cycle, i.e. played as normal. When a channel is inactive, the fade
      value is not set as the channel's duty-cycle, instead a value of 0% is
      set.

      **Note:** This means that fades will still start and finish on a channel
      as normal, except the output duty-cycle is kept at 0% when inactive.

      **Note:** If a channel is changed from active to inactive whilst a fade is
      playing, the output will change directly from the current fade value to
      0%.

      **Note:** If a channel is changed from inactive to active whilst a fade is
      playing, the output will change directly from 0% to the current fade
      value.

### Module Parameters
The module has the following parameters (applicable to all channels):

    sample_rate      - Sample rate of the fades in Hz
    pwm_period       - Default PWM period, allowed range 2 to 65535
    pwm_prescaler    - Default PWM prescaler, N-1 value, allowed range 0 to 4095
    pwm_invert       - Default PWM invert mode, 0=non-inverted, 1=inverted
    pwm_phase_offset - Channel phase offset in microseconds
    max_fades        - Maximum number of fades
    max_cues         - Maximum number of cues
    max_leds         - Maximum number of LEDs (PWM channels)
    max_fade_size    - Maximum fade size in bytes
    sdma_priority    - SDMA priority, allowed range 1 to 6

### ioctl Commands
The devices support the following `ioctl` commands defined in `pwm_led.h`:

    PWM_LED_CONFIGURE  - sets the PWM period, prescaler, and invert mode
    PWM_LED_OPEN_FADE  - opens the fade of given index
    PWM_LED_OPEN_CUE   - opens the cue of given index
    PWM_LED_CLOSE_FADE - closes the fade of given index
    PWM_LED_CLOSE_CUE  - closes a cue of given index
    PWM_LED_PLAY_FADE  - plays a fade of given index
    PWM_LED_PLAY_CUE   - plays the cue of given index
    PWM_LED_STOP_FADE  - stops the fade on the the device
    PWM_LED_STOP_CUE   - stops the cue of given index
    PWM_LED_SET_ACTIVE - sets the device as active/inactive
    PWM_LED_SET_DUTY   - sets the PWM duty-cycle value
    PWM_LED_GET_DUTY   - gets the PWM duty-cycle value
    PWM_LED_SET_ADAPT  - sets the adaptive mode

- After a fade or cue has been opened, data can be written by writing to the
  device file as normal, e.g. using `cat`.
- To prevent concurrent access by multiple processes. A channel/fade/cue can
  only be opened by one process at a time. A subsequent attempt to open the item
  will fail until the item has been closed. When the file handle of a channel is
  closed, the module automatically closes any fades or cues that were opened.

**Note:** Normally a process will open all of the channels, fades and cues it
will use at startup. This will prevent other processes from accessing these
resources while they are in use.

**Note:** The fades are also protected against writing by the owning process
whilst they are being played on a channel.

## Device Tree Bindings
```
Required properties:
- compatible : Should be "unu,imx-pwm-led"
- unu,sdma-script-origin : **Only required for the first device**
  Location to load the SDMA script (in 32-bit data words).
  The 4KB SDMA internal RAM area for scripts and data is located at
  0xc00 - 0xfff. The imx-pwm-led script is 10 words long.
- unu,timer : **Only required for the first device** The EPIT timer used.
- unu,sdma-channel : The SDMA channel number.

Example:
&epit1 {
	compatible = "fsl,imx6ul-epit";
	id = <0>;
	clocks = <&clks IMX6UL_CLK_EPIT1>;
	clock-names = "epit1";
	sdma-event = <16>;
};

&pwm1 {
	compatible = "unu,imx-pwm-led";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_pwm1>;
	status = "okay";
	unu,timer = <&epit1>;
	unu,sdma-script-origin = <0xc00>;
	unu,sdma-channel = <24>;
};

&pwm2 {
	compatible = "unu,imx-pwm-led";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_pwm2>;
	status = "okay";
	unu,sdma-channel = <25>;
};
```

## Building
Building is normally performed using [Yocto](https://www.yoctoproject.org/).

Here is an example `bitbake` script:
```
SUMMARY = "Kernel loadable module for i.MX PWM LEDs"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://LICENSE;md5=b234ee4d69f5fce4486a80fdaf4a4263"

inherit module

SRCBRANCH = "master"
SRC_GIT = "git://git@github.com/unumotors/kernel-module-imx-pwm-led.git;protocol=ssh"
SRC_URI = "${SRC_GIT};branch=${SRCBRANCH}"
SRCREV = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

S = "${WORKDIR}/git"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

RPROVIDES_${PN} += "kernel-module-imx-pwm-led"

do_install_append() {
    install -d ${D}${includedir}/linux
    install -m 0644 ${S}/pwm_led.h ${D}${includedir}/linux
}
```

## Quick-Start Guide
Using [ioctl](https://github.com/jerome-pouiller/ioctl/):
```
# Load the module with parameters:
insmod imx_pwm_led.ko pwm_period=48000 pwm_invert=1

# Open fade0:
ioctl /dev/pwm_led0 0x00007541 -v 0

# Write a fade file:
cat my_fade.bin > /dev/pwm_led0

# Open cue0:
ioctl /dev/pwm_led0 0x00007542 -v 0

# Write 2 cue items to play fade0 on channels 0 and 1:
printf "\x00\x00\x00\x00\x01\x00\x00\x00" > /dev/pwm_led0

# Play cue0:
ioctl /dev/pwm_led0 0x00007546 -v 0
```
