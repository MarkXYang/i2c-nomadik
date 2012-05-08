/*
 * lis3lv02d-nhk8815.h - ST LIS3LV02DL accelerometer driver (I2C)
 *                       for Nomadik NHK8815 platform
 *
 * (2012) Written by Fabrizio Ghiringhelli <fghiro@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __LIS3LV02D_NHK8815_H
#define __LIS3LV02D_NHK8815_H


/* Platform data */
struct lis3lv02d_nhk8815_platform_data {

	/*
	 * device_cfg:
	 * holds the device configuration:
	 *   DF0, DF1: define the Output Data Rate (ODR):
	 *             DF0 = 0, DF1 = 0 -->  ODR = 40Hz
	 *             DF0 = 1, DF1 = 0 -->  ODR = 160Hz
	 *             DF0 = 0, DF1 = 1 -->  ODR = 640Hz
	 *             DF0 = 1, DF1 = 1 -->  ODR = 2560Hz
	 *   FULLSCALE: A '0' = 2g and a '1' = 6g
	 */

#define	LIS3_DF0	(1 << 4)
#define	LIS3_DF1	(1 << 5)
#define LIS3_FULLSCALE	(1 << 7)

	unsigned char device_cfg;

	/*
	 * free_fall_cfg:
	 * holds the Free-Fall configuration
	 *   XLIE: A '1' = enable FF detection on X Low event
	 *   XHIE: A '1' = enable FF detection on X High event
	 *   YLIE: A '1' = enable FF detection on Y Low event
	 *   YHIE: A '1' = enable FF detection on Y High event
	 *   ZLIE: A '1' = enable FF detection on Z Low event
	 *   ZHIE: A '1' = enable FF detection on Z High event
	 * If none of the above flags is set, Free-Fall detection is disabled.
	 * AOI: A '1' performs the AND combination of the above events, whereas
	 * a '0' performs the OR combination.
	 */

#define LIS3_FF_XL	(1 << 0)
#define LIS3_FF_XH	(1 << 1)
#define LIS3_FF_YL	(1 << 2)
#define LIS3_FF_YH	(1 << 3)
#define LIS3_FF_ZL	(1 << 4)
#define LIS3_FF_ZH	(1 << 5)
#define LIS3_FF_AOI	(1 << 7)
#define LIS3_FF_ALL	( LIS3_FF_XL | LIS3_FF_XH | \
			  LIS3_FF_YL | LIS3_FF_YH | \
			  LIS3_FF_ZL | LIS3_FF_ZH )

	unsigned char free_fall_cfg;

	/*
	 * free_fall_threshold:
	 * holds the threshold value for Free-Fall detection.
	 * The scale factor is 0.97 mg/LSB if full scale is 2g, and 2.93 mg/LSB
	 * if full scale is 6g.
	 */

	unsigned int free_fall_threshold;

	/*
	 * free_fall_duration:
	 * holds the minimum duration of the Free-Fall event.
	 * The effective Free-Fall duration is computed as:
	 *        free_fall_threshold / ODR   [sec]
	 */

	unsigned char free_fall_duration;

	/*
	 * poll_interval:
	 * holds the sampling period. The unit is ms.
	 */

	unsigned int poll_interval;

};


#endif	/* __LIS3LV02D_NHK8815_H */
