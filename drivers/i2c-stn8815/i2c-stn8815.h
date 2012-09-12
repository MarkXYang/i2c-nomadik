/*
 * i2c-stn8815.h - I2C controller of STN8815
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

#ifndef __I2C_STN8815_H
#define __I2C_STN8815_H


/* Platform data */
struct i2c_stn8815_platform_data {

	/*
	 * filter:
	 * set up the digital filter. Holds the number of clock-wide-spikes
	 *   0 -->  no filter
	 *   1 -->  1 clock-wide-spikes filter
	 *   2 -->  2 clock-wide-spikes filter
	 *   3 -->  4 clock-wide-spikes filter
	 */

#define	I2C_STN8815_FILTER_NONE		0
#define	I2C_STN8815_FILTER_1_CLK	1
#define	I2C_STN8815_FILTER_2_CLK	2
#define	I2C_STN8815_FILTER_3_CLK	3
#define	I2C_STN8815_FILTER_MASK		I2C_STN8815_FILTER_3_CLK

	unsigned char filter;

	/*
	 * speed:
	 * defines the I2C speed mode related to the serial bit rate
	 *   0 -->  standard mode (up to 100Kb/s)
	 *   1 -->  fast mode (up to 400Kb/s)
	 *   2 -->  high-speed (up to 3.4Mb/s)
	 */

#define	I2C_STN8815_SPEED_STANDARD	0
#define	I2C_STN8815_SPEED_FAST		1
#define	I2C_STN8815_SPEED_HIGH		2
#define I2C_STN8815_SPEED_MASK		3

	unsigned char speed;

	/*
	 * master_code:
	 * set up the master code used by the master during configuration in
	 * high-speed mode. Only the last three bits are relevant.
	 *   0...7 -->  master code
	 */

	unsigned char master_code;

};


#endif	/* __I2C_STN8815_H */
