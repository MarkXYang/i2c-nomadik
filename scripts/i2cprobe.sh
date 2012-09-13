#!/bin/sh
#
# === Script for debugging the i2c adapter of the Nomadik nhk8815 ===
#
# v1.0 (11/09/2012) - written by Fabrizio Ghiringhelli <fghiro@gmail.com>
#


#### probe i2c slave device
probe_device () {
	i2cexe -r $1 &>/dev/null
	if [ "$?" -eq 0 ]; then
		echo "found"
	else
		echo "not found"
	fi
}

#### probing the i2c devices of nhk8815 board
echo -n "Probing LIS3LV02DL accelerometer at 0x1D..."
probe_device 1d
echo -n "Probing TSC2003IPW touch screen controller at 0x48..."
probe_device 48
echo -n "Probing STw4811 power management at 0x2d..."
probe_device 2d
echo -n "Probing STw5095 stereo audio codec at 0x1A..."
probe_device 1a
echo -n "Probing STw4102 battery charger at 0x70..."
probe_device 70
