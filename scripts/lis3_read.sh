#!/bin/sh
#
# === Script for debugging the linux li3lv02d accelerometer driver ===
#
# v1.0 (07/05/2012) - written by Fabrizio Ghiringhelli <fghiro@gmail.com>
#

if [ $# -ne 1 ]
then
  echo "usage: read_lis3lv02.sh <register file>"
  exit 1
fi

if ! [ -f "$1" ]
then
  echo "Cannot find $1"
  exit 1
fi

#### Read registers
while read line; do
	REG_NAME=$(echo $line | awk "!/^\#/ {print \$1}")
	REG_ADDR=$(echo $line | awk "!/^\#/ {print \$2}")
	if [ -n "$REG_NAME" ] && [ -n "$REG_ADDR" ]; then
		echo $REG_ADDR > /sys/devices/platform/lis3lv02d/read
		echo -n "$REG_NAME = "; cat /sys/devices/platform/lis3lv02d/read
	fi
done <$1
