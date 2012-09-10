#!/bin/sh
#
# === Master build for Nomadik project ===
#
# v1.0 (23/04/2012) - written by Fabrizio Ghiringhelli <fghiro@gmail.com>
#
# The script does the following:
#  - build all drivers in the 'drivers' folder (on subfolder for each driver)
#    and install the drivers (which are all modules) in $ROOTFS/lib/modules/...
#  - build each tools in the 'tools' folder and install them in $ROOTFS/bin
#  - copy all the scripts and files from the 'scripts' folder to $ROOTFS
#

#### build_driver (used later in this file)
build_driver () {
	make || exit 1
	make INSTALL_MOD_PATH=$MODULESDIR modules_install
	sudo cp -a $MODULESDIR/* $ROOTFS
}

#### build_tools (used later in this file)
build_tools () {
	make || exit 1

	# Copy object files to target /usr/bin folder
	sudo cp $(find -type f -perm -1 \( -not -iname "*.sh" \)) $ROOTFS/bin/
}

PRJDIR=$PRJROOT/project

#### Make and install the driver modules
cd $PRJDIR/drivers
for n in `ls -1`; do
	if test -d $n; then
		printf "\n*** Building driver $n...\n"
		cd $n
		build_driver
		cd ..
	fi
done

#### Make and install tools
cd $PRJDIR/tools
for n in `ls -1`; do
	if test -d $n; then
		printf "\n*** Building tools $n...\n"
		cd $n
		build_tools
		cd ..
	fi
done

#### Copy scripts
cd $PRJDIR/scripts
printf "\n*** Copying scripts...\n"
for n in `ls -1`; do
	echo "  $n"
	sudo cp $n $ROOTFS
done

#### Create ramdisk
printf "\n*** Creating the ramdisk...\n"
$DOCDIR/mkinitramfs $ROOTFS $ROOTFS/../ramdisk.cpio.gz

#### Copy ramdisk (only if SD is present)
if [ -d /media/B4E0-5BBD/ ]; then
  cp $ROOTFS/../ramdisk.cpio.gz /media/B4E0-5BBD/
else
  echo "SD card not found: manually copy the ramdisk to the SD card"
fi

#### Get the size of the ramdisk in hexadecimal format
SIZE=$(ls -l $ROOTFS/../ramdisk.cpio.gz | awk "{print \$5}")
SIZE_HEX=$(echo "obase=16; $SIZE" | bc)
echo "The size of the ramdisk is (hex format) 0x$SIZE_HEX"

printf "Done\n"
exit 0

