#!/bin/bash
if [ -f u-boot.bin ];
then
	rm u-boot.bin
fi

clear
echo "*** Generating BBT Boot-loader ***"
cp smdk2416-bbt.h include/configs/smdk2416.h
make

if [ $? != 0 ];
then
	echo "Error while generating BBT Boot-loader"
	exit 1
fi

cp u-boot.bin f-u-boot.bin
cp u-boot.bin /tftpboot/f-u-boot.bin
sync

clear
echo "*** Generating non-BBT Boot-loader ***"
cp smdk2416-no-bbt.h include/configs/smdk2416.h
make

if [ $? != 0 ];
then
	echo "Error while generating non-BBT Boot-loader"
	exit 1
fi

cp u-boot.bin /tftpboot/u-boot.bin
sync

