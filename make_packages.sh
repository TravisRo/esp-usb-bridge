#!/usr/bin/env bash

# Absolute path to this script, e.g. /home/user/bin/foo.sh
SCRIPT=$(readlink -f "$0")
# Absolute path this script is in, thus /home/user/bin
SCRIPTPATH=$(dirname "$SCRIPT")

function make_board() {
	cd ${basedir}
	BOARD_ID=$1
	shift 1
	ADD_CMAKE_CXX_FLAGS=$@
	rm -f -r -v build_$BOARD_ID
	mkdir build_$BOARD_ID
	cd build_$BOARD_ID
	cmake -DPICO_BOARD=$BOARD_ID -DUSER_COMPILE_DEFINES=$ADD_CMAKE_CXX_FLAGS -G "MinGW Makefiles" ..
	mingw32-make
	rename -v dev_usbbridge_jtag $BOARD_ID dev_usbbridge_jtag.*
	cp -f $BOARD_ID.* ${packagedir}
}

FWVER=0.3
basedir=$SCRIPTPATH
packagedir=${basedir}/package_output/${FWVER}
curdir=$(pwd)

cd ${basedir}
mkdir -p ${packagedir}

# build generic pico
make_board pico $@

# build waveshare_rp2040_zero
make_board waveshare_rp2040_zero $@

# build tzt_rgb_usbc_rp2040
make_board tzt_rgb_usbc_rp2040 $@
