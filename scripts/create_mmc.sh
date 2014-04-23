#!/bin/bash
# This script generates SD card disk images suitable for use with QEMU.
#
# Copyright (C) 2011 Ash Charles
# Copyright (C) 2013 BMW AG
# Copyright (C) 2013 BMW Car IT GmbH
#
# Based on:
#   Narcissus - Online image builder for the angstrom distribution
#   Copyright (C) 2008 - 2011 Koen Kooi
#   Copyright (C) 2010        Denys Dmytriyenko
# and
#   Linaro Images Tools.
#   Author: Guilherme Salgado <guilherme.salgado@linaro.org>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

# fail on error
set -e

# treat empty variables as error
set -u

# debug
#set -x

if [ "$(whoami)" != "root" ]; then
        echo "Error: Root rights missing. Please run as root or with sudo."
        exit 1;
fi

LC_ALL=C
LANG=en_US
set -e

BOOT_SIZE=128000000
ROOTFS_SIZE=3000000000

function usage()
{
    echo "This utility generates SD card images suitable for use with QEMU."
    echo "Usage:"
    echo "  $0 <output name| mmc> <filesstem ext3|ext4>"
    echo "Example:"
    echo "  $0 /dev/sdf ext4"
}

function check_args()
{
    echo $#
    if [ $# -lt 2 ]; then
        usage
        exit 1
    fi
    OUTFILE=$1
    FILESYSTEM=$2
    MLO=MLO
    UBOOT=u-boot.img
    KERNEL=uImage-pandaboard.bin
    ROOTFS=core-image-ros-roscore-pandaboard.tar.gz
    BOOTTXT=boot.txt
    BOOTSCR=boot.scr
    UENVTXT=""
    TEMPDIR=`mktemp -d --tmpdir`

    echo $FILESYSTEM

    if [ "$FILESYSTEM" != "ext3" ] && [ "$FILESYSTEM" != "ext4" ]
    then
        echo "No reasonable filesystem provided. Please choose between ext3 and ext4! Quitting..."
        exit 1
    fi

    if ! [[ -e ${MLO} ]]; then
        echo "MLO not found at ${MLO}! Quitting..."
        exit 1
    fi
    if ! [[ -e ${UBOOT} ]]; then
        echo "U-boot not found at ${UBOOT}! Quitting..."
        exit 1
    fi
    if ! [[ -e ${KERNEL} ]]; then
        echo "Kernel not found at ${KERNEL}! Quitting..."
        exit 1
    fi
    if ! [[ -e ${ROOTFS} ]]; then
        echo "Rootfs not found at ${ROOTFS}! Quitting..."
        exit 1
    fi
    echo
    if [ ! -r ${OUTFILE} -a -w ${OUTFILE} ]; then
        echo Please make sure you have read and write permissions to device $1
        cleanup 1
    fi

    if ! [[ -e ${UENVTXT} ]]; then
        echo "UENVTXT not found at ${UENVTXT}! Ignoring..."
    fi
    if ! [[ -e ${BOOTTXT} ]]; then
        echo "BOOTTXT not found at ${BOOTTXT}! Ignoring..."
    fi
    if ! [[ -e ${BOOTSCR} ]]; then
        if ! [[ -e ${BOOTTXT} ]]; then
            echo "BOOTSCR not found at ${BOOTSCR}! Using default values ..."
            BOOTTXT=${TEMPDIR}/boot.txt
            echo "fatload mmc 0:1 0x80000000 uImage" >${BOOTTXT}
            echo "setenv bootargs rw vram=64M omapfb.vram=0:64M omapfb.debug=y mem=1G@0x80000000 root=/dev/mmcblk0p2 --no-log rootwait console=ttyO2,115200n8 ip=192.168.0.2::::zgw:eth0:off ipv6.disable=1 smsc95xx.turbo_mode=N" >>${BOOTTXT}
            echo "bootm 0x80000000" >>${BOOTTXT}
        else
            echo "BOOTSCR not found at ${BOOTSCR}! Using ${BOOTTXT} ..."
        fi

        BOOTSCR=${TEMPDIR}/boot.scr
        mkimage -A arm -T script -C none -n "Pandaboard GPT-NG" -d ${BOOTTXT} ${BOOTSCR}
    fi
}

function make_mmc()
{
    export LC_ALL=C

    if [ $# -ne 1 ]; then
        echo "Usage: $0 <drive>"
        exit 1;
    fi

    DRIVE=${OUTFILE}

    if [ ! "`mount | grep ${DRIVE}`" == "" ]; then
        echo "DEVICE STILL MOUNTED: `mount | grep ${DRIVE}`";
        echo "Unmount drives & partitions first"
        cleanup 1
    fi

    dd if=/dev/zero of=$DRIVE bs=1024 count=1024

    HEADS=`sfdisk -g $DRIVE 2>/dev/null | cut -d ' ' -f 4`

    SECTORS=`sfdisk -g $DRIVE 2> /dev/null | cut -d ' ' -f 6`

    CYLINDERS=`sfdisk -g $DRIVE 2> /dev/null | cut -d ' ' -f 2`

    if [ ${HEADS:-0} -le 0 -o ${SECTORS:-0} -le 0 -o ${CYLINDERS:-0} -le 0 ]; then
        echo Could not retrieve heads, sectors or cylinders from destination.
        cleanup 1
    fi

    echo HEADS - $HEADS heads
    echo SECTORS - $SECTORS sectors per track
    echo CYLINDERS - $CYLINDERS cylinders

    TOTALSPACE=`echo $HEADS*$SECTORS*$CYLINDERS | bc`
    echo Total - $TOTALSPACE sectors

    BOOT_CYLS=`echo $BOOT_SIZE/$HEADS/$SECTORS/512 | bc`
    echo Cylinder Boot Partition - $BOOT_CYLS cylinder

    ROOTFS_CYLS=`echo $ROOTFS_SIZE/$HEADS/$SECTORS/512 | bc`
    echo Cylinder Root FS - $ROOTFS_CYLS cylinder

    echo CYLINDERS - $CYLINDERS
    {
    echo ,$BOOT_CYLS,0x0C,*
    echo ,$ROOTFS_CYLS,0x83,-
    echo ,,0x83,-
    } | sfdisk $DRIVE

    sleep 2
}


function populate_mmc()
{

    array=`sfdisk -l ${DRIVE} | grep "^/dev" | awk '{ print $1 }'`
    DRIVE1=`echo $array | cut -d \  -f1`
    DRIVE2=`echo $array | cut -d \  -f2`
    DRIVE3=`echo $array | cut -d \  -f3`

    echo "[ Format vfat partition ]"
    mkfs.vfat -F 16 -n "boot" ${DRIVE1}

    echo "[ Format ${FILESYSTEM} partition ]"
    /sbin/mkfs.${FILESYSTEM} -L rootfs ${DRIVE2}

    echo "[ Format ${FILESYSTEM} partition ]"
    /sbin/mkfs.${FILESYSTEM} -L data ${DRIVE3}

    echo "[ Copying files to vfat ]"
    MNT_BOOT=${TEMPDIR}/mnt/boot
    mkdir -p ${MNT_BOOT}

    sudo mount -onoatime ${DRIVE1} ${MNT_BOOT}
    cp -v ${MLO} ${MNT_BOOT}/MLO
    cp -v ${UBOOT} ${MNT_BOOT}/u-boot.img
    cp -v ${KERNEL} ${MNT_BOOT}/uImage
#    cp -v ${UENVTXT} /mnt/uEnv.txt
    if [ -f ${BOOTTXT} ]; then
        cp -v ${BOOTTXT} ${MNT_BOOT}/boot.txt
    fi
    cp -v ${BOOTSCR} ${MNT_BOOT}/boot.scr
    sync
    sudo umount ${DRIVE1}

    echo "[ Copying file system ]"
    MNT_ROOT=${TEMPDIR}/mnt/root
    mkdir -p ${MNT_ROOT}

    mount -o noatime ${DRIVE2} ${MNT_ROOT}
    tar xaf ${ROOTFS} -C ${MNT_ROOT}
    sync
    umount ${DRIVE2}
}

function cleanup()
{
    for DRIVE in DRIVE1 DRIVE2; do
        eval DRIVE='${'${DRIVE}':-}'

        if [ -n ${DRIVE} -a -d ${DRIVE} ]; then
            umount ${DRIVE} 2> /dev/null
        fi
    done

    if [ -d ${TEMPDIR} ]; then
        rm -rf --one-file-system ${TEMPDIR}
    fi

    exit ${1}
}


ARGS=$*
check_args $ARGS
make_mmc $1
populate_mmc
echo "[ Done ]"
cleanup 0

