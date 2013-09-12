#!/bin/sh

sudo dd if=arch/arm/boot/uImage of=/dev/sdb bs=512 seek=2048 conv=fsync
sync
