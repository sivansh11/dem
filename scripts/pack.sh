#!/bin/bash

mkdir -p rootfs
cd rootfs
find . -print0 | cpio --null -ov --format=newc -R 0:0 > ../rootfs.cpio
cd ../
