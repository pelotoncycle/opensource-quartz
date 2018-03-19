BUILD ENVIRONMENT:

	1. Build Host OS: Ubuntu 14.04

	2. Toolchain: CodeSourcery compiler version Sourcery G++ Lite 2010q1-202 for ARM GNU/Linux 

	3. Installing required packages (Ubuntu 14.04):
		$ sudo apt-get install libc6:i386 libncurses5:i386 libstdc++6:i386 lzop



HOW TO BUILD KERNEL FOR QUARTZ:

	1. Download and extract the toolchain:
		http://www.codesourcery.com/sgpp/lite/arm/portal/package6488/public/arm-none-linux-gnueabi/arm-2010q1-202-arm-none-linux-gnueabi-i686-pc-linux-gnu.tar.bz2

	2. Build commands:
		$ cd <path-to-kernel-source>/android-3.0
		$ export ARCH=arm
		$ export CROSS_COMPILE=<path-to-toolchain>/bin/arm-none-linux-gnueabi-
		$ make distclean
		$ make quartz_defconfig
		$ make -j6 zImage

	3. Output file:
		zImage: <path-to-kernel-source>/android-3.0/arch/arm/boot/zImage




HOW TO UPDATE KERNEL AND RECOVERY IMAGE FOR QUARTZ:

	1. Press VOLUME DOWN + POWER button to enter fastboot mode.

	2. Open a terminal and move into the folder that containing this README text file.

	3. The commands to upate kernel and recovery: 
		$ ./mkbootimg  --kernel <path-to-zImage> --ramdisk ramdisk.img --base 0x80000000 --output boot.img
		$ ./mkbootimg  --kernel <path-to-zImage> --ramdisk ramdisk-recovery.img --base 0x80000000 --output recovery.img
		$ ./fastboot flash boot boot.img
		$ ./fastboot flash recovery recovery.img
		$ ./fastboot reboot


