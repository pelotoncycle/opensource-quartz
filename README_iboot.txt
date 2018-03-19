BUILD ENVIRONMENT:

	1. Build Host OS: Ubuntu 14.04

	2. Toolchain: CodeSourcery compiler version Sourcery G++ Lite 2010q1-202 for ARM GNU/Linux 

	3. Installing required packages (Ubuntu 14.04):
		$ sudo apt-get install libc6:i386 libncurses5:i386 libstdc++6:i386 lzop



HOW TO BUILD I-BOOT FOR QUARTZ:

	1. Download and extract the toolchain:
		http://www.codesourcery.com/sgpp/lite/arm/portal/package6488/public/arm-none-linux-gnueabi/arm-2010q1-202-arm-none-linux-gnueabi-i686-pc-linux-gnu.tar.bz2

	2. Build commands:
		$ cd <path-to-iboot-source>
		$ export ARCH=arm
		$ export CROSS_COMPILE=<path-to-toolchain>/bin/arm-none-linux-gnueabi-
		$ make distclean
		$ make innocomm_quartz_config O=<path-to-iboot-source>/out/quartz
		$ make -j6 O=<path-to-iboot-source>/out/quartz

	3. Output files:
		SPL OMAP4-GP-signed image: <path-to-iboot-source>/out/quartz/MLO
		Bootloader image: <path-to-iboot-source>/out/quartz/iboot.img



HOW TO UPDATE I-BOOT FOR QUARTZ:

	1. Press VOLUME DOWN + POWER button to enter fastboot mode.

	2. Open a terminal and move into the folder that containing this README text file.

	3. The commands to update i-Boot: 
		$ ./fastboot flash xloader <path-to-MLO>
		$ ./fastboot flash bootloader <path-to-iboot.img>
		$ ./fastboot reboot
