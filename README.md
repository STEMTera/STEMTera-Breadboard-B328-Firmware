STEMTera Breadboard B328 Firmware using old LUFA-100807
=======================================================

![Image of STEMTera Breadboard Black](https://ksr-ugc.imgix.net/assets/013/802/899/bbadd26de0db2ed45bcaaae6ba0681be_original.jpg?w=680&fit=max&v=1474365591&auto=format&q=92&s=24632ecbf3e3ddc46b2484f247c5ecff)

Supported SKU : 
* STM100001
* STM100002
* STM100003
* STM100004
* STM100005
* STM100006
* STM100007

Summary
-------

STEMTera Breadboard is an innovation in breadboard history. It is the first breadboard with an Arduino compatible built-in that works with thousands of shields. With ATmega16U2/32U2 exposed, native USB projects can be easily developed using the LUFA framework. The LEGO® compatible bottom cover empowers projects to be built beyond imagination. 

Prerequisite
------------
1. [Atmel AVR 8-bit and 32-bit Toolchain 3.4.1 - Windows 98MBytes](http://www.atmel.com/tools/studioarchive.aspx)
2. [dfu-programmer](https://sourceforge.net/projects/dfu-programmer/files/dfu-programmer/0.7.2/)
3. [MinGW](http://www.mingw.org), install mingw32-base and msys-base package

Repository Contents
-------------------

* **/LUFA** - LUFA Framework
* **/STEMTera-Breadboard-B328-Firmware**
	* **/usbdfu** - USB DFU Bootloader, this bootloader can only be flashed using ICSP programmer.
	* **/usbserial** - USB CDC Serial Firmware, this firwmare can be flashed using dfu-programmer by putting STEMTera Breadboard in DFU mode (Please see below). This firmware also talks to AVRDUDE and sends Sketch to Arduino Bootloader. 

Description
-----------

This repository is almost identical to the original Arduino Firmware with some minor modification made to accomodate the ATmega16U2 and ATmega32U2.

Putting STEMTera Breaboard B328 into DFU Mode and Flashing Firmware
-------------------------------------------------------------------

1. Insert a [Mini Push Button Switch](https://www.sparkfun.com/products/97) into RST2 and GND of the STEMTera Breadboard.
2. Insert [Micro USB Cable](https://www.sparkfun.com/products/10215) into the computer and the other end of the micro usb connector to the STEMTera Breadboard. The GREEN LED on the STEMTera Breadboard should light up and the STEMTera Breadboard will be detected as Arduino UNO COM Port.
3. Press and HOLD the Mini Push Button until you hear a USB detached sound from the computer (**NOTE: Hold as firmly as possible because Windows OS is very sensitive to USB attach and detach really quickly.**). Release the Mini Push Button and an ATMEL USB Devices -> ATmega32U2 will appear in the Device Manager.
4. Change directory to **usbserial** and type `make dfu`.
5. The USB CDC Serial firmware will be flashed into the ATmega16U2/ATmega32U2.
6. Remove and re-insert the USB cable to start the firmware.