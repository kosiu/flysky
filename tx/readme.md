
Tutorial upload bootloader M8-8mhz internal clock.

Thanks to phil/SadSack which compile and give me the files.


1. In the folder
```
\arduino-1.0.4-windows\arduino-1.0.4\hardware\arduino\bootloaders\optiboot
```
put file
```
"optiboot_atmega8-int.hex"
```


2.In the file
Insert these lines below in
```
\arduino-1.0.4-windows\arduino-1.0.4\hardware\arduino\boards.txt
```
and save it.
```
##############################################################

atmega8_8mhz_int.name=ATmega8 phil(8MHz internal OSC)

atmega8_8mhz_int.upload.protocol=arduino
atmega8_8mhz_int.upload.maximum_size=7168
atmega8_8mhz_int.upload.speed=57600

atmega8_8mhz_int.bootloader.low_fuses=0xc4
atmega8_8mhz_int.bootloader.high_fuses=0xca
atmega8_8mhz_int.bootloader.path=optiboot
atmega8_8mhz_int.bootloader.file=optiboot_atmega8-int.hex
atmega8_8mhz_int.bootloader.unlock_bits=0x3f
atmega8_8mhz_int.bootloader.lock_bits=0x0F

atmega8_8mhz_int.build.mcu=atmega8
atmega8_8mhz_int.build.f_cpu=8000000L
atmega8_8mhz_int.build.core=arduino
atmega8_8mhz_int.build.variant=standard
##############################################################
```
3.Open again arduino ide and in the menu tools/board...you'll see
"ATmega8 phil(8MHz internal OSC)"
Select that board...connect the USB ASP programmer(or other programmer)
and select tools/burn bootloader.
END.
USE THIS BOOTLOADER ONLY FOR M8.

