
CYStream20160226¡Gsupport 2.04G and USB3.1



Ver05.05_Modifying
==============================
data:20160226
1.add 3.0 GPIO6 and GPIO7.
	GPIO7	GPIO6
	1	1	default
	1	0	2.0 signal port
	0	1	don't care
	0	0	2.0 dual port
2.Modify Version 05.05

FW_Ver0505_201603041015
==============================
date:2016.03.04
1.add USB3.0 download mode command
	enter USB3.0 download mode Req Code: 0xbb  wValue: 0x0005
	exit  USB3.0 download mode Req Code: 0xbb  wValue: 0x0006


FW_Ver0505_201603071123
==============================
date:2016.03.07
1.in download FW mode,WP follow it's previous status


FW_Ver0506_160310
================================
date: 2016.03.10
1.modify the version to 0506


FW_Ver0507_160329
================================
release ª©¥»

2016.08.23
================================
version 0509
modify PD5 to control WP of eeprom.
Modify usb3.0 update FW error.
PCB REV:1.02

2016.10.11
================================
version 050A
add function: support Intel AR Chip
initial D flipflop

2017.2.6	Jimbo_Zhang
================================
version 050B
PCB REV	1.03
add function: 	speed check	(Alt7_BulkSpeed,EP2 Bulk IN(0xAA),EP6 Bulk OUT).
add function:	support header card.(add timer)
add function:	support video card.(add RLS_HPD and GET_RSLT)

RLS_HPD command: 0xbb

2017.07.18	 Jimbo_Zhang
================================
version 050B
PCB REV	1.03
1. Clear up EZUSB_Delay(ms).
2. Add SET_WAIT_ACK command. bRequest = 0xc6. wVlue = delay times. uint ms. Default 1ms.
3. Optimization speed check code.

2017.07.18	 Jimbo_Zhang
================================
version 050B
PCB REV	1.03
1. Replace SET_WAIT_ACK with SET_WAIT_TIME, bRequest = 0xc7. wVlue = waitTimes. uint ms. Default 1ms. 

2017.09.20	 Jimbo_Zhang
================================
version 050C
PCB REV	1.03
1. Release¡C Modify version to 050C¡C
2. Clear up Delay before switch speed¡C
2. Optimization speed check code.