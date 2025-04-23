# Readme

This folder and all its contents contain the project running on STM32CubeIDE and STM32F103C8T6 and SX1278 on RA01 module.
This is a test, it works, but it is certainly not a good porting.
My goal was to study and test the 5.0 branch with this, let's say unsupported, hardware.

Do not use this in production!
The ping pong main has also been modified and to make it work on one module you need to set is_master = true and on the other is_master = false.
A Log has also been added, an in memory log! that I use to trace the hardware commands (registers) that the software manipulates.

See myvarie.c file and run debug. I recommend inserting break-points in MySx1278_hal_read and MySx1278_hal_write

Below is the log produced while the master (TX) is running, without the counterpart (RX). So in this case the PING packet is sent every 1 sec but there are no responses because RX is off.

Basically this log is the printf that the program executes.

Reset il hardware reset of sx1278 with pin for reset.
```
      R: 01 ->09  R is Read of registry at adress 0x01
                  09 is value read from registry.
      W:  is write in registry.
```
Basically, in addition to being able to debug, which is very useful to understand the details of the loramac_radio, this log documents all the commands sent by the driver to the sx1278.
Happy Debugging and studying!

```
MyPrintf
radio_init
Reset 
R: 01 ->09 
W: 01 ->01 
R: 01 ->01 
W: 0C ->23 
R: 01 ->01 
W: 0D ->1E 
R: 01 ->01 
W: 0E ->D2 
R: 01 ->01 
W: 1A ->01 
R: 01 ->01 
W: 1F ->AA 
R: 01 ->01 
W: 24 ->07 
R: 01 ->01 
W: 27 ->12 
R: 01 ->01 
W: 28 ->C1 
R: 01 ->01 
W: 29 ->94 
R: 01 ->01 
W: 2A ->C1 
R: 01 ->01 
W: 30 ->D8 
R: 01 ->01 
W: 35 ->9F 
R: 01 ->01 
W: 3B ->02 
R: 01 ->01 
W: 40 ->00 
R: 01 ->01 
W: 41 ->30 
radio_lora_set_cfg
R: 01 ->01 
W: 01 ->01 
R: 01 ->01 
R: 01 ->01 
W: 01 ->00 
R: 01 ->00 
W: 01 ->80 
W: 40 ->00 00 
R: 1E ->70 64 
W: 1E ->70 00 
R: 09 ->4F 
W: 09 ->00 
W: 06 ->6C 80 00 
R: 3B ->1D 
W: 3B ->5D 
R: 3B ->5D 
W: 09 ->4F 
W: 06 ->6C 80 00 
R: 09 ->4F 09 
R: 4D ->84 
W: 09 ->CC 09 
W: 4D ->84 
R: 1D ->72 70 
R: 26 ->04 
W: 1D ->72 70 
W: 26 ->04 
R: 31 ->C3 
W: 31 ->C3 
W: 37 ->0A 
R: 01 ->80 
W: 01 ->01 
W: 0E ->00 00 
R: 1D ->72 70 
W: 1D ->72 74 
W: 20 ->00 08 
W: 22 ->10 10 
W: 39 ->12 
radio_set_rx
W: 11 ->1F 
R: 33 ->27 
W: 33 ->27 
W: 3B ->1D 
W: 40 ->00 00 
W: 36 ->03 
R: 31 ->C3 
W: 31 ->43 
W: 2F ->40 00 
W: 06 ->6C 80 00 
W: 0D ->00 
R: 01 ->81 
W: 01 ->05 
rx_timeout go
irq_rx_timeout
R: 01 ->85 
W: 01 ->00 
timerEnd
radio_trasmit ping
W: 22 ->10 
W: 0E ->00 
W: 0D ->00 
W: 00 ->50 49 4E 47 00 00 01 02 03 04 05 06 07 08 09 0A 
W: 11 ->F7 
R: 09 ->CC 09 
R: 4D ->84 
W: 09 ->CC 09 
W: 4D ->84 
R: 33 ->27 
W: 33 ->27 
W: 3B ->1D 
W: 40 ->40 00 
R: 01 ->80 
W: 01 ->03 
IrqD0 go! 
R: 01 ->81 
W: 12 ->08 
IrqD0 end 
is_irq_fired
R: 01 ->81 
W: 01 ->00 
radio_set_rx
W: 11 ->1F 
R: 33 ->27 
W: 33 ->27 
W: 3B ->1D 
W: 40 ->00 00 
W: 36 ->03 
R: 31 ->43 
W: 31 ->43 
W: 2F ->40 00 
W: 06 ->6C 80 00 
W: 0D ->00 
R: 01 ->80 
W: 01 ->05 
rx_timeout go
irq_rx_timeout
R: 01 ->85 
W: 01 ->00 
timerEnd
radio_trasmit ping
W: 22 ->10 
W: 0E ->00 
W: 0D ->00 
W: 00 ->50 49 4E 47 00 00 01 02 03 04 05 06 07 08 09 0A 
W: 11 ->F7 
R: 09 ->CC 09 
R: 4D ->84 
W: 09 ->CC 09 
W: 4D ->84 
R: 33 ->27 
W: 33 ->27 
W: 3B ->1D 
W: 40 ->40 00 
R: 01 ->80 
W: 01 ->03 
IrqD0 go! 
R: 01 ->81 
W: 12 ->08 
IrqD0 end 
is_irq_fired
R: 01 ->81 
W: 01 ->00 
radio_set_rx
W: 11 ->1F 
R: 33 ->27 
W: 33 ->27 
W: 3B ->1D 
W: 40 ->00 00 
W: 36 ->03 
R: 31 ->43 
W: 31 ->43 
W: 2F ->40 00 
W: 06 ->6C 80 00 
W: 0D ->00 
R: 01 ->80 
W: 01 ->05 
rx_timeout go
irq_rx_timeout
R: 01 ->85 
W: 01 ->00 
timerEnd
radio_trasmit ping

```
