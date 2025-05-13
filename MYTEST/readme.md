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

in my first "release" of this TEST, see ghithub Tag MYTEST_V00 the printf were captured in a character buffer.
Now instead in this second "release" the printf are addressed to the serial port huart1 set to 9600 N 8 1

The log file is now like this:

```
Reset         is hardware reset of sx1278 with pin for reset
DelayMs(1)    is delay for reset 
DelayMs(6)    

R: 01 ->09 R is Read of registry at adress 0x01
                  09 is value read from registry.

W FSK : 01 ->01  FSK_OPMODE
                     W FSK :  s write in registry.
                      01 is adress registry 0x01
                        FSK_OPMODE is human name of registry

```
Basically, in addition to being able to debug, which is very useful to understand the details of the loramac_radio, this log documents all the commands sent by the driver to the sx1278.
Happy Debugging and studying!

```
MyPrintf
radio_init
Reset 
DelayMs(1)
DelayMs(6)
R: 01 ->09 
W FSK : 01 ->01  FSK_OPMODE
R: 01 ->01 
W FSK : 0C ->23  FSK_LNA
R: 01 ->01 
W FSK : 0D ->1E  FSK_RXCONFIG
R: 01 ->01 
W FSK : 0E ->D2  FSK_RSSICONFIG
R: 01 ->01 
W FSK : 1A ->01  FSK_AFCFEI
R: 01 ->01 
W FSK : 1F ->AA  FSK_PREAMBLEDETECT
R: 01 ->01 
W FSK : 24 ->07  FSK_OSC
R: 01 ->01 
W FSK : 27 ->12  FSK_SYNCCONFIG
R: 01 ->01 
W FSK : 28 ->C1  FSK_SYNCVALUE1
R: 01 ->01 
W FSK : 29 ->94  FSK_SYNCVALUE2
R: 01 ->01 
W FSK : 2A ->C1  FSK_SYNCVALUE3
R: 01 ->01 
W FSK : 30 ->D8  FSK_PACKETCONFIG1
R: 01 ->01 
W FSK : 35 ->9F  FSK_FIFOTHRESH
R: 01 ->01 
W FSK : 3B ->02  FSK_IMAGECAL
R: 01 ->01 
W FSK : 40 ->00  FSK_DIOMAPPING1
R: 01 ->01 
W FSK : 41 ->30  FSK_DIOMAPPING2
radio_lora_set_cfg
R: 01 ->01 
W FSK : 01 ->01  FSK_OPMODE
R: 01 ->01 
R: 01 ->01 
W FSK : 01 ->00  FSK_OPMODE
R: 01 ->00 
W LR : 01 ->80  REG_LR_OPMODE
W LR : 40 ->00  REG_LR_DIOMAPPING1
W LR : 41 ->00  REG_LR_DIOMAPPING2
R: 1E ->70 64 
W LR : 1E ->70  REG_LR_MODEMCONFIG2
W LR : 1F ->00  REG_LR_SYMBTIMEOUTLSB
R: 09 ->4F 
W LR : 09 ->00  REG_LR_PACONFIG
W LR : 06 ->6C  REG_LR_FRFMSB
W LR : 07 ->80  REG_LR_FRFMID
W LR : 08 ->00  REG_LR_FRFLSB
R: 3B ->1D 
W LR : 3B ->5D  REG_LR_INVERTIQ2
R: 3B ->5D 
W LR : 09 ->4F  REG_LR_PACONFIG
W LR : 06 ->6C  REG_LR_FRFMSB
W LR : 07 ->80  REG_LR_FRFMID
W LR : 08 ->00  REG_LR_FRFLSB
R: 09 ->4F 09 
R: 4D ->84 
W LR : 09 ->CC  REG_LR_PACONFIG
W LR : 0A ->09  REG_LR_PARAMP
W LR : 4D ->84  REG_LR_PADAC
R: 1D ->72 70 
R: 26 ->04 
W LR : 1D ->72  REG_LR_MODEMCONFIG1
W LR : 1E ->70  REG_LR_MODEMCONFIG2
W LR : 26 ->04  REG_LR_MODEMCONFIG3
R: 31 ->C3 
W LR : 31 ->C3  REG_LR_DETECTOPTIMIZE
W LR : 37 ->0A  REG_LR_DETECTIONTHRESHOLD
R: 01 ->80 
W LR : 01 ->01  REG_LR_OPMODE
W LR : 0E ->00  REG_LR_FIFOTXBASEADDR
W LR : 0F ->00  REG_LR_FIFORXBASEADDR
R: 1D ->72 70 
W LR : 1D ->72  REG_LR_MODEMCONFIG1
W LR : 1E ->74  REG_LR_MODEMCONFIG2
W LR : 20 ->00  REG_LR_PREAMBLEMSB
W LR : 21 ->08  REG_LR_PREAMBLELSB
W LR : 22 ->10  REG_LR_PAYLOADLENGTH
W LR : 23 ->10  REG_LR_PAYLOADMAXLENGTH
W LR : 39 ->12  REG_LR_SYNCWORD

Report registri usati e ultimo valore scritto 
FSK_OPMODE 0x00 
FSK_LNA 0x23 
FSK_RXCONFIG 0x1E 
FSK_RSSICONFIG 0xD2 
FSK_AFCFEI 0x01 
FSK_PREAMBLEDETECT 0xAA 
FSK_OSC 0x07 
FSK_SYNCCONFIG 0x12 
FSK_SYNCVALUE1 0xC1 
FSK_SYNCVALUE2 0x94 
FSK_SYNCVALUE3 0xC1 
FSK_PACKETCONFIG1 0xD8 
FSK_FIFOTHRESH 0x9F 
FSK_IMAGECAL 0x02 
FSK_DIOMAPPING1 0x00 
FSK_DIOMAPPING2 0x30 
REG_LR_OPMODE 0x01 
REG_LR_FRFMSB 0x6C 
REG_LR_FRFMID 0x80 
REG_LR_FRFLSB 0x00 
REG_LR_PACONFIG 0xCC 
REG_LR_PARAMP 0x09 
REG_LR_FIFOTXBASEADDR 0x00 
REG_LR_FIFORXBASEADDR 0x00 
REG_LR_MODEMCONFIG1 0x72 
REG_LR_MODEMCONFIG2 0x74 
REG_LR_SYMBTIMEOUTLSB 0x00 
REG_LR_PREAMBLEMSB 0x00 
REG_LR_PREAMBLELSB 0x08 
REG_LR_PAYLOADLENGTH 0x10 
REG_LR_PAYLOADMAXLENGTH 0x10 
REG_LR_MODEMCONFIG3 0x04 
REG_LR_DETECTOPTIMIZE 0xC3 
REG_LR_DETECTIONTHRESHOLD 0x0A 
REG_LR_SYNCWORD 0x12 
REG_LR_INVERTIQ2 0x5D 
REG_LR_DIOMAPPING1 0x00 
REG_LR_DIOMAPPING2 0x00 
REG_LR_PADAC 0x84 

radio_set_rx
W LR : 11 ->1F  REG_LR_IRQFLAGSMASK
R: 33 ->27 
W LR : 33 ->27  REG_LR_INVERTIQ
W LR : 3B ->1D  REG_LR_INVERTIQ2
W LR : 40 ->00  REG_LR_DIOMAPPING1
W LR : 41 ->00  REG_LR_DIOMAPPING2
W LR : 36 ->03  REG_LR_HIGHBWOPTIMIZE1
R: 31 ->C3 
W LR : 31 ->43  REG_LR_DETECTOPTIMIZE
W LR : 2F ->40  REG_LR_IFFREQ1
W LR : 30 ->00  REG_LR_IFFREQ2
W LR : 06 ->6C  REG_LR_FRFMSB
W LR : 07 ->80  REG_LR_FRFMID
W LR : 08 ->00  REG_LR_FRFLSB
W LR : 0D ->00  REG_LR_FIFOADDRPTR
R: 01 ->81 
W LR : 01 ->05  REG_LR_OPMODE

Report registri usati e ultimo valore scritto 
REG_LR_OPMODE 0x05 
REG_LR_FRFMSB 0x6C 
REG_LR_FRFMID 0x80 
REG_LR_FRFLSB 0x00 
REG_LR_FIFOADDRPTR 0x00 
REG_LR_IRQFLAGSMASK 0x1F 
REG_LR_IFFREQ1 0x40 
REG_LR_IFFREQ2 0x00 
REG_LR_DETECTOPTIMIZE 0x43 
REG_LR_INVERTIQ 0x27 
REG_LR_HIGHBWOPTIMIZE1 0x03 
REG_LR_INVERTIQ2 0x1D 
REG_LR_DIOMAPPING1 0x00 
REG_LR_DIOMAPPING2 0x00 


rx_timeout go
irq_rx_timeout
R: 01 ->85 
W LR : 01 ->00  REG_LR_OPMODE
DelayMs(2)
timerEnd
DelayMs(1)
radio_trasmit ping

W LR : 22 ->10  REG_LR_PAYLOADLENGTH
W LR : 0E ->00  REG_LR_FIFOTXBASEADDR
W LR : 0D ->00  REG_LR_FIFOADDRPTR
W: 00 ->50 49 4E 47 00 00 01 02 03 04 05 06 07 08 09 0A 
W LR : 11 ->F7  REG_LR_IRQFLAGSMASK
R: 09 ->CC 09 
R: 4D ->84 
W LR : 09 ->CC  REG_LR_PACONFIG
W LR : 0A ->09  REG_LR_PARAMP
W LR : 4D ->84  REG_LR_PADAC
R: 33 ->27 
W LR : 33 ->27  REG_LR_INVERTIQ
W LR : 3B ->1D  REG_LR_INVERTIQ2
W LR : 40 ->40  REG_LR_DIOMAPPING1
W LR : 41 ->00  REG_LR_DIOMAPPING2
R: 01 ->80 
W LR : 01 ->03  REG_LR_OPMODE

Report registri usati e ultimo valore scritto 
REG_LR_OPMODE 0x03 
REG_LR_PACONFIG 0xCC 
REG_LR_PARAMP 0x09 
REG_LR_FIFOADDRPTR 0x00 
REG_LR_FIFOTXBASEADDR 0x00 
REG_LR_IRQFLAGSMASK 0xF7 
REG_LR_PAYLOADLENGTH 0x10 
REG_LR_INVERTIQ 0x27 
REG_LR_INVERTIQ2 0x1D 
REG_LR_DIOMAPPING1 0x40 
REG_LR_DIOMAPPING2 0x00 
REG_LR_PADAC 0x84 

IrqD0 go! 
R: 01 ->81 
W LR : 12 ->08  REG_LR_IRQFLAGS
IrqD0 end 
is_irq_fired
irq_tx_done
R: 01 ->81 
W LR : 01 ->00  REG_LR_OPMODE
DelayMs(2)
radio_set_rx
W LR : 11 ->1F  REG_LR_IRQFLAGSMASK
R: 33 ->27 
W LR : 33 ->27  REG_LR_INVERTIQ
W LR : 3B ->1D  REG_LR_INVERTIQ2
W LR : 40 ->00  REG_LR_DIOMAPPING1
W LR : 41 ->00  REG_LR_DIOMAPPING2
W LR : 36 ->03  REG_LR_HIGHBWOPTIMIZE1
R: 31 ->43 
W LR : 31 ->43  REG_LR_DETECTOPTIMIZE
W LR : 2F ->40  REG_LR_IFFREQ1
W LR : 30 ->00  REG_LR_IFFREQ2
W LR : 06 ->6C  REG_LR_FRFMSB
W LR : 07 ->80  REG_LR_FRFMID
W LR : 08 ->00  REG_LR_FRFLSB
W LR : 0D ->00  REG_LR_FIFOADDRPTR
R: 01 ->80 
W LR : 01 ->05  REG_LR_OPMODE
rx_timeout go
irq_rx_timeout
R: 01 ->85 
W LR : 01 ->00  REG_LR_OPMODE
DelayMs(2)
timerEnd
DelayMs(1)
radio_trasmit ping

```
