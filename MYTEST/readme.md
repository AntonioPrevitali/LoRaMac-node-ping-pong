This folder and all its contents contain the project running on STM32CubeIDE and STM32F103C8T6 and SX1278 on RA01 module.
This is a test, it works, but it is certainly not a good porting.
My goal was to study and test the 5.0 branch with this, let's say unsupported, hardware.

Do not use this in production!
The ping pong main has also been modified and to make it work on one module you need to set is_master = true and on the other is_master = false.
A Log has also been added, an in memory log! that I use to trace the hardware commands (registers) that the software manipulates.
