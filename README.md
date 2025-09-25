# NES_DE2_70
NES For FPGA Terasic Altera DE2-70 Cyclone 2 EP2C70F896C6
# Preamble
Please note and give credit to strigeus and its fpganes project ([strigeus/fpganes](https://github.com/strigeus/fpganes)). This is a fork and a modified version of its work to port its project to the DE2-70 board.
## Feel free to submit correction
# Requirements
Quartus 2 64-bit Web Edition v13sp1<br>
Visual Studio 2022 Community Edition v143<br>
GCC 14.2.1
# Loader
Use "loader"(nes_loader) to download ROMS to it over the built-in UART.<br>
"loader"(nes_loader) also transmits joypad commands from my USB joypad to the FPGA across UART. It use the joystick library of windows (joystickapi.h).<br>
Run from command line. Need 2 arguments : path/name of the nes rom to load and the com port of the UART adapter for the board :
### Windows :
find your UART com port of your board and use it in the command line.
```
$> C:\path\to\the\loader\nes_loader.exe donkeykong.nes COM7
```
### Linux :
joypad must be at /dev/input/js0.<br>
find your UART file of your board and use it in the command line.
```
$> /path/to/the/loader/nes_loader donkeykong.nes /dev/ttyUSB0
```
