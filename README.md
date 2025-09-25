# NES_DE2_70
NES For FPGA Terasic Altera DE2-70 Cyclone 2 EP2C70F896C6
# Preamble
Please note and give credit to strigeus and its fpganes project ([strigeus/fpganes](https://github.com/strigeus/fpganes)). This is a fork and a modified version of its work to port its project to the DE2-70 board.
## Feel free to submit correction
# Requirements
Quartus 2 64-bit Web Edition v13sp1<br>
Visual Studio 2022 Community Edition v143
# Loader
Use "loader"(nes_loader) to download ROMS to it over the built-in UART.<br>
"loader"(nes_loader) also transmits joypad commands from my USB joypad to the FPGA across UART. It use the joystick library of windows (joystickapi.h).<br>
Run from command line. Need 2 arguments : path/name of the nes rom to load and the com port of the UART adapter for the board :
```
$> C:\path\to\the\loader\nes_loader.exe donkeykong.nes COM7
```
