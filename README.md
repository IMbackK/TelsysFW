#Telsys firmware

Requirments To build:

*  arm-none-eabi-gcc >= 7.2
*  openocd >0.10 (git needed as of 10.12.2017)
*  nrf5 sdk 12.3.0 (exact version needed due to sdk instability)
*  A Stlinkv2 is required to flash nordic device.

Procedure to make and flash device.

1. Edit CMakeLists.txt to reflect SDK, arm-none-eabi-gcc, and openocd install directories.
2. connect vcc and gnd to 3.3v supply
3. connect sconnect swdio, swclk and gnd from device to Stlinkv2
4. In the base directory execute:
    1. "cmake ."
    2. If this is the first flash of the device: "make softdevice" 
    3. "make"

To erase device use: "make erase"

Gui client software to log and analyze data transmitted by this firmware can be found here:
https://github.com/IMbackK/TelsysMaster

Open source hardware plans for the hardware this firmware runs on can be found here:
https://github.com/IMbackK/TelsysHW

