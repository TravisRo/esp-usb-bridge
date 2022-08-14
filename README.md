# ESP USB Bridge


ESP USB Bridge Pico is a port of the [ESP-IDF](https://github.com/espressif/esp-idf) project utilizing a Raspberry PI RP2040 chip to create a bridge between a computer (PC) and a target microcontroller (MCU). It can serve as a replacement for USB-to-UART chips (e.g. CP210x).
The JTAG implementation uses the RP2040 infamous PIO peripheral in combination with DMA to achieve JTAG speeds in excess of 20 MHZ.  The JTAG frequency is set in KHZs using the openocd command parameter `-c adapter speed xxxx`.  IE: `-c "adapter speed 10000"` sets the frequency to 10MHZ.

The concept of ESP USB Bridge is shown in the following figure.

![ESP USB Bridge concept](images/concept.png)

ESP USB Bridge Pico creates a composite USB device accessible from the PC when they are connected through a USB cable. The main features are the following.
- *Serial bridge*: The developer can run [esptool](https://github.com/espressif/esptool) or connect a terminal program to the serial port provided by the USB CDC. The communication is transferred in both directions between the PC and the target MCU through the ESP USB bridge.
- *JTAG bridge*: [openocd-esp32](https://github.com/espressif/openocd-esp32) can be run on the PC which will connect to the ESP USB Bridge. The ESP32-S2 acts again as bridge between the PC and the MCU, and transfers JTAG communication between them in both directions.
- *Mass storage device*: USB Mass storage device is created which can be accessed by a file explorer in the PC. Binaries in UF2 format can be copied to this disk and the ESP32-S2 will use them to flash the target MCU. Currently ESP USB Bridge is capable of flashing various Espressif microcontrollers.

## How to Compile the Project

### Using VisualGDB

[VisualGDB](https://visualgdb.com) is an extention for Microsoft Visual Studios which enables a developer to seamlessly develop embedded software.  It's not free but it is the best tool for embedded software development I've ever used.
If you don't want to waste time fiddling with build environments and sub-par code editors then it's definitely for you.  It is available in four different flavors but the `Embedded` version ($99 US) is all you need for this project.

A VisualGDB solution and project file is provided but you will need to replace the pico-sdk that VisualGDB installs with my [pico-sdk](https://github.com/TravisRo/pico-sdk) fork. This is neccessary because the [tinyusb](https://github.com/hathach/tinyusb)
version included with orginal pico-sdk is `0.12` and this project requires `0.14`.

By default, VisualGDB will install the pico-sdk in:
'''
C:\Users\[Your Username]\AppData\Local\VisualGDB\PicoSDK\1.3.0-Package
'''
- Download my [pico-sdk fork as a zip file](https://github.com/TravisRo/pico-sdk/archive/refs/heads/master.zip)
- Unzip it into the VisualGDB pico-sdk folder shown above and overwrite all files

### Using MSYS2 MinGW64
MSYS2 MinGW64 is a completely free solution and works quite well for building RP2040 based projects but requires some additional setup and configuration explained below:
- Download [MSYS2](https://www.msys2.org/)
- Follow their `Installation` guide

Open the `MSYS2 MSYS` prompt and execute the following commands to install all the neccessary packages:
```bash
pacman -S --needed base-devel mingw-w64-x86_64-toolchain
pacman -S mingw-w64-x86_64-arm-none-eabi-toolchain
pacman -S mingw-w64-x86_64-cmake
```

Close the `MSYS2 MSYS` prompt. Open the `MSYS2 MinGW x64` prompt and execute the following commands to install the required source code:
```bash
mkdir ~/esp-usb-bridge-pico-dev
cd ~/esp-usb-bridge-pico-dev
git clone --recurse-submodules https://github.com/TravisRo/pico-sdk.git
git clone --recurse-submodules https://github.com/TravisRo/esp-usb-bridge-pico
```

To build the project, execute the following commands:
```bash
export PICO_SDK_PATH="~/esp-usb-bridge-pico-dev/pico-sdk"
alias make=mingw32-make
cd ~/esp-usb-bridge-pico-dev/esp-usb-bridge-pico
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
make
```

## Development/Breakout Board
The only requirement for the RP2040 board is that it has 8 pins available for full functionality.  4 pins are required for the JTAG interface and 4 pins are required for the programming interface.  The JTAG TDI and TMS pins must be sequential!  IE: If you use GPIO 27 for TDI then you MUST use GPIO 28 for TMS.

> Default PIN Assignments

|ESP Signal    |RP2040 PIN |Description                                       |
|:-------------|:---------:|:-------------------------------------------------|
|IO-0          |GPIO6      |ESP BOOT pin for serial programming interface     |
|EN            |GPIO7      |ESP Reset/EN pin for serial programming interface |
|U0RX          |GPIO4      |ESP RX pin for serial programming interface       |
|U0TX          |GPIO5      |ESP TX pin for serial programming interface       |
|MTDO / GPIO15 |GPIO14     |TDO pin for JTAG debugging interface              |
|MTDI / GPIO12 |GPIO27     |TDI pin for JTAG debugging interface              |
|MTCK / GPIO13 |GPIO29     |TCK pin for JTAG debugging interface              |
|MTMS / GPIO14 |GPIO28     |TMS pin for JTAG debugging interface              |

## Serial Programming Interface

The USB stack of ESP USB Bridge creates a virtual serial port through which the serial port of the target MCU is accessible. For example, this port can be `/dev/ttyACMx` or `COMx` depending on the operating system and is different from the PORT used for flashing the ESP USB Bridge.

For example, an ESP32 target MCU can be flashed and monitored with the following commands.
```bash
cd AN_ESP32_PROJECT
idf.py build
idf.py -p COMx flash monitor
```

Please note that [esptool](https://github.com/espressif/esptool) or any terminal program can connect to the virtual serial port as well.

## JTAG Debugging Interface

The ESP USB Bridge provides a JTAG device. The following command can be used to connect to an ESP32 target MCU.
```bash
idf.py openocd --openocd-commands "-f board/esp32-bridge.cfg"
```

## Mass Storage Device

A mass storage device will show up in the PC connected to the ESP USB bridge. This can be accessed as any other USB storage disk. Binaries built in [the UF2 format](https://github.com/microsoft/uf2) can be copied to this disk and the ESP32-S2 will flash the target MCU accordingly.

Binary `uf2.bin` will be generated and placed into the `AN_ESP32_PROJECT/build` directory by running the following commands.
```bash
cd AN_ESP32_PROJECT
idf.py uf2
```

## Known Issues

- None

## License

The code in this project Copyright 2020-2022 Espressif Systems (Shanghai) Co Ltd., and is licensed under the Apache License Version 2.0. The copy of the license can be found in the [LICENSE](LICENSE) file. Portions Copyright (C) 2022 Travis Robinson


## Contributing

We welcome contributions to this project in the form of bug reports, feature requests and pull requests.

Issue reports and feature requests can be submitted using Github Issues: https://github.com/TravisRo/esp-usb-bridge-pico/issues. Please check if the issue has already been reported before opening a new one.

Contributions in the form of pull requests should follow ESP-IDF project's [contribution guidelines](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/contribute/index.html). 
