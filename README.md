# INF2004 - T15 - Pico W Motor Control Project

This project is about developing an intelligent autonomous robotic car with two wheels, guided by the Raspberry Pi Pico microcontroller. It involves utilizing a range of sensors and a sophisticated PID (Proportional-Integral-Derivative) control system. The challenge is to engineer a robotic vehicle capable of navigating a pre-set track while proficiently avoiding obstacles and accurately decoding barcodes using infrared sensors. The integration of the PID controller is crucial, as it significantly improves the precision and stability of the car's movements, ensuring smooth navigation and effective barcode scanning.

## Getting Started

These instructions will guide you through the setup and deployment of the motor control project on your Raspberry Pi Pico W.

### Dependencies

- Raspberry Pi Pico W
- C/C++ development environment with `arm-none-eabi-gcc` compiler
- CMake (version 3.12 or later)
- FreeRTOS (bundled with this project)
- Motor driver and encoders corresponding to the hardware setup in the code

### Installing

1. **Setting Up the Pico SDK**:
   Follow the instructions from the Raspberry Pi Foundation to set up the Pico SDK and toolchain on your development machine:
   [Getting started with Raspberry Pi Pico](https://www.raspberrypi.org/documentation/pico/getting-started/).

2. **Cloning the Repository**:
   Clone the repository to your local machine using git:

   ```bash
   git clone https://github.com/Nexecis/INF2004_T15.git

### Executing program

1. **Building the Project**:
Create a build directory and navigate into it:

    ```bash
    mkdir build
    cd build
    ```

2. **Run CMake to build the project**:

    ```bash
    cmake ..
    make
    ```

Uploading to the Pico W:

Connect your Raspberry Pi Pico W to your computer while holding the BOOTSEL button to put it into USB mass storage mode.

![Screenshot of Pico w before uploading - Visual Studio Code](/docs/upload_pico.jpeg)

Drag and drop the generated .uf2 file from the build directory onto the RPI-RP2 volume.

![Screenshot of uf2 file prepared - Visual Studio Code](/docs/uf2_file.png)

### Hardware Setup

1. **Pico GPIO Connection**

![Screenshot of Pico w - Visual Studio Code](/docs/pico_w_board.jpg)

## Need all the pin connections listed here!

