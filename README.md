# INF2004 - T15 - Pico W Motor Control Project

This project is about developing an intelligent autonomous robotic car with two wheels, guided by the Raspberry Pi Pico microcontroller. It involves utilizing a range of sensors and a sophisticated PID (Proportional-Integral-Derivative) control system. The challenge is to engineer a robotic vehicle capable of navigating a pre-set track while proficiently avoiding obstacles and accurately decoding barcodes using infrared sensors. The integration of the PID controller is crucial, as it significantly improves the precision and stability of the car's movements, ensuring smooth navigation and effective barcode scanning.

## Getting Started

These instructions will guide you through the setup and deployment of the motor control project on your Raspberry Pi Pico W.

### Dependencies

- Raspberry Pi Pico W
- C/C++ development environment with `arm-none-eabi-gcc` compiler
- CMake (version 3.12 or later)
- FreeRTOS (bundled with this project)
- Motor driver and encoders for motor control feedback
- IR sensors for barcode recognition
- Ultrasound sensors for obstacle detection and distance measurement
- Appropriate connecting wires and resistors for sensor integration
- Personal computer for compiling and uploading the firmware to the Raspberry Pi Pico W

### Installing

1. **Setting Up the Pico SDK**:
   Follow the instructions from the Raspberry Pi Foundation to set up the Pico SDK and toolchain on your development machine:
   [Getting started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf).


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

3. **Uploading to the Pico W**:

Connect your Raspberry Pi Pico W to your computer while holding the BOOTSEL button to put it into USB mass storage mode.

![Screenshot of Pico w before uploading - Visual Studio Code](/docs/upload_pico.jpeg)

Drag and drop the generated .uf2 file from the build directory onto the RPI-RP2 volume.

![Screenshot of uf2 file prepared - Visual Studio Code](/docs/uf2_file.png)

### Hardware Setup

**Pico GPIO Connection**

![Screenshot of Pico w - Visual Studio Code](/docs/pico_w_board.jpg)

Overall Pins' connection:

1. **L298N motor driver**

![Screenshot of motor driver module - Visual Studio Code](/docs/L298N_motor_driver.jpg)

GP0: ENA
GP1: ENB
GND (Pinout 3): GND
GP2: IN2
GP3: IN1
GP4: IN4
GP5: IN3
VBUS: +5V

OUT1: left motor (GND)
OUT2: left motor (+5V)
OUT3: right motor (GND)
OUT4: right motor (+5V)

2. **GY-511 magnometer driver**

![Screenshot of magnetometer module - Visual Studio Code](/docs/GY-511_magnetometer.jpg)

GND (Pinout 8): GND
GP6: SDA
GP7: SCL
GP8: Vcc

3. **HC020K rotary encoder driver**

![Screenshot of rotary encoder module - Visual Studio Code](/docs/HC020K_rotary_encoder.png)

GND (Pinout 13): GND (Left encoder/wheel)
GP22: Vcc (Left encoder/wheel)
GP26: OUT (Left encoder/wheel)
GP27: OUT (right encoder/wheel)
GP28: Vcc (right encoder/wheel)
GND (Pinout 38): GND (right encoder/wheel)

4. **HC-SR04 Ultrasound driver**

![Screenshot of ultrasound sensor module - Visual Studio Code](/docs/HC-SR04_ultrasound_sensor.jpg)

GND (Pinout 18):
GP14:
GP15:
VBUS: Vcc



# Partial Integration

Under the partial_integration folder, you will find 4 subfolders for each of this project's partial integration.

1. /motor_forward_straight
2. /motor_turn_magno
3. /barcode_scanner_wifi
4. /obstacle_detector



## motor_forward_straight

The diagram below is the flowchart for the integration of:

1. motor driver
2. encoder driver
3. PID Controller
4. FreeRTOS

![Screenshot of integration #1 Flowchart - Visual Studio Code](/Diagram/Flowchart_motor_forward_straight.png)

### What it does:

It has 3 RTOS tasks: Task 1) check_speed_a_task, Task 2) check_speed_b_task, Task 3) move_forward_task.

Task 1 & 2 uses GPIO Interrupts together with the encoder module to trigger the check_speed_callback(). This callback will then calculate the speed for each wheel and send the value trhough the Queue Handlers managed by RTOS.

Task 3 will then receive the value from the queue buffers. with that, based on the two speeds (speedA and speedB), error and speed_correction is being calculated.

![Screenshot of move_forward_task error ans speed_correction code - Visual Studio Code](/docs/error_and_speed_correction_code.png)

Finally, the speed of the motor chnages according to the speed_correction as shown below:

![Screenshot of motor speed change code - Visual Studio Code](/docs/speed_changes.png)



## motor_turn_magno

The diagram below is the flowchart for the integration of:

1. motor driver
2. magnometer driver

![Screenshot of integration #2 Flowchart - Visual Studio Code](/Diagram/Flowchart_motor_turn_magno.png)

### What it does:

In main(), the motor can either turn left or right, based on which line of code you uncomment and comment at a given point of time.

![Screenshot of turning sides code - Visual Studio Code](/docs/turn_left_or_right.png)



## barcode_scanner_wifi




## obstacle_detector