# INF2004 - T15 - Pico W robot car Project

This project is about developing an intelligent autonomous robotic car with two wheels, guided by the Raspberry Pi Pico microcontroller. It involves utilizing a range of sensors and a sophisticated PID (Proportional-Integral-Derivative) control system. The challenge is to engineer a robotic vehicle capable of navigating a pre-set track while proficiently avoiding obstacles and accurately decoding barcodes using infrared sensors. The integration of the PID controller is crucial, as it significantly improves the precision and stability of the car's movements, ensuring smooth navigation and effective barcode scanning.

## Getting Started

These instructions will guide you through the setup and deployment of the robot car project on your Raspberry Pi Pico W.

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

![Screenshot of Pico w before uploading - Visual Studio Code](/image/upload_pico.jpeg)

Drag and drop the generated .uf2 file from the build directory onto the RPI-RP2 volume.

![Screenshot of uf2 file prepared - Visual Studio Code](/image/uf2_file.png)

### Hardware Setup

**Pico GPIO Connection**

![Screenshot of Pico w - Visual Studio Code](/image/pico_w_board.jpg)

Overall Pins' connection:

1. **L298N motor controller module**

![Screenshot of motor driver module - Visual Studio Code](/image/L298N_motor_controller.jpg)

GP0: ENA\
GP1: ENB\
GND (Pinout 3): GND\
GP2: IN2\
GP3: IN1\
GP4: IN4\
GP5: IN3\
3V3: +5V\

OUT1: left motor (GND)\
OUT2: left motor (+5V)\
OUT3: right motor (GND)\
OUT4: right motor (+5V)\

2. **GY-511 magnetometer module**

![Screenshot of magnetometer module - Visual Studio Code](/image/GY-511_magnetometer.jpg)

GND (Pinout 8): GND\
GP6 (I2C1 SDA): SDA\
GP7 (I2C1 SCL): SCL\
GP8: Vcc\

3. **HC020K rotary encoder module**

![Screenshot of rotary encoder module - Visual Studio Code](/image/HC020K_rotary_encoder.png)

GND (Pinout 13): GND (Left encoder/wheel)\
GP22: Vcc (Left encoder/wheel)\
GP26: OUT (Left encoder/wheel)\
GP27: OUT (right encoder/wheel)\
GP28: Vcc (right encoder/wheel)\
GND (Pinout 38): GND (right encoder/wheel)\

4. **HC-SR04 Ultrasound sensor module**

![Screenshot of ultrasound sensor module - Visual Studio Code](/image/HC-SR04_ultrasound_sensor.jpg)

GP13: Vcc\
GND (Pinout 18): GND\
GP14: TRIG\
GP15: ECHO\

5. **TCRT5000 Infra-Red line tracking module**

![Screenshot of infrared sensor module - Visual Studio Code](/image/TCRT5000_infrared_sensor.jpg)

GP16: Vcc\
GP17: A0 (Analog)\
GND (Pinout 23): GND\

# Project Block Diagram

## Components Block Diagram

![Screenshot of main block diagram - Visual Studio Code](/diagram/Block_Diagram_Final.png)

## Data Flow Diagram

![Screenshot of data flow diagram - Visual Studio Code](/diagram/Data_Flow_Diagram.png)



# Partial Integration

Under the partial_integration folder, you will find 4 subfolders for each of this project's partial integration.

1. /motor_forward_straight
2. /motor_turn_magno
3. /barcode_scanner_wifi
4. /obstacle_detector



## motor_forward_straight

The diagram below is the code flowchart for the integration of:

1. motor driver
2. encoder driver
3. PID Controller
4. FreeRTOS

![Screenshot of integration #1 Flowchart - Visual Studio Code](/diagram/Flowchart_motor_forward_straight.png)

### What it does:

It has 3 RTOS tasks: Task 1) check_speed_a_task, Task 2) check_speed_b_task, Task 3) move_forward_task.

Task 1 & 2 uses GPIO Interrupts together with the encoder module to trigger the check_speed_callback(). This callback will then calculate the speed for each wheel and send the value trhough the Queue Handlers managed by RTOS.

Task 3 will then receive the value from the queue buffers. with that, based on the two speeds (speedA and speedB), error and speed_correction is being calculated.

![Screenshot of move_forward_task error ans speed_correction code - Visual Studio Code](/docs/error_and_speed_correction_code.png)

Finally, the speed of the motor chnages according to the speed_correction as shown below:

![Screenshot of motor speed change code - Visual Studio Code](/image/speed_changes.png)



## motor_turn_magno

The diagram below is the code flowchart for the integration of:

1. motor driver
2. magnometer driver

![Screenshot of integration #2 Flowchart - Visual Studio Code](/diagram/Flowchart_motor_turn_magno.png)

### What it does:

In main(), the motor can either turn left or right, based on which line of code you uncomment and comment at a given point of time.

![Screenshot of turning sides code - Visual Studio Code](/image/turn_left_or_right.png)

Once executed, the robot car will move forward first. Afterwards it will turn either left or right, depending on the cinfiguration made as mentioned above.

The magnetometer will measure the angle turn from the y-axis. Either a 90 degress to the right (+90°) or left (-90°).

Finally, it will move forward a while more.



## barcode_scanner_wifi

The diagram below is the code flowchart for the integration of:

1. IR sensor
2. WIFI (TCP protocol)
3. FreeRTOS

TCP Server:
![Screenshot of integration #3 Flowchart server - Visual Studio Code](/diagram/Flowchart_barcode_scanner_wifi_server.png)

TCP CLient (Python):
![Screenshot of integration #3 Flowchart client - Visual Studio Code](/diagram/Flowchart_barcode_scanner_wifi_client.png)

### What it does:

As the IR sensor detect and scans the barcode lines one by one (Hand-driven), it is being transmitted to the TCP client on another device in the network.

The time take for each line determine the type of data it represents (as shown in the code below).

![Screenshot of ir barcode append - Visual Studio Code](/image/barcode_append.png)

Once a full-scan has been done, the sever will decode the barcode and transmits it to the client (as shown below).

![Screenshot of ir barcode console print - Visual Studio Code](/image/barcode_console_print.jpg)

(Note: the barcode scanning can be done in reverse too!)

![Screenshot of ir barcode reverse code - Visual Studio Code](/image/barcode_reverse.png)



## obstacle_detector

The diagram below is the code flowchart for the integration of:

1. motor driver
2. ultrasound sensor
3. FreeRTOS

![Screenshot of integration #4 Flowchart - Visual Studio Code](/diagram/Flowchart_obstacle_detector.png)

### What it does:

The car will continue to drive till it meets an obstacle blocking its way forward, detected using the Ultrasound sensor.

Once detected at a distance of below 30cm, it would stop and reverse for some distants.

![Screenshot of ultrasound detection code - Visual Studio Code](/image/ultrsound_detect.png)

after awhile, It would continue to drive again and keep detecting obstacle ahead of it.

![Screenshot of ultrasound task code - Visual Studio Code](/image/ultrasound_task.png)

# Conclusion

