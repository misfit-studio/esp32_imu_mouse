# ESP32 IMU-Based Motion Controller

This project is an implementation of an Inertial Measurement Unit (IMU) based motion controller using the ESP32 microcontroller. The project uses the BNO08x sensor for motion tracking and the RS485 protocol for communication.

## Project Structure

The project is organized into several directories, each with a specific purpose:

- `src`: Contains the main source code files for the project.
- `include`: Contains the project header files.
- `lib`: Contains project specific (private) libraries.
- `test`: Contains the unit tests for the project.

## Main Components

The main components of the project are:

- `main.cpp`: The main program file. It initializes the hardware, sets up the BNO08x sensor, and contains the main loop that reads sensor data, processes it, and sends it over RS485.
- `config.h`: Contains definitions for various hardware configuration parameters, such as pin assignments.
- `types.h`: Contains definitions for various data types used in the project.
- `utils.h`: Contains utility functions for tasks such as calculating CRCs.
- `quat_math.h`: Contains functions for converting quaternion data to Euler angles.

## Dependencies

The project depends on several libraries, which are specified in the `platformio.ini` file. These include:

- Adafruit_BNO08x: A library for interfacing with the BNO08x sensor.
- Bounce2: A library for debouncing button inputs.

## Building and Running

The project is built and run using PlatformIO. You can build the project by running `platformio run` in the project directory, and upload it to the ESP32 by running `platformio run --target upload`.