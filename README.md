# Object Tracking System Using FPGA

This project implements an object-centering system on an FPGA platform, utilizing an Opal Kelly interface with a CMV300 image sensor and an LSM303DLHC IMU. The system computes the center of mass from the camera's captured image and dynamically adjusts a motor to center the object within the frame.

---

## Features

- **FPGA Integration:** Built on an Opal Kelly FPGA platform with USB communication via FrontPanel.
- **Sensor Interfaces:** Utilized SPI for camera communication and I2C for IMU integration.
- **Object Centering:** Dynamically calculates the object's center of mass from image data and adjusts motor positioning via a PmodDHB1 dual H-Bridge interface.
- **Real-Time Image Processing:** Used Python and OpenCV to process images and detect motion.
- **System Optimization:** Implemented precise timing and finite state machines (FSMs) for robust sensor interfacing and control.

---

## System Requirements

- **Hardware:**
  - FPGA development board (Opal Kelly or equivalent).
  - Camera module (e.g., CMV300) and IMU (e.g., LSM303DLHC).
  - Motor with PmodDHB1 interface.
  - Computer with USB for interfacing.
- **Software:**
  - Vivado Design Suite.
  - Python with Spyder IDE.
  - OpenCV library for image processing.
  - Opal Kelly FrontPanel software.

---

## File Descriptions

- **Hardware Files:**
  - `I2C_Transmit.v`: Manages I2C communication with the IMU.
  - `SPI.v`: Handles SPI communication with the camera.
  - `ClockGenerator.v`: Generates required clock signals.
  - `BTPipeExample.v`: Integrates USB communication with FPGA logic.
- **Software Files:**
  - `cam.py`: Python script for image processing and motor control logic.

---

## Installation and Setup

1. Clone or download this repository.
2. Open the Vivado project and synthesize the hardware design.
3. Load the bitstream onto the FPGA using Opal Kelly FrontPanel.
4. Run `cam.py` in Spyder or your preferred Python IDE.

---

## How It Works

1. **Sensor Configuration:**
   - The IMU is initialized via I2C to capture accelerometer and magnetometer data.
   - The camera is configured via SPI for image capture.
2. **Image Processing:**
   - Frames are captured from the camera and processed using OpenCV to compute the center of mass of the detected object.
3. **Motor Control:**
   - Based on the computed center of mass, the motor adjusts to center the object in the frame using the PmodDHB1 dual H-Bridge interface.
4. **User Interaction:**
   - Debugging and system visualization are enabled through Python scripts.

---

## Future Improvements

- Implement advanced algorithms for object tracking.
- Establish a quadratic relationship between the number of pulses according to the object's x position. 

---
