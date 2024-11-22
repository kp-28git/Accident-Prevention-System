# Accident Detection and Reporting System with Alcohol Detection and GPS Tracking

## Overview

This project implements a real-time **Accident Detection and Reporting System** using an **ESP32 microcontroller**. The system detects accidents, alcohol presence, and GPS location, sending SMS alerts and uploading data to **ThingSpeak** for remote monitoring.

Key Features:
- **Accident Detection** using an accelerometer (threshold-based).
- **Alcohol Detection** via an MQ-3 sensor.
- **GPS Tracking** using a SIM808 GPS module.
- **SMS Alerts** sent with accident location and Google Maps link.
- **Real-Time Data Upload** to ThingSpeak for monitoring.
- **WiFi Connectivity** for seamless communication.

---

## Components Used

- **ESP32 Microcontroller**: Handles the logic, sensor readings, and communication.
- **SIM808 GPS Module**: Provides real-time GPS coordinates.
- **MQ-3 Alcohol Sensor**: Detects alcohol concentration.
- **3-Axis Accelerometer**: Measures acceleration in X, Y, and Z directions to detect sudden changes.
- **LED Indicator**: Shows system status (working, accident detected, etc.).
- **ThingSpeak Cloud**: For real-time monitoring of accident data.
- **SMS Functionality**: For sending SMS alerts with the accident location.

---

## Features

### 1. **Accident Detection**
- The system detects an accident based on changes in accelerometer data.
- Threshold values are used to determine if the system needs to trigger an alert.
  
### 2. **Alcohol Detection**
- Alcohol concentration is monitored via the MQ-3 sensor.
- If the detected alcohol concentration exceeds a threshold, an alert is triggered.

### 3. **GPS Tracking**
- The SIM808 GPS module provides latitude and longitude of the detected accident.
- The GPS coordinates are used to create a Google Maps link for the emergency services.

### 4. **SMS Alert**
- When an accident is detected, an SMS is sent to a predefined contact with the accident location and a Google Maps link.
  
### 5. **ThingSpeak Integration**
- Real-time data from sensors (alcohol level, accelerometer values, GPS coordinates) are sent to **ThingSpeak** for remote monitoring.

---

## System Diagram

> (Insert system block diagram or circuit diagram here)

---

## Installation

### Hardware Setup

1. **Connect the MQ-3 Alcohol Sensor** to the ESP32's ADC pin (GPIO 35).
2. **Connect the 3-Axis Accelerometer** to the ESP32 (GPIO 32, GPIO 33, GPIO 34).
3. **Connect the SIM808 GPS Module** to the ESP32 via UART (GPIO 16 and GPIO 17 for RX and TX).
4. **LED Indicator** connected to GPIO 2 for system status.
  
---

### Software Setup

1. **Install the Arduino IDE** and set up the ESP32 board.
   - Go to **File > Preferences > Additional Boards Manager URLs** and add:
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - Open **Tools > Board > Board Manager**, search for ESP32 and install it.

2. **Install the necessary libraries** in Arduino IDE:
   - **TinyGPS++**: For GPS data parsing.
   - **WiFi.h**: For WiFi functionality.
   - **ThingSpeak.h**: To upload data to ThingSpeak.

3. **Modify the Configuration**:
   - Change the WiFi credentials and ThingSpeak API Key in the code.
   - Define the GSM number to receive the SMS alert.

```cpp
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
const char* thingSpeakApiKey = "YourThingSpeakAPIKey";
const int channel_ID = YourChannelID;
