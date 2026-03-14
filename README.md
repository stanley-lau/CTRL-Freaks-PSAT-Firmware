# CTRL Freaks PSAT Flight Computer Firmware

## Overview
This repository contains the firmware for the on-board flight computer developed by Team CTRL Freaks for the PSAT mission.

## Features
- Runs on **MSP430FR2355** microcontroller  
- Sensor interfaces:  
  - **BMP390L** – Barometric pressure sensor  
  - **ICM-42670-P** – 6-axis IMU (accelerometer + gyroscope)  
- Monitors multiple analog channels for:  
  - Temperature  
  - Current  
- Communication with **beacon board** for:  
  - **GPS data acquisition**  
  - **LoRa radio communication**


## Build Instructions

1. Open the project in **Code Composer Studio (CCS)**  
2. Select the **MSP430FR2355** target  
3. Build the project: Project → Build Project
4. Flash firmware to the board using CCS