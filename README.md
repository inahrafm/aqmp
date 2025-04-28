# Portable Air Quality Monitoring Device (PM2.5)

This Portable Air Quality Monitoring device is designed to measure real-time air quality, focusing on fine particulate matter (PM2.5). Its compact design allows mounting on two-wheeled vehicles like motorcycles or bicycles, enabling monitoring even in narrow alleys and hard-to-reach areas.

## Features
- **Real-Time PM2.5 Measurement:** Uses the GP2Y1010AU0F sensor to accurately detect fine particulate matter.
- **Dual Data Logging:** Logs PM2.5 sensor data along with GPS location to a MicroSD card as backup.
- **WiFi Data Transmission:** Sends data via WiFi to an online dashboard for real-time viewing.
- **ESP32 Module:** Efficient processing and WiFi connectivity.
- **Long Battery Life:** Powered by a 3000mAh battery supported by a mini solar panel for extended operation without frequent charging.
- **Portable and Compact:** Easily mounts on motorcycles or bicycles, ideal for urban areas including narrow streets inaccessible to traditional monitors.

## Components
- ESP32 (microcontroller with WiFi)
- GP2Y1010AU0F PM2.5 sensor
- MicroSD module for data storage
- GPS module for location tracking
- 3000mAh battery and mini solar panel

## Benefits
- Enables local authorities, researchers, and communities to map air pollution in urban spaces.
- Provides accurate, real-time air quality data in hard-to-reach locations.
- Supports data-driven decisions for air pollution control.

## Installation
Steps to install the project:
1. Mount the device securely on a two-wheeled vehicle (motorcycle or bicycle).
2. Ensure the battery is charged and the solar panel is properly connected.
3. Power on the device; it will start measuring and transmitting data automatically.
4. Access the online dashboard via WiFi to monitor air quality data in real-time.

## Installation
Steps to install the project:
1. Ensure you have the necessary hardware (temperature sensors, GPS module, and LoRa gateway).
2. Clone this repository:
   ```bash
   git clone https://github.com/inahrafm/aqmp.git
