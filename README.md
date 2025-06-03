# Wave Rover - Navigation App

This application is part of the bigger project - the Wave Rover, a 4WD mobile robot with remote control, AI vision, navigation, various manipulators and more.

This particular app implements the navigation part of the robot and runs on its own ESP32 board.

## Hardware

- **Microcontroller:** ESP-WROOM-32 (ESP32 Dev Kit V1 board)
- **Sensors:** ublox NEO-M8N

## Software and Framework

- **Framework:** ESP-IDF (Espressif IoT Development Framework)
- **Language:** C
- **Operating System:** FreeRTOS (as part of ESP-IDF)

## Functionalities

### Core Navigation Features
- **GNSS Data Reception**: Receives and processes NMEA 0183 data from u-blox NEO-M8N GPS module via UART
- **Multi-Constellation Support**: Supports GPS, GLONASS, Galileo, and BeiDou navigation systems (GN, GP, GL, GA sentence types)
- **Real-time Position Tracking**: Continuously parses GGA (Global Positioning System Fix Data) sentences to extract:
  - Latitude and longitude coordinates in decimal degrees
  - Altitude above sea level in meters
  - UTC time stamps
  - Fix quality indicators (0-6 scale including RTK support)
  - Number of satellites in use
  - Horizontal Dilution of Precision (HDOP)
  - Geoidal separation values

### Communication & Data Transmission
- **ESP-NOW Wireless Communication**: Transmits navigation data wirelessly to other ESP32 devices
  - Operates on fixed WiFi channel (channel 9)
  - Sends coordinate data every 1 second
  - Configured for unicast communication to Wave Rover Driver ESP32
  - MAC address: `D8:13:2A:2F:3C:E4` (receiver device)

### Data Processing & Management
- **NMEA Sentence Processing**: 
  - Handles multiple NMEA sentence types (up to 5 concurrent sentences)
  - Validates sentence integrity with checksum verification
  - Converts NMEA coordinate format (DDMM.MMMMM/DDDMM.MMMMM) to decimal degrees
  - Processes directional indicators (N/S for latitude, E/W for longitude)

### System Architecture
- **Multi-tasking FreeRTOS Design**:
  - Main application task for system monitoring
  - Dedicated UART RX task for continuous GPS data reception
  - Navigation logging task for periodic data transmission
- **Component-based Architecture**:
  - `GNSS_ublox`: Handles GPS module communication and data parsing
  - `NaviLogging`: Manages ESP-NOW communication and data transmission
  - `Common`: Shared utilities and logging functions

### Hardware Interface
- **UART Communication**: 
  - UART2 interface at 9600 baud rate
  - GPIO16 (RX) and GPIO17 (TX) pins
  - 1024-byte receive buffer for continuous data reception
  - 20ms timeout for data reception

### Monitoring & Diagnostics
- **System Information Display**: Shows ESP32 chip details, flash size, and memory usage
- **Real-time Logging**: Comprehensive logging of GPS coordinates, system status, and communication events
- **Error Handling**: Graceful handling of communication failures and invalid data