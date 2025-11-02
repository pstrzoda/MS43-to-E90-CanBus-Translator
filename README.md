# MS43 to E90 CAN Bus Translator

A CAN bus translator project that enables communication between BMW MS43 DME (Digital Motor Electronics) and E90 series dashboard/body control systems. This project acts as a bridge, translating CAN messages between the two different protocols to maintain proper functionality when swapping engine management systems.

## 🎯 Project Overview

This translator allows you to use an MS43 DME (typically from E46/E39 models) with an E90 series vehicle while maintaining dashboard functionality, error reporting, and system integration. The project handles:

- **Engine Data Translation**: RPM, temperature, torque, and other engine parameters
- **Error Management**: Translates and manages error codes between systems
- **Dashboard Integration**: Maintains proper gauge functionality and warning lights
- **Safety Features**: Automatic fan control and overheat protection

## 🔧 Hardware Requirements

### Microcontroller
- **Teensy 4.0** or **Teensy 4.1** (recommended)
- Dual CAN bus support via FlexCAN_T4 library

### CAN Interfaces
- **CAN1**: Connected to E90 vehicle CAN bus
- **CAN2**: Connected to MS43 DME CAN bus
- CAN transceivers (e.g., MCP2551 or TJA1050)

### Additional Components
- **Fan Control**: Pin 19 for cooling fan relay control
- **Status LED**: Pin 13 (built-in LED) for system status indication
- **Input Sensors**:
  - Pin 17: Battery/Alternator light input
  - Pin 16: Oil pressure switch input
  - Pin 15: KL15 (ignition) input

### Wiring Connections
```
Teensy Pin | Function           | Connection
-----------|-------------------|------------------
13         | Status LED        | Built-in LED
15         | KL15 Input        | Ignition switch (+12V)
16         | Oil Pressure      | Oil pressure switch (GND when low)
17         | Battery Light     | Alternator warning (+12V when fault)
19         | Fan Control       | Cooling fan relay control
22         | CAN1 RX           | E90 CAN bus (via transceiver)
23         | CAN1 TX           | E90 CAN bus (via transceiver)
30         | CAN2 RX           | MS43 DME CAN bus (via transceiver)
31         | CAN2 TX           | MS43 DME CAN bus (via transceiver)
```


## 📊 CAN Message Translation

### Outgoing Messages (to E90)
- **0x0A8**: Torque control data (10ms interval)
- **0x0A9**: Torque limits and status (10ms interval)
- **0x0AA**: Engine RPM and torque data (10ms interval)
- **0x1D0**: Engine temperature and status (200ms interval)
- **0x3B4**: Battery voltage and engine status (4s interval)
- **0x592**: Error/warning messages (8s interval for active errors)

### Incoming Messages (from MS43)
- **0x316**: Engine RPM and torque
- **0x329**: Water temperature, throttle position, clutch/brake status
- **0x720**: Intake/exhaust/oil temperatures, battery voltage, speed
- **0x545**: Engine error states and warning flags

## 🚨 Error Management

The system monitors and translates various error conditions:

- **Engine Overheating** (0x27): Water temperature > 108°C
- **Check Engine** (0x1F): DME detected fault
- **EML Light** (0x1E): Engine management light
- **Low Oil Level** (0xDA): Oil level sensor
- **Low Oil Pressure** (0xDB/0xD4): Oil pressure monitoring
- **Alternator Failure** (0xD5): Charging system fault
- **Communication Error** (0x99): DME CAN timeout

## 💡 Status LED Patterns

The built-in LED provides visual system status:

- **Slow Blink** (1s on/off): Normal operation, DME communication active
- **Fast Blink** (200ms on/off): DME communication error/timeout

## 🌡️ Safety Features

### Automatic Fan Control
- **Fan ON**: Water temperature ≥ 105°C
- **Fan OFF**: Water temperature ≤ 97°C
- **Fail-safe**: Fan automatically activates if DME communication is lost and ignition is on

### Temperature Monitoring
- Continuous monitoring of engine coolant temperature
- Automatic overheating warnings to dashboard
- Safe temperature threshold management

## 🔧 Configuration

### Key Parameters (configurable in code)
```cpp
#define MAX_ENGINE_TORQUE 360           // Maximum engine torque (Nm)
#define OVERHEAT_THRESHOLD 108          // Overheating warning (°C)
#define SAFE_TEMP_THRESHOLD 90          // Safe temperature (°C)
#define FAN_ON_TEMP_THRESHOLD 105       // Fan activation (°C)
#define FAN_OFF_TEMP_THRESHOLD 97       // Fan deactivation (°C)
#define CAN2_TIMEOUT_PERIOD 10000       // DME timeout (ms)
```

### Serial Monitoring
Connect to serial monitor (9600 baud) for diagnostic information:
```
Temp: 85 RPM: 1250 Torque: 45.6 Engine Running: 1
```

## 📝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/improvement`)
5. Create a Pull Request

## ⚠️ Disclaimer

This project is for educational and experimental purposes. Use at your own risk. Always ensure proper safety measures when working with automotive systems. The authors are not responsible for any damage to vehicles or injury resulting from the use of this code.

## 🤝 Support

For questions, issues, or contributions, please open an issue on GitHub or contact the project maintainers.
