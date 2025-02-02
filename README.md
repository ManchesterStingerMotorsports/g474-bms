# Battery Management System (BMS)

Otherwise known by Formula Student as Accumulator Management System (AMS)



## Hardware

![alt text](<doc/System Recommendation.png>)
![alt text](<doc/ADI ICs.png>)
![alt text](<doc/Diagnostic Recommendation.png>)

Terminology:

- BMS (or AMS) -> **STM32G474RET6** (Microcontroller)
- BMS Transciever -> **ADBMS6822** (Usually attached on top of the microcontroller)
- BMS Master -> **ADBMS2950** (First device in daisy chain)
- BMS Slave -> **ADBMS6830** (7x of this in the daisy chain)


## Software Design Notes

- C++ for "namespace" feature to avoid variable name conflict
- Due to CubeMX unable to generate main.cpp, to generate code, you have to first rename to main.c, generate code, and then rename it back to main.cpp 


## Operation Modes

Idle
- Cell Balancing: ON
- Sensor Update Rate: Low


Active
- Cell Balancing: OFF
- Sensor Update Rate: High


Charging
- Cell Balancing: ON
- Sensor Update Rate: Medium
- Charger CAN Communication


## TO-DOs

- [ ] Charging
- [ ] BMS Master Measurements
- [ ] Error Flag (Status Register Flags)
- [ ] Error Handling
- [ ] Operation Modes Implementation

## Pinouts and Peripherals

### Pinouts

![alt text](<doc/mcu pinouts.png>)




