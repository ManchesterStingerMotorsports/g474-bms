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

*Making notes here because I have already forgot some of it just after a month.*

Please read this before going through the code, and **READ THE DATASHEET** for more info

**GENERAL**
- UART Baud Rate = 1000 kbit/s

**IsoSPI**
- Timeout
    - IsoSPI timeout Duration: 4.3 ms
    - If IsoSPI sleeps, need to wake up isoSPI by sending CS pulse for every device in chain before sending commands
- Interfacing
    - isoSPI can be interfaced using ADBMS6822 with SPI
    - ADBMS6822 will behave like a typical SPI shift register
- Commands
    - There is no way to send a command to a specific device in a daisy chain
        - So the master and slave will get the same commands
    - Readall are not possible with all devices in the chain.
        - The first device (master) might be able to, but no plans of adding them for now

**ADBMS2950**
- Core States
    - Standby
        - Unlike 6830 this state is a temporary state and will automatically transition to Refup
        - ADC won't trigger in this state
        - REFUP bit can only be read (6830 can be written)
    - Refup
        - OCxADC are set into operation (if OCEN = 1)
    - Measure
        - Enters Measure state after recieving atleast one 
        ADI, ADI2, ADV or ADX command from within REFUP State 
        - Remains in the MEASURE state until power is removed or the SRST command is sent.

**ADBMS6830**
- Core States
    - Sleep
        - State after reset or 1.8s of inactivity
        - Should not be in this state in any case other than reset
        - Will reset all config registers (safe state)
    - Standby
        - Default standby state
    - Refup
        - Basically same as Standby if REFON = 1
    - Extended Balancing
        - Basically same as Standby if DCTO = 1
        - Default state if discharging is ON
    - Measure
        - State where ADC conversion is happening
        - Measurements are usually continous so it will mostly be in this state
    - LPCM Modes
        - Primarily for very low power application
        - extra complicated, so not used for now.
    - DTM
        - Used for low power standby + Balancing
        - No plans of using it currently
- Balancing
    - DTM state will not be used for simplicity and safety
    - Instead, a command will be sent every second manually. The commands:
        - measure the voltages
        - measure temperature
        - set which cells to discharge
        - reset DCTO


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




