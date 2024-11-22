# Battery Management System (BMS)

Otherwise known by Formula Student as Accumulator Management System (AMS)



## Hardware

![alt text](<doc/System Recommendation.png>)
![alt text](<doc/ADI ICs.png>)
![alt text](<doc/Diagnostic Recommendation.png>)

Our terminology:

- BMS or AMS -> **STM32G474RET6** (Microcontroller)
- BMS Transciever -> **ADBMS6822** (Usually attached on top of the microcontroller)
- BMS Master -> **ADBMS2950** (First device in daisy chain)
- BMS Slave -> **ADBMS6830** (7x of this in the daisy chain)


## Software Design Notes

- C++ for "namespace" feature to avoid variable name conflict
- Due to CubeMX unable to generate main.cpp, to generate code, you have to first rename to main.c, generate code, and then rename it back to main.cpp 


## Testing and Progress

Currently only tested for 6830 cell voltage Measurement



## TO-DOs

- [x] task1
- [ ] task2
- [ ] task3
- [ ] task4


## Pinouts and Peripherals

### Pinouts

![alt text](<doc/mcu pinouts.png>)




