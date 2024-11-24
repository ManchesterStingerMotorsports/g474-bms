


#pragma once
#include "common.h"
#include "adbms2950_data.h"

namespace AD29_NS
{
    void printWriteConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
    void printReadConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
    void printDeviceSID(uint8_t tIC, cell_asic *IC, TYPE type);
    void printWriteCommData(uint8_t tIC, cell_asic *IC, TYPE type);
    void printReadCommData(uint8_t tIC, cell_asic *IC, TYPE type);
    void printCr(uint8_t tIC, cell_asic *IC);
    void printVoltage(uint8_t tIC, cell_asic *IC, TYPE type);
    void printVbat(uint8_t tIC, cell_asic *IC);
    void printIvbat(uint8_t tIC, cell_asic *IC);
    void printAvgVbat(uint8_t tIC, cell_asic *IC);
    void printAvgIVbat(uint8_t tIC, cell_asic *IC);
    void printAvgCr(uint8_t tIC, cell_asic *IC);
    void printOc(uint8_t tIC, cell_asic *IC);
    void printStatus(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
    void printMsg(char *msg);
    void printResultCount(int count);
    void readUserInupt(int *user_command);
    float getCurrent(uint32_t data);
    float getAvgCurrent(uint32_t data);
    float getAvgVbat(uint32_t data);
    float getOverCurrent(uint8_t data);

    float getVoltage(int data);
    void printPollAdcConvTime(int count);

    void printMenu(void);
}
