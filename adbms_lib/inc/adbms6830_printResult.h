

#pragma once

#include "common.h"
#include "adbms6830_data.h"

namespace AD68_NS
{
    void printWriteConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
    void printReadConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
    void printDeviceSID(uint8_t tIC, cell_asic *IC, TYPE type);
    void printWriteCommData(uint8_t tIC, cell_asic *IC, TYPE type);
    void printReadCommData(uint8_t tIC, cell_asic *IC, TYPE type);
    void printVoltages(uint8_t tIC, cell_asic *IC, TYPE type);
    void printStatus(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
    void printWritePwmDutyCycle(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
    void printReadPwmDutyCycle(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp);
    void printDiagnosticTestResult(uint8_t tIC, cell_asic *IC, DIAGNOSTIC_TYPE type);
    void diagnosticTestResultPrint(uint8_t result);
    void printOpenWireTestResult(uint8_t tIC, cell_asic *IC, TYPE type);
    void openWireResultPrint(uint8_t result);
    float getVoltage(int data);
    void printPollAdcConvTime(int count);

    void printMenu();
}


