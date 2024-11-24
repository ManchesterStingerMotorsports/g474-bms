
// Source: adbms_application.c (6830) + application.c (2950)

#pragma once


#include "common.h"
#include "adbms6830_data.h"
#include "adbms2950_data.h"

namespace AD68_NS
{
    extern cell_asic IC[TOTAL_IC];

    /**
    *******************************************************************************
    * @brief Setup Variables
    * The following variables can be modified to configure the software.
    *******************************************************************************
    */

    /* ADC Command Configurations */
    extern RD      REDUNDANT_MEASUREMENT;
    extern CH      AUX_CH_TO_CONVERT;
    extern CONT    CONTINUOUS_MEASUREMENT;
    extern OW_C_S  CELL_OPEN_WIRE_DETECTION;
    extern OW_AUX  AUX_OPEN_WIRE_DETECTION;
    extern PUP     OPEN_WIRE_CURRENT_SOURCE;
    extern DCP     DISCHARGE_PERMITTED;
    extern RSTF    RESET_FILTER;
    extern ERR     INJECT_ERR_SPI_READ;

    /* Set Under Voltage and Over Voltage Thresholds */
    extern const float OV_THRESHOLD;                    /* Volt */
    extern const float UV_THRESHOLD;                    /* Volt */
    extern const int OWC_Threshold;                     /* Cell Open wire threshold(mili volt) */
    extern const int OWA_Threshold;                     /* Aux Open wire threshold(mili volt) */
    extern const uint32_t LOOP_MEASUREMENT_COUNT;       /* Loop measurment count */
    extern const uint16_t MEASUREMENT_LOOP_TIME;        /* milliseconds(mS)*/
    extern uint32_t loop_count;
    extern uint32_t pladc_count;

    /*Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS*/
    extern LOOP_MEASURMENT MEASURE_CELL;            /*   This is ENABLED or DISABLED       */
    extern LOOP_MEASURMENT MEASURE_AVG_CELL;        /*   This is ENABLED or DISABLED       */
    extern LOOP_MEASURMENT MEASURE_F_CELL;          /*   This is ENABLED or DISABLED       */
    extern LOOP_MEASURMENT MEASURE_S_VOLTAGE;       /*   This is ENABLED or DISABLED       */
    extern LOOP_MEASURMENT MEASURE_AUX;             /*   This is ENABLED or DISABLED       */
    extern LOOP_MEASURMENT MEASURE_RAUX;            /*   This is ENABLED or DISABLED       */
    extern LOOP_MEASURMENT MEASURE_STAT;            /*   This is ENABLED or DISABLED       */
}


namespace AD29_NS
{
    //// 2950 Application
    /*******************************************************************************
    * @brief Setup Variables
    * The following variables can be modified to configure the software.
    *******************************************************************************/
    /*!< ********************************GLOBAL VARIABLES****************************/
    /*!< ADC Command Configurations */
    extern VCH   VOLTAGE_MEASUREMENT;
    extern ACH   AUX_CH_TO_CONVERT;
    extern OW    OW_WIRE_DETECTION;
    extern RD    REDUNDANT_MEASUREMENT;
    extern CONT  CONTINUOUS_MEASUREMENT;
    extern ERR   INJECT_ERR_SPI_READ;
    extern uint32_t pladc_count;
    /*!< ****************************************************************************/

    /**
    *******************************************************************************
    * @brief Setup Variables
    * The following variables can be modified to configure the software.
    *******************************************************************************
    */

    // #define TOTAL_IC 1
    extern cell_asic IC[TOTAL_IC];
    extern int loop_measurment_count;      /* Loop measurment count (default count)*/
    extern int loop_measurment_time;        /* milliseconds(mS)*/
    extern int loop_count;
    //// 2950 Application
}
