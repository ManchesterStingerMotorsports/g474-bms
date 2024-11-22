
// Source: adbms_application.c (6830) + application.c (2950)



#include "adbms_config.h"
#include "common.h"
#include "adbms6830_data.h"
#include "adbms2950_data.h"


namespace AD68_NS
{
    /**
    *******************************************************************************
    * @brief Setup Variables
    * The following variables can be modified to configure the software.
    *******************************************************************************
    */

    //constexpr uint8_t TOTAL_IC = 1;
    cell_asic IC[TOTAL_IC];

    /* ADC Command Configurations */
    RD      REDUNDANT_MEASUREMENT           = RD_OFF;
    CH      AUX_CH_TO_CONVERT               = AUX_ALL;
    CONT    CONTINUOUS_MEASUREMENT          = SINGLE;
    OW_C_S  CELL_OPEN_WIRE_DETECTION        = OW_OFF_ALL_CH;
    OW_AUX  AUX_OPEN_WIRE_DETECTION         = AUX_OW_OFF;
    PUP     OPEN_WIRE_CURRENT_SOURCE        = PUP_DOWN;
    DCP     DISCHARGE_PERMITTED             = DCP_OFF;
    RSTF    RESET_FILTER                    = RSTF_OFF;
    ERR     INJECT_ERR_SPI_READ             = WITHOUT_ERR;

    /* Set Under Voltage and Over Voltage Thresholds */
    const float OV_THRESHOLD = 4.2;                 /* Volt */
    const float UV_THRESHOLD = 3.0;                 /* Volt */
    const int OWC_Threshold = 2000;                 /* Cell Open wire threshold(mili volt) */
    const int OWA_Threshold = 50000;                /* Aux Open wire threshold(mili volt) */
    const uint32_t LOOP_MEASUREMENT_COUNT = 1;      /* Loop measurment count */
    const uint16_t MEASUREMENT_LOOP_TIME  = 10;     /* milliseconds(mS)*/
    uint32_t loop_count = 0;
    uint32_t pladc_count;

    /*Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS*/
    LOOP_MEASURMENT MEASURE_CELL            = ENABLED;        /*   This is ENABLED or DISABLED       */
    LOOP_MEASURMENT MEASURE_AVG_CELL        = ENABLED;        /*   This is ENABLED or DISABLED       */
    LOOP_MEASURMENT MEASURE_F_CELL          = ENABLED;        /*   This is ENABLED or DISABLED       */
    LOOP_MEASURMENT MEASURE_S_VOLTAGE       = ENABLED;        /*   This is ENABLED or DISABLED       */
    LOOP_MEASURMENT MEASURE_AUX             = DISABLED;        /*   This is ENABLED or DISABLED       */
    LOOP_MEASURMENT MEASURE_RAUX            = DISABLED;        /*   This is ENABLED or DISABLED       */
    LOOP_MEASURMENT MEASURE_STAT            = DISABLED;        /*   This is ENABLED or DISABLED       */
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
    VCH   VOLTAGE_MEASUREMENT     = SM_V1;
    ACH   AUX_CH_TO_CONVERT       = ALL; // conflicting name with above
    OW    OW_WIRE_DETECTION       = OW_OFF;
    RD    REDUNDANT_MEASUREMENT   = RD_OFF;
    CONT  CONTINUOUS_MEASUREMENT  = SINGLE;
    ERR   INJECT_ERR_SPI_READ     = WITHOUT_ERR;
    uint32_t pladc_count;
    /*!< ****************************************************************************/

    /**
    *******************************************************************************
    * @brief Setup Variables
    * The following variables can be modified to configure the software.
    *******************************************************************************
    */

    // #define TOTAL_IC 1
    cell_asic IC[TOTAL_IC];
    int loop_measurment_count = 10;      /* Loop measurment count (default count)*/
    int loop_measurment_time = 1;        /* milliseconds(mS)*/
    int loop_count = 0;

    //// 2950 Application

}
