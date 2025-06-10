/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

#include "bms_cmdlist.h"
#include "bms_datatypes.h"
#include "bms_utility.h"
#include "bms_mcuWrapper.h"
#include "bms_libWrapper.h"

#include "uartDMA.h"
#include "bms_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


const float deltaThreshold = 0.010; // In volts


typedef enum
{
    ACTIVE,
    BALANCING,
    CHARGING,
    INACTIVE,
    IDLE,
} BmsStates;

volatile BmsStates bmsState;         // Controlled by ISR
volatile BmsStates bmsCurrState;
volatile BmsStates bmsPrevState;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint32_t getRuntimeMs(void);
uint32_t getRuntimeMsDiff(uint32_t startTime);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint32_t runtime_sec = 0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


    // Set CS2 Pin to HIGH to disable second SPI on 6822 + MSTR should be high by default
    HAL_GPIO_WritePin(BMS_CS2_GPIO_Port, BMS_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMS_MSTR_GPIO_Port, BMS_MSTR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMS_MSTR2_GPIO_Port, BMS_MSTR2_Pin, GPIO_PIN_SET);

    // Start Timer16
    HAL_TIM_Base_Start_IT(&htim16);

    // Configure FDCAN
    BMS_CAN_Config();

//    // Initialise BMS configs (No commands sent)
//    bms_init();

    uint32_t timeDiff = 0;
    uint32_t timeStart;

    printfDma("\n\n +++++             PROGRAM START             +++++ \n\n");

    bms_softReset();
    HAL_Delay(200);         // Initialisation delay

    bms_wakeupChain();
    bms_init();             // Initialise BMS configs and send them
    bms_readSid();

    bmsState = IDLE;
    bmsPrevState = IDLE;
    bmsCurrState = IDLE;



    while (1)
    {
        bmsCurrState = bmsState;        // Copy value to ensure value is not changed throughout the loop

        if (bmsPrevState != bmsCurrState)
        {
            bms_wakeupChain();
            bms_init();             // Initialise BMS configs and send them
            bms_readSid();
            bms_startAdcvCont();            // Need to wait 8ms for the average register to fill up
            bms29_setGpo();

            if (bmsState == ACTIVE)
            {
                // Measure the current cell voltage first
//                printfDma("Measuring Cell Voltage: \n");
//                bms_wakeupChain();              // Wakeup needed every 4ms of Inactivity
//                bms_startAdcvCont();            // Need to wait 8ms for the average register to fill up
//                bms_delayMsActive(12);
//                bms_readAvgCellVoltage();
//
//                bms_startBalancing(deltaThreshold); // Uses the current C value, so need to measure the cell voltage first

//                // Calculate the discharge threshold
//                float discharge_threshold = bms_calculateBalancing(deltaThreshold);
//
//                // Check if need to balance the cells
//                if (discharge_threshold > 0)
//                {
//                    printfDma("Start Discharge: \n");
//                    bms_wakeupChain();
//                    bms_startDischarge(discharge_threshold);
//
//                    bms_wakeupChain();              // Wakeup needed every 4ms of Inactivity
//                    bms_startAdcvCont();            // Need to wait 8ms for the average register to fill up
//                    bms_delayMsActive(12);
//                }

//                HAL_Delay(1000);
            }
            bmsPrevState = bmsCurrState;
        }


        switch(bmsCurrState)
        {
        case ACTIVE:

            timeStart = getRuntimeMs();

            printfDma("C Voltage: \n");
            bms_wakeupChain();              // Wakeup needed every 4ms of Inactivity
            bms_startAdcvCont();            // Need to wait 8ms for the average register to fill up
            bms_delayMsActive(12);
            bms_readAvgCellVoltage();
            bms_readSVoltage();
//
//            HAL_Delay(100);
//            bms_wakeupChain();
//            printfDma("======== C VOLTAGE MEASUREMENT ======== \n");
//            for (int i = 0; i < 3; i++)
//            {
//                bms_readAvgCellVoltage();
//                bms_delayMsActive(50);
//            }
//            printfDma("======================================= \n\n");

            bms_wakeupChain();
            printfDma("Single Shot S Voltage (PWM Interrupted): \n");
            bms_balancingMeasureVoltage();


            printfDma("Temp Measurements: \n");
            bms_getAuxMeasurement();

////            bms_startAdcvCont();            // Need to wait 8ms for the average register to fill up
////            bms_delayMsActive(12);
//
            bms_wakeupChain();
            bms_startBalancing(deltaThreshold);
//            HAL_Delay(500);
//
////            bms_wakeupChain();
////            bms_readSid();
//
//            bms_wakeupChain();
//            bms29_readVB();
//            bms29_readCurrent();

            BMS_CAN_Test();

            HAL_Delay(1000);

            timeDiff = getRuntimeMsDiff(timeStart);
            printfDma("Runtime: %ld ms, CommandTime: %ld ms \n\n", getRuntimeMs(), timeDiff);

            break;

        case INACTIVE:

            bms_stopDischarge();
            bmsState = IDLE;
            HAL_Delay(500);
            break;

        default:
            break;
        }

        HAL_Delay(1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */





/// Timer interrupt callback
// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Check which version of the timer triggered this callback and toggle LED
    if (htim == &htim16)
    {
        runtime_sec += 1;
    }
}




uint32_t getRuntimeMs(void)
{
    return HAL_GetTick();
}


uint32_t getRuntimeMsDiff(uint32_t startTime)
{
    return HAL_GetTick() - startTime; // Divide 10 to get 10ms
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == B1_Pin) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
        // Do something when button pressed
        if (bmsState == ACTIVE)
        {
            bmsState = INACTIVE;
        }
        else
        {
            bmsState = ACTIVE;
        }
    }
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
