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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

//#include "adbms_libWrapper.h"
//#include "adbms_config.h"
//
//#include "adbms_mcuWrapper.h"
//#include "adbms_cmdlist.h"
//#include "adbms_utility.h"
//
//#include "adbms2950_data.h"

#include "bms_cmdlist.h"
#include "bms_datatypes.h"
#include "bms_mcuWrapper.h"
#include "bms_utility.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

uint32_t getRuntimeMs(void);
uint32_t getRuntimeMsDiff(uint32_t startTime);

// ADBMS Experimental Functions
// Array size are unnecessary but helpful to debug

void readDaisyChainSID(void);

void sendCmd(uint8_t cmd[2]);
void sendData(const uint8_t data[6 * TOTAL_IC]);

void readData(uint8_t rxData[6 * TOTAL_IC], uint16_t rxPec[TOTAL_IC], uint8_t rxCc[TOTAL_IC]);
bool checkRxPec(uint8_t rxData[6 * TOTAL_IC], uint16_t rxPec[TOTAL_IC], uint8_t rxCc[TOTAL_IC], bool errorIndex[TOTAL_IC]);

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

  setvbuf(stdin, NULL, _IONBF, 0); //disable buffering for input stream

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
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


    // Set CS2 Pin to HIGH to disable second SPI on 6822 + MSTR should be high by default
    HAL_GPIO_WritePin(BMS_CS2_GPIO_Port, BMS_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMS_MSTR_GPIO_Port, BMS_MSTR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMS_MSTR2_GPIO_Port, BMS_MSTR2_Pin, GPIO_PIN_SET);


//    AD68_NS::adBms6830_init_config(TOTAL_IC, AD68_NS::IC);
//    AD29_NS::adi2950_init_config(TOTAL_IC, AD29_NS::IC);


    HAL_TIM_Base_Start_IT(&htim16);

    char message[50];

    uint32_t timeDiff = 0;
    uint32_t timeStart;

    printf("Start Program \n\n");

    while (1)
    {

        timeStart = getRuntimeMs();


//        AD68_NS::adBms6830_start_adc_cell_voltage_measurment(TOTAL_IC);
//        HAL_Delay(1);
//        AD68_NS::adBms6830_read_cell_voltages(TOTAL_IC, AD68_NS::IC);

//        AD29_NS::adi2950_read_device_sid(TOTAL_IC, AD29_NS::IC);

        readDaisyChainSID();

        timeDiff = getRuntimeMsDiff(timeStart);
        sprintf(message, "Runtime: %ld ms, CommandTime: %ld ms \n\n", getRuntimeMs(), timeDiff);
        HAL_UART_Transmit(&hlpuart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

        HAL_Delay(1000);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 170-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 17000 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMS_CS2_GPIO_Port, BMS_CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMS_MSTR_GPIO_Port, BMS_MSTR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BMS_MSTR2_Pin|BMS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_WAKE_Pin BMS_INT_Pin */
  GPIO_InitStruct.Pin = BMS_WAKE_Pin|BMS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BMS_CS2_Pin */
  GPIO_InitStruct.Pin = BMS_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMS_CS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMS_MSTR_Pin */
  GPIO_InitStruct.Pin = BMS_MSTR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMS_MSTR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_WAKE2_Pin BMS_INT2_Pin */
  GPIO_InitStruct.Pin = BMS_WAKE2_Pin|BMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_MSTR2_Pin BMS_CS_Pin */
  GPIO_InitStruct.Pin = BMS_MSTR2_Pin|BMS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/// printf and scanf compatibility code ///

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)

extern "C"
{
    PUTCHAR_PROTOTYPE
    {
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)(&ch), 1, HAL_MAX_DELAY);
        setbuf(stdout, NULL);
        return ch;
    }

    GETCHAR_PROTOTYPE
    {
        uint8_t ch;
        __HAL_UART_CLEAR_OREFLAG(&hlpuart1);
        HAL_UART_Receive(&hlpuart1, (uint8_t *)(&ch), 1, HAL_MAX_DELAY);
        return (int)ch;
    }
}


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


void readDaisyChainSID(void)
{
    uint8_t  rxBuff_data[6 * TOTAL_IC] = {0};       // 6 Data (not including 2 DPEC) per IC
    uint16_t rxBuff_pec[TOTAL_IC] = {0};            // Data PEC
    uint8_t  rxBuff_cc[TOTAL_IC] = {0};             // Command counter
    bool     errorIndex[TOTAL_IC] = {0};

    bms_wakeupChain();                              // IC wakeup

    bms_csLow();                                                // Start SPI Comms
    bms_spiTransmitCmd(RDSID);                                  // Send command
    bms_spiRecieveData(rxBuff_data, rxBuff_pec, rxBuff_cc);     // read incoming bytes
    bms_csHigh();                                               // End SPI Comms

    bms_checkRxPec(rxBuff_data, rxBuff_pec, rxBuff_cc, errorIndex);

    for(int ic = 0; ic < TOTAL_IC; ic++)
    {
        printf("IC%d: \n", ic+1);
        if (errorIndex[ic])
        {
            printf("SID: ");
            for (int j = 0; j < 6; j++)                     // For every byte recieved (6 bytes)
            {
                printf("0x%02X, ", rxBuff_data[j + ic*6]);  // Print each of the bytes
            }
            printf(" // Command Counter: %d \n", rxBuff_cc[ic]);     // Print command counter
        }
        else // If PEC error
        {
            printf("WARNING! PEC ERROR \n");
        }
    }
}



//
//
//void initBoth(void)
//{
//    adBmsWakeupIc(TOTAL_IC);
//
//
//    using namespace AD29_NS;
//    {
//        for(uint8_t cic = 0; cic < tIC; cic++)
//        {
//            /* Init config A */
//            ic[cic].tx_cfga.refon = PWR_UP;
//
//            /* Init config B */
//            ic[cic].tx_cfgb.vs2 = VSM_SGND;
//        }
//        adBmsWakeupIc(tIC);
//
//        AD29_NS::adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
//        AD29_NS::adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
//    }
//
//
//
//    {
//      for(uint8_t cic = 0; cic < tIC; cic++)
//      {
//        /* Init config A */
//        ic[cic].tx_cfga.refon = PWR_UP;
//    //    ic[cic].cfga.cth = CVT_8_1mV;
//    //    ic[cic].cfga.flag_d = ConfigA_Flag(FLAG_D0, FLAG_SET) | ConfigA_Flag(FLAG_D1, FLAG_SET);
//    //    ic[cic].cfga.gpo = ConfigA_Gpo(GPO2, GPO_SET) | ConfigA_Gpo(GPO10, GPO_SET);
//        ic[cic].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */
//    //    ic[cic].cfga.soakon = SOAKON_CLR;
//    //    ic[cic].cfga.fc = IIR_FPA256;
//
//        /* Init config B */
//    //    ic[cic].cfgb.dtmen = DTMEN_ON;
//        ic[cic].tx_cfgb.vov = SetOverVoltageThreshold(OV_THRESHOLD);
//        ic[cic].tx_cfgb.vuv = SetUnderVoltageThreshold(UV_THRESHOLD);
//    //    ic[cic].cfgb.dcc = ConfigB_DccBit(DCC16, DCC_BIT_SET);
//    //    SetConfigB_DischargeTimeOutValue(tIC, &ic[cic], RANG_0_TO_63_MIN, TIME_1MIN_OR_0_26HR);
//      }
//      adBmsWakeupIc(tIC);
////      adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
//      for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
//      {
//        ic[curr_ic].configa.tx_data[0] = (((ic[curr_ic].tx_cfga.refon & 0x01) << 7) | (ic[curr_ic].tx_cfga.cth & 0x07));
//        ic[curr_ic].configa.tx_data[1] = (ic[curr_ic].tx_cfga.flag_d & 0xFF);
//        ic[curr_ic].configa.tx_data[2] = (((ic[curr_ic].tx_cfga.soakon & 0x01) << 7) | ((ic[curr_ic].tx_cfga.owrng & 0x01) << 6) | ((ic[curr_ic].tx_cfga.owa & 0x07) << 3));
//        ic[curr_ic].configa.tx_data[3] = ((ic[curr_ic].tx_cfga.gpo & 0x00FF));
//        ic[curr_ic].configa.tx_data[4] = ((ic[curr_ic].tx_cfga.gpo & 0x0300)>>8);
//        ic[curr_ic].configa.tx_data[5] = (((ic[curr_ic].tx_cfga.snap & 0x01) << 5) | ((ic[curr_ic].tx_cfga.mute_st & 0x01) << 4) | ((ic[curr_ic].tx_cfga.comm_bk & 0x01) << 3) | (ic[curr_ic].tx_cfga.fc & 0x07));
//      }
//
//
//      adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
//    }
//
//}



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
