/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

// Authors: Christian Valles, Armando Rivero, Sebastiano Marinelli
// Date of modification: 2/09/2022
// Version 1.3

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define timer_freq 100000 // in [Hz]
#define VCP_BUFF_SIZE 100000

#define EFFECTIVE_PIXELS 1728
#define TOTAL_PIXELS EFFECTIVE_PIXELS *3 //5184
#define INATIVE_PIXELS 38+1+1  // 38 are inactive 2 are found during measurements
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// VCP related variables
uint8_t bufferVCP_Rx[VCP_BUFF_SIZE];
char msgVCP[VCP_BUFF_SIZE];
int msgVCP_len = 0;
char* token;
const char *tokenSeparator = ".\n\r";
char strCMD_1[] = "SingleSample";
char strCMD_2[] = "cfg";
char strCMD_2_1[] = "exp";
char strCMD_2_2[] = "gain";
char strCMD_2_3[] = "RLED";
char strCMD_2_4[] = "GLED";
char strCMD_2_5[] = "BLED";

char enableSampleTimerFlag = 0;

unsigned int encoder_tick = 0, pixel_values[5184] = {0};
unsigned int LEDR_time, LEDG_time, LEDB_time, exposureTime_phase, LEDR_phase, LEDG_phase, LEDB_phase, LEDR_residual, LEDG_residual, LEDB_residual;
unsigned char get_image = 0, send_data_main = 0;
float encoder_resolution, encoder_distance;

uint32_t odr;

//-Counters for the clock generation
short int CP_cnt = 0;
short int Tim2_tick = 0;
short int tick_SP = 0;
short int ADC_cnt = 0;

// EXPOSURE TIME
int exposureVal = 120; // in microseconds
int expDuration_10us = 0 ;

int i = 0;
int alternateByte = 0;
int reg_High[TOTAL_PIXELS];
int reg_Low[TOTAL_PIXELS];
int reg_High_R[TOTAL_PIXELS];
int reg_Low_R[TOTAL_PIXELS];
int ADC_B0 = 0;
int ADC_B1 = 0;
int ADC_B2 = 0;
int ADC_B3 = 0;
int ADC_B4 = 0;
int ADC_B5 = 0;
int ADC_B6 = 0;
int ADC_B7 = 0;


//-Flags
short int flag_ExpAndRead = 1;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  expDuration_10us = 6*(int)(exposureVal/60);

  unsigned int exposure_time;

  // AD9826 instructions - Serial communication (SPI)
  const unsigned int config_reg =  0b0000000011101000;
  //const unsigned int MUX_config_reg = 0b000100001110000;
  const unsigned int MUX_config_reg = 0b0001000011110000;

  unsigned int red_PGA_reg, green_PGA_reg, blue_PGA_reg, red_offset_reg, green_offset_reg, blue_offset_reg;
  unsigned int offset_red, offset_green, offset_blue;
  unsigned char gain_red, gain_green, gain_blue;

  uint8_t singlePx_value[TOTAL_PIXELS*2]; //*2 since the USB 8 bit at a time
  unsigned int dataToSend = 0;
  int idxUSB,idx;
  /* Data settings */
  /***********************************************************************************************************/

  encoder_resolution = 0.1;   // in [um]
  encoder_distance = 0.5; // in [um] -> (0.5 [um] = 5 tick)

  // Timing - Integration time of the CIS
  // (1) Resolution of 10 [us] [it depends on the timer frequency (1/timer_freq)]
  // (2) Maximum error of 50 [us] [it depends on the time difference respect to the rising edge of the CDSCLK2 (CP) signal]
  exposure_time = 1000;   // in [us]

  // Timing - LED duration
  // (1) Resolution of 10 [us] [it depends on the timer frequency (1/timer_freq)]
  LEDR_time = 0;  // in [us]
  LEDG_time = 0;  // in [us]
  LEDB_time = 0;  // in [us]

  // Input gain - From 0 (gain = 1) to 63 (gain = 6)
  gain_red = 0;
  gain_green = 0;
  gain_blue = 0;

  // Input offset - From 0 (0 [mV]) to 255 (+300 [mV]) or from 256 (0 [mV]) to 511 (-300 [mV])
  offset_red = 0;
  offset_green = 0;
  offset_blue = 0;

  /***********************************************************************************************************/

  // ADC gain
  red_PGA_reg = 0b0010000000000000 | gain_red;
  green_PGA_reg = 0b0011000000000000 | gain_green;
  blue_PGA_reg = 0b0100000000000000 | gain_blue;

  // ADC offset
  red_offset_reg = 0b0101000000000000 | offset_red;
  green_offset_reg = 0b0110000000000000 | offset_green;
  blue_offset_reg = 0b0111000000000000 | offset_blue;

  // Number of multiplexing phases
  exposureTime_phase = (unsigned int)(((exposure_time / 1000000.0) * timer_freq) / 6.0);
  LEDR_phase = (unsigned int)(((LEDR_time / 1000000.0) * timer_freq) / 6.0);
  LEDG_phase = (unsigned int)(((LEDG_time / 1000000.0) * timer_freq) / 6.0);
  LEDB_phase = (unsigned int)(((LEDB_time / 1000000.0) * timer_freq) / 6.0);

  // Number of residual ticks
  LEDR_residual = (unsigned int)((LEDR_time / 1000000.0) * timer_freq) % 6 + 1;
  LEDG_residual = (unsigned int)((LEDG_time / 1000000.0) * timer_freq) % 6 + 1;
  LEDB_residual = (unsigned int)((LEDB_time / 1000000.0) * timer_freq) % 6 + 1;

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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* Setting transmission to AD9826 */

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1,(uint8_t *)&config_reg, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&MUX_config_reg, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&red_PGA_reg, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&green_PGA_reg, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&blue_PGA_reg, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&red_offset_reg, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&green_offset_reg, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&blue_offset_reg, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

  //HAL_TIM_Base_Start_IT(&htim2);    // Start timer 2

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int msgVCP_len = 0;
  char msgVCP[VCP_BUFF_SIZE];
  while (1)
  {
    // tokenize command recevied in CDC_Receive_FS(...) (file: usbd_cdc_ig.c)
        token = strtok((char*)bufferVCP_Rx,tokenSeparator);
        if (token != NULL)
        {
          if(strcmp(token,strCMD_1) == 0)
          {
            memset (bufferVCP_Rx, '\0', 64);  // clear the VCP buffer always after receiving

            enableSampleTimerFlag = 1;
            HAL_TIM_Base_Start_IT(&htim2);    // Start timer 2


            GPIOE->BSRR = GPIO_PIN_1;  // On-board Yellow LED Set to "1"

            // Toggle on board Yellow LED
    //        odr = GPIOE->ODR;
    //        GPIOE->BSRR = ((odr & GPIO_PIN_1) << 16U) | (~odr & GPIO_PIN_1);
          }
          if(strcmp(token,strCMD_2) == 0)
          {
            token = strtok(NULL,tokenSeparator);
            if(strcmp(token,strCMD_2_1) == 0)
            {
              token = strtok(NULL,tokenSeparator);
//              msgVCP_len = sprintf(msgVCP, "exposure time is %s\n",token);
//              CDC_Transmit_HS((uint8_t *)msgVCP, msgVCP_len);
              exposureVal = atoi(token);
              expDuration_10us = 6*(int)(exposureVal/15);
            }
            memset (bufferVCP_Rx, '\0', 64);  // clear the VCP buffer always
          }
        }



        if(send_data_main)
        {
          GPIOE->BSRR = (uint32_t)GPIO_PIN_1 << 16U; // On-board Yellow LED Set to "0"

          HAL_TIM_Base_Stop_IT(&htim2);
          dataToSend = 0;
          idxUSB = 0;
          idx = 0;


          while(dataToSend == 0)
          {
            //-CH1 (red)
            singlePx_value[idxUSB]   = reg_High[idx];
            singlePx_value[idxUSB+1] = reg_Low[idx];
            //-CH2 (Green)
            singlePx_value[idxUSB+2] = reg_High[idx+1];
            singlePx_value[idxUSB+3] = reg_Low[idx+1];
            //-CH3 (Blue)
            singlePx_value[idxUSB+4] = reg_High[idx+2];
            singlePx_value[idxUSB+5] = reg_Low[idx+2];

            idxUSB = idxUSB + 6;
            idx    = idx + 3;

            if(idx >= TOTAL_PIXELS)
            {
              dataToSend = 1;
            }

          }
          //singlePx_value[TOTAL_PIXELS - 2] = (uint8_t) "\r";
          //singlePx_value[TOTAL_PIXELS - 1] = (uint8_t) "\n";
          __NOP();
          CDC_Transmit_HS(singlePx_value, TOTAL_PIXELS * 2);
          //while(USBD_BUSY);

          //HAL_Delay(300);
          //memset(singlePx_value,0,TOTAL_PIXELS * 2);
          //CDC_Transmit_HS((uint8_t *)"\n", 1);


          //CDC_Transmit_HS((uint8_t *) "\r\n", 1);
          HAL_TIM_Base_Start_IT(&htim2);
          /*HAL_TIM_Base_Stop_IT(&htim2);   // Stop interrupt of TIM2

              memset(msgVCP, '\0', msgVCP_len);  // Clear the buffer

              for(unsigned int i = 0; i < 5184; i++) msgVCP_len += sprintf(&msgVCP[msgVCP_len], "%u ", pixel_values[i]);
              msgVCP_len += sprintf(&msgVCP[msgVCP_len], "\r\n");

              while((CDC_Transmit_HS((uint8_t *)msgVCP, msgVCP_len)) == USBD_BUSY); // Data transmission
              msgVCP_len = 0;

              HAL_TIM_Base_Start_IT(&htim2);    // Start interrupt of TIM2
           */
          send_data_main = 0;
        }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
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
  htim2.Init.Prescaler = 343;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DRV_GREEN_Pin|DRV_RED_Pin|DRV_BLUE_Pin|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TEST_PIN_Pin|PIN_TEST1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_FS_PWR_EN_Pin|SP_Pin|CDSCLK2_Pin|ADCCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DRV_GREEN_Pin DRV_RED_Pin DRV_BLUE_Pin LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = DRV_GREEN_Pin|DRV_RED_Pin|DRV_BLUE_Pin|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE6 PE7 PE8
                           PE10 PE12 PE15 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin ENCODER_B_Pin ENCODER_Z_Pin */
  GPIO_InitStruct.Pin = B1_Pin|ENCODER_B_Pin|ENCODER_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3
                           PF4 PF5 PF6 PF8
                           PF10 PF11 PF12 PF13
                           PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : TEST_PIN_Pin PIN_TEST1_Pin */
  GPIO_InitStruct.Pin = TEST_PIN_Pin|PIN_TEST1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC2 PC3 PC6
                           PC7 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_MDC_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_MDC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA3 PA4 PA6
                           PA7 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB11
                           PB12 PB15 PB4 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG2 PG3 PG4
                           PG5 PG6 PG8 PG9
                           PG10 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_TEST2_Pin */
  GPIO_InitStruct.Pin = PIN_TEST2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_TEST2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D2_Pin D4_Pin D3_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin|D4_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_VCP_RX_Pin STLK_VCP_TX_Pin */
  GPIO_InitStruct.Pin = STLK_VCP_RX_Pin|STLK_VCP_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_PWR_EN_Pin SP_Pin CDSCLK2_Pin ADCCLK_Pin
                           SPI1_CS_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin|SP_Pin|CDSCLK2_Pin|ADCCLK_Pin
                          |SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD15 PD0 PD1 PD2
                           PD3 PD4 PD5 PD6
                           PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_A_Pin */
  GPIO_InitStruct.Pin = ENCODER_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D5_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_8) // If The INT Source Is EXTI Line8 (pin PC8)
  {
    encoder_tick++;
    if(encoder_tick == (unsigned int)(encoder_distance/encoder_resolution))
    {
      encoder_tick= 0;
      get_image = 1;

      // Test pin (PF7)
      // odr = GPIOF->ODR;
      // GPIOF->BSRR = ((odr & GPIO_PIN_7) << 16U) | (~odr & GPIO_PIN_7);
    }
  }
}

//-Tick every 10[uS]
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(enableSampleTimerFlag==1)
  {
    //---ADC_Clk: To generate a 50KHz frequency (ADCCLK - pin PD13)
    odr = GPIOD->ODR;
    GPIOD->BSRR = ((odr & GPIO_PIN_13) << 16U) | (~odr & GPIO_PIN_13);

//    odr = GPIOD->ODR;
//    GPIOD->BSRR = ((odr & GPIO_PIN_12) << 16U) | (~odr & GPIO_PIN_12);


    //---Toggle CDSCLK2 and CP_Clk: To generate the frequency for the ADC Sampling  and the readout (CDSCLK2 and CP_Clk - pin PD12)
    if(Tim2_tick == 0)
    {
      GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16U; // CP_Clk set to 0
    }

    // The CP clock is always 6 ticks (4 at low level, 2 at high level) non symetrical clock
    // since this signal is used for the AFE as CDSCLK2
    if (Tim2_tick == 5)
    {
      GPIOD->BSRR = GPIO_PIN_12;                  // CP_Clk set to 1
      Tim2_tick = 0;
      CP_cnt++;
    }
    else
    {
      Tim2_tick++;
    }



    ADC_cnt++;

    //---CIS SP signal to set exposure duration (SP_CIS - pin PD11)
    // sensor starts acquiring immediately at the tick=2 of the timer counter
    // The Integration time is defined by the 'expDuration_10us' variable
    // there is no reading of the data from the sensor between these two pulses
    if(tick_SP == 2 || tick_SP == (expDuration_10us+2) )
    {
      GPIOD->BSRR = GPIO_PIN_11;              //-Set GPIO to 1
    }
    else if(tick_SP == 7 || tick_SP == (expDuration_10us+7) )
    {
      GPIOD->BSRR = (uint32_t)GPIO_PIN_11 << 16U; //-Set GPIO to 0
    }

    if(tick_SP == (expDuration_10us+2) )
    {
      CP_cnt=0;
    }

    tick_SP++;




    // read all ports
    if ( (CP_cnt > INATIVE_PIXELS) && (CP_cnt <= INATIVE_PIXELS + EFFECTIVE_PIXELS + 100) ) // 100 is an exageration value of
                                                                                            // oversampling never reach due to next loop control
    {
      // D0: pin PG12
      ADC_B0 = (GPIOG->IDR >> 12) & 0x01;
      // D1: pin PE9
      ADC_B1 = (GPIOE->IDR >> 9) & 0x01;
      // D2: pin PE11
      ADC_B2 = (GPIOE->IDR >> 11) & 0x01;
      // D3: pin PE14
      ADC_B3 = (GPIOE->IDR >> 14) & 0x01;
      // D4: pin PE13
      ADC_B4 = (GPIOE->IDR >> 13) & 0x01;
      // D5: pin PG14
      ADC_B5 = (GPIOG->IDR >> 14) & 0x01;
      // D6: pin PB6
      ADC_B6 = (GPIOB->IDR >> 6) & 0x01;
      // D7: pin PB7
      ADC_B7 = (GPIOB->IDR >> 7) & 0x01;

      if(alternateByte == 1)
      {
        reg_Low[i] = ADC_B0 | (ADC_B1<<1) | (ADC_B2<<2) | (ADC_B3<<3) | (ADC_B4<<4) | (ADC_B5<<5) | (ADC_B6<<6)| (ADC_B7<<7);
        i++;
        alternateByte = 0;
      }
      else
      {
        reg_High[i] = ADC_B0 | (ADC_B1<<1) | (ADC_B2<<2) | (ADC_B3<<3) | (ADC_B4<<4) | (ADC_B5<<5) | (ADC_B6<<6)| (ADC_B7<<7);
        alternateByte = 1;
      }
    }

    if (i == TOTAL_PIXELS)//CP_cnt == (INATIVE_PIXELS + EFFECTIVE_PIXELS+1+1) ) // one oversample cycle added
    {
      HAL_TIM_Base_Stop_IT(&htim2);

      GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16U; // CP_Clk set to 0
      send_data_main = 1;
      enableSampleTimerFlag = 0;
      CP_cnt = 0;
      tick_SP = 0;
      i = 0;
    }
  }




  //      // Toggle ADC_CDS_Clk2 (CDSCLK2 and CP_Clk - pin PD12)
  //      odr = GPIOD->ODR;
  //      GPIOD->BSRR = ((odr & GPIO_PIN_12) << 16U) | (~odr & GPIO_PIN_12);


  //---Toggle CP_Clk: To generate the readout frequency
  /*if(Tim2_tick == 0 || Tim2_tick == 3)
    {
        odr = GPIOD->ODR;
        GPIOD->BSRR = ((odr & GPIO_PIN_12) << 16U) | (~odr & GPIO_PIN_12);
    }
   */
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
