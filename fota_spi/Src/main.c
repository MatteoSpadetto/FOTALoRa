/* USER CODE BEGIN Header */
/*

  /$$$$$$  /$$$$$$$  /$$$$$$       /$$   /$$ /$$$$$$$  /$$$$$$$   /$$$$$$  /$$$$$$$$ /$$$$$$$$
 /$$__  $$| $$__  $$|_  $$_/      | $$  | $$| $$__  $$| $$__  $$ /$$__  $$|__  $$__/| $$_____/
| $$  \__/| $$  \ $$  | $$        | $$  | $$| $$  \ $$| $$  \ $$| $$  \ $$   | $$   | $$      
|  $$$$$$ | $$$$$$$/  | $$        | $$  | $$| $$$$$$$/| $$  | $$| $$$$$$$$   | $$   | $$$$$   
 \____  $$| $$____/   | $$        | $$  | $$| $$____/ | $$  | $$| $$__  $$   | $$   | $$__/   
 /$$  \ $$| $$        | $$        | $$  | $$| $$      | $$  | $$| $$  | $$   | $$   | $$      
|  $$$$$$/| $$       /$$$$$$      |  $$$$$$/| $$      | $$$$$$$/| $$  | $$   | $$   | $$$$$$$$
 \______/ |__/      |______/       \______/ |__/      |_______/ |__/  |__/   |__/   |________/
                                                                                              
*/
/* Used for LoRa Firmware Update Over the Air project. Embedded system course University of Trento.
This application is able to update the firmware of another STM32 via SPI thanks to the following steps:
1) SYNC of the two boards
2) MASS ERASE of the receiver flash memory
3) WRITING firmware to the second STM32 memory*/
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "parsed_hex.h" // Auto-generated header file with addresses and data

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
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char txt[100];                                                   // Debug buffer
  size_t arr_aaaa_length = sizeof(arr_aaaa) / sizeof(arr_aaaa[0]); // Addresses array size
  uint8_t status_flag = 0;                                         // Flag to pass from one to another status
  uint8_t dd_index = 0;                                            // Data index
  uint8_t aaaa_index = 0;                                          // Address indexI
  uint8_t ack_frame[3];                                            // ACK frame  to send
  uint8_t ack_resp[3];                                             // ACK frame for response
  uint8_t flag_stop = 0;                                           // Flag stopping code at the end of the writing process

  while (1)
  {
    // SYNC ROUTINE
    if (status_flag == 0)
    {
      // Send SYNC request
      uint8_t sync_frame[1];
      uint8_t sync_resp[1];
      sync_frame[0] = 0x5A;                      // SYNC frame
      while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
        ;
      HAL_SPI_TransmitReceive(&hspi1, sync_frame, &sync_resp, 1, 1000); // Send + Rec SYNC
      // Debug SYNC resp
      sprintf(txt, "RESP_OF_SYNC: %02x\r\n", sync_resp[0]);
      HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);

      //Send ACK request
      ack_frame[0] = 0x00;                       // ACK frame [0]
      ack_frame[1] = 0xFF;                       // ACK frame [1]
      ack_frame[2] = 0x79;                       // ACK frame [2]
      while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
        ;
      HAL_SPI_TransmitReceive(&hspi1, ack_frame, &ack_resp, 3, 1000); // Send + Rec ACK
      if (sync_resp[0] == 0xA5 && ack_resp[1] == 0x79)                // If it is a positive response
      {
        status_flag = 1; // Pass to MASS ERASE ROUTINE
        // Debug ACK of SYNC
        sprintf(txt, "ACK_OF_SYNC: %02x || %02x || %02x\r\n", ack_resp[0], ack_resp[1], ack_resp[2]);
        HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
      }
    }
    // MASS ERASE ROUTINE
    else if (status_flag == 1)
    {
      uint8_t cmd_frame[3];
      uint8_t cmd_resp[3];
      cmd_frame[0] = 0x5A;                       // ERASE CMD frame [0]
      cmd_frame[1] = 0x44;                       // ERASE CMD frame [1]
      cmd_frame[2] = 0xBB;                       // ERASE CMD frame [2]
      while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
        ;
      HAL_SPI_TransmitReceive(&hspi1, cmd_frame, &cmd_resp, 3, 1000); // Send + Rec ERASE CMD
      // Debug ERASE CMD resp
      sprintf(txt, "RESP_OF_CMD_ER: %02x || %02x || %02x\r\n", cmd_resp[0], cmd_resp[1], cmd_resp[2]);
      HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);

      ack_frame[0] = 0x00;                       // ACK frame [0]
      ack_frame[1] = 0x00;                       // ACK frame [1]
      ack_frame[2] = 0x79;                       // ACK frame [2]
      while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
        ;
      HAL_SPI_TransmitReceive(&hspi1, ack_frame, &ack_resp, 3, 1000); // Send + Rec ACK
                                                                      // Debug ACK of ERASE CMD
      sprintf(txt, "ACK_OF_CMD_ER: %02x || %02x || %02x\r\n", ack_resp[0], ack_resp[1], ack_resp[2]);
      HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
      if (ack_resp[1] == 0x79) // If it is a positive response
      {
        uint8_t erase_frame[3];
        uint8_t erase_resp[3];
        erase_frame[0] = 0xFF;                     // ERASE OPT frame [0]
        erase_frame[1] = 0xFF;                     // ERASE OPT frame [1]
        erase_frame[2] = 0x00;                     // ERASE OPT frame [2]
        while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
          ;
        HAL_SPI_TransmitReceive(&hspi1, erase_frame, &erase_resp, 3, 1000); // Send + Rec ERASE OPT
        // Debug ERASE OPT resp
        sprintf(txt, "RESP_OF_ERASE: %02x || %02x || %02x\r\n", erase_resp[0], erase_resp[1], erase_resp[2]);
        HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);

        ack_frame[0] = 0x00;                       // ACK frame [0]
        ack_frame[1] = 0xFF;                       // ACK frame [1]
        ack_frame[2] = 0x79;                       // ACK frame [2]
        while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
          ;
        HAL_SPI_TransmitReceive(&hspi1, ack_frame, &ack_resp, 3, 1000); // Send + Rec ACK
        // Debug ACK of ERASE OPT
        sprintf(txt, "ACK_OF_ERASE: %02x || %02x || %02x\r\n", ack_resp[0], ack_resp[1], ack_resp[2]);
        HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);

        status_flag = 2; // Pass to WRITE ROUTINE
      }
    }
    // WRITE ROUTINE
    else if (status_flag >= 2 & status_flag <= 3)
    {
      uint8_t data_resp[18];
      data_resp[3] = 0x00;

      while (data_resp[3] != 0x79) // Retry to write the same bytes until the ACK is positive
      {
        uint8_t cmd_frame[3];
        uint8_t cmd_resp[3];
        cmd_frame[0] = 0x5A;                       // WRITE CMD frame [0]
        cmd_frame[1] = 0x31;                       // WRITE CMD frame [1]
        cmd_frame[2] = 0xCE;                       // WRITE CMD frame [2]
        while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
          ;
        HAL_SPI_TransmitReceive(&hspi1, cmd_frame, &cmd_resp, 3, 1000); // Send + Rec WRITE CMD
        // Debug WRITE CMD resp
        sprintf(txt, "RESP_OF_CMD_WR: %02x || %02x || %02x\r\n", cmd_resp[0], cmd_resp[1], cmd_resp[2]);
        HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);

        ack_frame[0] = 0x00;                       // ACK frame [0]
        ack_frame[1] = 0x00;                       // ACK frame [1]
        ack_frame[2] = 0x79;                       // ACK frame [2]
        while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
          ;
        HAL_SPI_TransmitReceive(&hspi1, ack_frame, &ack_resp, 3, 1000); // Send + Rec ACK
        // Debug ACK resp
        sprintf(txt, "ACK_OF_CMD_WR: %02x || %02x || %02x\r\n", ack_resp[0], ack_resp[1], ack_resp[2]);
        HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
        if (ack_resp[1] == 0x79 || ack_resp[1] == 0xA5) // If it is a positive response
        {

          uint8_t add_frame[5];
          uint8_t add_resp[5];
          uint8_t tmp_aaaa = 0;

          // Sending address 0x0800cccc + XOR of 0x0800cccc
          add_frame[0] = 0x08;                                              // ADDRESS frame [1]
          add_frame[1] = 0x00;                                              // ADDRESS frame [2]
          add_frame[2] = (arr_aaaa[aaaa_index] >> 8) & 0xFF;                // ADDRESS frame [3]
          add_frame[3] = arr_aaaa[aaaa_index] & 0xFF;                       // ADDRESS frame [4]
          add_frame[4] = 0x08 ^ add_frame[1] ^ add_frame[2] ^ add_frame[3]; // ADDRESS frame [5]
          aaaa_index = aaaa_index + 2;                                      // Prepare index for the next address
          while (hspi1.State != HAL_SPI_STATE_READY)                        // Wait until SPI is not busy
            ;
          HAL_SPI_TransmitReceive(&hspi1, add_frame, &add_resp, 5, 1000); // Send + Rec ADDRESS
          // Debug ADDRESS resp
          sprintf(txt, "RESP_OF_ADD: %02x || %02x || %02x || %02x || %02x\r\n", add_resp[0], add_resp[1], add_resp[2], add_resp[3], add_resp[4]);
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
          // Debug ADDRESS values
          sprintf(txt, "ADDRESS: %02x || %02x || %02x || %02x\r\n", add_frame[0], add_frame[1], add_frame[2], add_frame[3]);
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);

          ack_frame[0] = 0x00;                       // ACK frame [0]
          ack_frame[1] = 0x00;                       // ACK frame [1]
          ack_frame[2] = 0x79;                       // ACK frame [2]
          while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
            ;
          HAL_SPI_TransmitReceive(&hspi1, ack_frame, &ack_resp, 3, 1000); // Send + Rec ACK
          // Debug ACK resp
          sprintf(txt, "ACK_OF_ADD: %02x || %02x || %02x\r\n", ack_resp[0], ack_resp[1], ack_resp[2]);
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);

          // Sending SIZE of data buffer + DATA to be written + XOR of size and data
          uint8_t data_frame[17];
          uint8_t frame_resp[17];
          data_frame[0] = 0x0F; // DATA frame [0] --> SIZE
          uint8_t data_xor[1];
          uint8_t xor_resp[1];
          data_xor[0] = data_frame[0]; // Set XOR initial value == SIZE

          uint8_t index = 1;
          uint8_t tmp_index = dd_index;
          for (size_t k = tmp_index; k < tmp_index + 4; k++) // Cycling array data with steps of 4
          {
            uint32_t to_parse = arr_dd[k]; // Storing data to parse [4 bytes]
            for (size_t i = 0; i < 4; i++) // For each byte of the data
            {
              data_frame[index] = (to_parse >> ((3 - i) * 8)) & 0xFF; // Write data to the buffer to send
              data_xor[0] = data_xor[0] ^ data_frame[index];          // Updating XOR values
              // Debug DATA values
              sprintf(txt, "DATA_DATA: %02x\r\n", data_frame[index]);
              HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
              index++;
            }
            dd_index++;
          }
          // Debug XOR value
          sprintf(txt, "XOR_OF_ADD: %02x\r\n", data_xor[0]);
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
          while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
            ;
          HAL_SPI_TransmitReceive(&hspi1, data_frame, &frame_resp, 17, 1000); // Send + Rec SIZE + DATA
          while (hspi1.State != HAL_SPI_STATE_READY)                          // Wait until SPI is not busy
            ;
          HAL_SPI_TransmitReceive(&hspi1, data_xor, &xor_resp, 1, 1000); // Send + Rec XOR byte
          // Debug DATA resp
          sprintf(txt, "RESP_OF_DATA: %02x || %02x || %02x || %02x\r\n", frame_resp[0], frame_resp[1], frame_resp[2], frame_resp[3]);
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);

          if (frame_resp[3] != 0x79) // If it is not a positive response
          {
            dd_index = dd_index - 4;     // Reset the data index to the current value
            aaaa_index = aaaa_index - 2; // Reset the index to the current value
          }

          ack_frame[0] = 0x00;                       // ACK frame [0]
          ack_frame[1] = 0x00;                       // ACK frame [1]
          ack_frame[2] = 0x79;                       // ACK frame [2]
          while (hspi1.State != HAL_SPI_STATE_READY) // Wait until SPI is not busy
            ;
          HAL_SPI_TransmitReceive(&hspi1, ack_frame, &ack_resp, 3, 1000); // Send + Rec ACK
          // Debug ACK resp
          sprintf(txt, "ACK_OF_DATA: %02x || %02x || %02x\r\n", ack_resp[0], ack_resp[1], ack_resp[2]);
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
          // Debug DATA INDEX and STATUS FLAG
          sprintf(txt, "len: %d, %d\r\n", dd_index, status_flag);
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
          // Debug ADDRESS INDEX and ADDRESS VALUE at that position
          sprintf(txt, "index: %d, %d\r\n", aaaa_index, arr_aaaa[aaaa_index]);
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
          HAL_Delay(20); // Just for safety reason and make serial readable. Cuold be commented

          if (aaaa_index >= arr_aaaa_length) // If the WRITE PROCESS ends stop the code
          {
            status_flag = 5;
            return;
          }
        }
      }
      //status_flag = 5;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)"ERROR\r\n", strlen("ERROR\r\n"), 1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
