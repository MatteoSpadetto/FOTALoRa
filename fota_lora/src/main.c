/*!
 * \file      main.c
 *
 * \brief     Ping-Pong implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "low_power_manager.h"
#include "vcom.h"
#include "parsed_hex.h"

#if defined(REGION_AS923)

#define RF_FREQUENCY 923000000 // Hz

#elif defined(REGION_AU915)

#define RF_FREQUENCY 915000000 // Hz

#elif defined(REGION_CN470)

#define RF_FREQUENCY 470000000 // Hz

#elif defined(REGION_CN779)

#define RF_FREQUENCY 779000000 // Hz

#elif defined(REGION_EU433)

#define RF_FREQUENCY 433000000 // Hz

#elif defined(REGION_EU868)

#define RF_FREQUENCY 868000000 // Hz

#elif defined(REGION_KR920)

#define RF_FREQUENCY 920000000 // Hz

#elif defined(REGION_IN865)

#define RF_FREQUENCY 865000000 // Hz

#elif defined(REGION_US915)

#define RF_FREQUENCY 915000000 // Hz

#elif defined(REGION_RU864)

#define RF_FREQUENCY 864000000 // Hz

#else
#error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER 14 // dBm

#if defined(USE_MODEM_LORA)

#define LORA_BANDWIDTH 2 // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1        // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8 // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 5  // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#elif defined(USE_MODEM_FSK)

#define FSK_FDEV 25000          // Hz
#define FSK_DATARATE 50000      // bps
#define FSK_BANDWIDTH 50000     // Hz
#define FSK_AFC_BANDWIDTH 83333 // Hz
#define FSK_PREAMBLE_LENGTH 5   // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON false

#else
#error "Please define a modem in the compiler options."
#endif

#define APPLICATION_ADDRESS 0x08020000 // ADDRESS choose for test. It is possible to change with another compatible one
#define KEY 0xFF                       // Key value for sending and receiving check integrity
typedef enum
{
  LOWPOWER,
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 11 // Define the payload size here
#define LED_PERIOD_MS 200

#define LEDS_OFF         \
  do                     \
  {                      \
    LED_Off(LED_BLUE);   \
    LED_Off(LED_RED);    \
    LED_Off(LED_GREEN1); \
    LED_Off(LED_GREEN2); \
  } while (0);

const uint8_t PingMsg[] = "PINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPINGPING";
const uint8_t PongMsg[] = "PONG";
uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE]; // Buffer to send and received

States_t State = LOWPOWER;

int8_t RssiValue = 0; // Debugging RSSI
int8_t SnrValue = 0;  // Debugging SNR

/* Led Timers objects*/
static TimerEvent_t timerLed;

UART_HandleTypeDef huart2;
static void MX_USART2_UART_Init(void);

/* Private function prototypes -----------------------------------------------*/
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

/*!
 * \brief Function executed on when led timer elapses
 */
static void OnledEvent(void *context);
static void FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data); // Programming recevier memory

/**
 * Function to perform jump to system memory boot from user application
 *
 * Call function when you want to jump to system memory
 */
void JumpToBootloader(void)
{
  void (*SysMemBootJump)(void);

  /**
	 * Step: Set system memory address.
	 *
	 *       For STM32F429, system memory is on 0x1FFF 0000
	 *       For other families, check AN2606 document table 110 with descriptions of memory addresses
	 */
  volatile uint32_t addr = APPLICATION_ADDRESS;

  /**
	 * Step: Disable RCC, set it to default (after reset) settings
	 *       Internal clock, no PLL, etc.
	 */
#if defined(USE_HAL_DRIVER)
  HAL_RCC_DeInit();
#endif /* defined(USE_HAL_DRIVER) */
#if defined(USE_STDPERIPH_DRIVER)
  RCC_DeInit();
#endif /* defined(USE_STDPERIPH_DRIVER) */

  /**
	 * Step: Disable systick timer and reset it to default values
	 */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  SCB->VTOR = 0;

  /**
	 * Step: Disable all interrupts
	 */
  __disable_irq();

  /**
	 * Step: Remap system memory to address 0x0000 0000 in address space
	 *       For each family registers may be different.
	 *       Check reference manual for each family.
	 *
	 *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
	 *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
	 *       For others, check family reference manual
	 */
  //Remap by hand... {
#if defined(STM32F4)
  SYSCFG->MEMRMP = 0x01;
#endif
#if defined(STM32F0)
  SYSCFG->CFGR1 = 0x01;
#endif
  //} ...or if you use HAL drivers
  //__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();	//Call HAL macro to do this for you

  /**
	 * Step: Set jump memory location for system memory
	 *       Use address with 4 bytes offset which specifies jump location where program starts
	 */
  SysMemBootJump = (void (*)(void))(*((uint32_t *)(addr + 4)));

  /**
	 * Step: Set main stack pointer.
	 *       This step must be done last otherwise local variables in this function
	 *       don't have proper value since stack pointer is located on different position
	 *
	 *       Set direct address location which specifies stack pointer in SRAM location
	 */
  __set_MSP(*(uint32_t *)addr);

  /**
	 * Step: Actually call our function to jump to set location
	 *       This will start system memory execution
	 */
  SysMemBootJump();

  /**
	 * Step: Connect USB<->UART converter to dedicated USART pins and test
	 *       and test with bootloader works with STM32 Flash Loader Demonstrator software
	 */
}

typedef void (*pFunction)(void);
pFunction JumpToApplication;
uint32_t JumpAddress;

void test_jump(void) // Another method to jump to the new ADDRESS
{
  /* Test if user code is programmed starting from address 'APPLICATION_ADDRESS' */

  if (((*(__IO uint32_t *)APPLICATION_ADDRESS) & 0x2FFE0000) == 0x20000000)
  {

    /* Jump to user application */
    JumpAddress = *(__IO uint32_t *)(APPLICATION_ADDRESS + 4);

    JumpToApplication = (pFunction)JumpAddress;

    /* Initialize user application's Stack Pointer */

    __set_MSP(*(__IO uint32_t *)APPLICATION_ADDRESS);

    JumpToApplication();
  }
}

/**
 * Main application entry point.
 */
int main(void)
{

  bool isMaster = false;
  uint8_t i;

  MX_USART2_UART_Init();

  HAL_Init();

  SystemClock_Config();

  DBG_Init();

  HW_Init();

  /*Disbale Stand-by mode*/

  LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);

  /* Led Timers*/
  TimerInit(&timerLed, OnledEvent);
  TimerSetValue(&timerLed, LED_PERIOD_MS);

  TimerStart(&timerLed);

  // Radio initialization
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  Radio.Init(&RadioEvents);

  HAL_UART_Transmit(&huart2, (uint8_t *)"InitDone\r\n", 10, 200);
  Radio.SetChannel(RF_FREQUENCY);
#if defined(USE_MODEM_LORA)

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

#elif defined(USE_MODEM_FSK)

  Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                    FSK_DATARATE, 0,
                    FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, 0, 3000);

  Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                    0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                    0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                    0, 0, false, true);

#else
#error "Please define a frequency band in the compiler options."
#endif

  //JumpToBootloader();
  Radio.Rx(RX_TIMEOUT_VALUE);
  char txt[100];                // Debug buffer
  uint32_t counter_byte_tx = 0; // Debug MSG SENT
  uint32_t counter_byte_rx = 0; // Debug MSG RECEIVED
  uint16_t dd_index = 0;        // DATA index
  uint16_t aaaa_index = 0;      // ADDRESS index
  uint8_t update_flag = 0;      // 0 = NO UPDATE | 1 = UPDATE RUNNING
  uint8_t retry_flag = 0;

  while (1)
  {
    State = RX;
    isMaster = false; // Must be set if it is Master [SENDER] or Slave [RECEIVER]
    sprintf(txt, "IsMaster?: %d\r\n", isMaster);
    HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 200);

    switch (State)
    {
    case RX:
      if (isMaster == true) // Master CODE
      {
        if (BufferSize > 0)
        {
          // Give update configuration to the slave
          Buffer[0] = 0xAA;      // Update request
          Radio.Send(Buffer, 1); // Send update request

          TimerStop(&timerLed);
          LED_Off(LED_BLUE);
          LED_Off(LED_GREEN);
          LED_Off(LED_RED1);
          LED_Toggle(LED_RED2);

          Buffer[0] = KEY;                       // KEY value
          Buffer[1] = arr_aaaa[aaaa_index] >> 8; // ADDRESS MSByte
          Buffer[2] = arr_aaaa[aaaa_index];      // ADDRESS LSByte
          aaaa_index++;                          // Preparing for next address
          sprintf(txt, "DATA_AAAA: %02x%02x\r\n", Buffer[0], Buffer[1]);
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);

          uint8_t index = 3;
          uint16_t tmp_index = dd_index;
          // Preparing the buffer DATA to send with two DATA for each LoRa message
          for (size_t k = tmp_index; k < tmp_index + 2; k++)
          {
            uint16_t to_parse = arr_dd[k]; // Using the data to store
            for (size_t i = 0; i < 4; i++)
            {
              Buffer[index] = (to_parse >> ((3 - i) * 8)) & 0xFF; // Storing in buffer to send all bytes
              index++;
            }
            dd_index++;
          }
          DelayMs(25);
          // Debug BUFFER to send
          for (size_t i = 0; i < sizeof(Buffer); i++)
          {
            sprintf(txt, "DATA[%d]: %02x\r\n", i, Buffer[i]);
            HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
          }
          // Debug BUFFER SIZE
          sprintf(txt, "SIZE[%d]\r\n", sizeof(Buffer));
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
          // Debug the END OF BUFFER PREPARATION
          HAL_UART_Transmit(&huart2, (uint8_t *)"DONE\r\n", strlen("DONE\r\n"), 1000);
          // Sending via LoRa
          Radio.Send(Buffer, BufferSize);
          // Incrementing the number of messages sent
          counter_byte_tx++;
          // Debug NUMBER OF BYTES SENT and BUFFER
          sprintf(txt, "ByteSENT: %d -- %02x\r\n", counter_byte_tx * BUFFER_SIZE, Buffer);
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 200);
          // If is the last message to send then STOP
          if (counter_byte_tx * BUFFER_SIZE > (sizeof(arr_dd) / sizeof(arr_dd[0])))
          {
            Buffer[0] = 0xFA;      // End of update
            Radio.Send(Buffer, 1); // Send end of update
            return 0;
          }
        }
      }
      else // Receiver CODE
      {
        if (BufferSize > 0)
        {
          if (Buffer[0] == 0xAA) // Init of update
          {
            update_flag++;
            // ERASE operation. ADDRESSES must be set accordingly to the memory to write
            if (isMaster == false & update_flag == 1)
            {
              for (size_t i = 0; i < 256; i++)
              {
                FLASH_PageErase(i, FLASH_BANK_1); // Erasing all the needed space
                uint8_t status;
                status = FLASH_WaitForLastOperation(1000); // Wait until the operation ends
              }
            }
          }
          else if (Buffer[0] = 0xFA) // End of update
          {
            test_jump(); // Jump to the application address
          }
          else if (Buffer[0] == KEY) // Check for intgrity
          {
            TimerStop(&timerLed);
            LED_Off(LED_RED1);
            LED_Off(LED_RED2);
            LED_Off(LED_GREEN);
            LED_Toggle(LED_BLUE);

            counter_byte_rx++; // Incrementing the number of received messages

            ////WRITING on address
            uint8_t buff_to_store[sizeof(Buffer)];
            for (int i = 0; i < sizeof(Buffer); i++)
            {
              buff_to_store[i] = Buffer[i];
            }
            // Debug received message
            for (size_t i = 0; i < sizeof(buff_to_store); i++)
            {
              sprintf(txt, "DATA[%d]: %02x\r\n", i, buff_to_store[i]);
              HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 1000);
            }
            HAL_StatusTypeDef status; // Debug status

            // UNLOACK flash
            status = HAL_FLASH_Unlock();

            uint32_t aaaa = APPLICATION_ADDRESS + (Buffer[1] << 8) + Buffer[2]; // Setting ADDRESS
            uint64_t dd = 0x00;                                                 // Cleaning data bytes
            // Storing DATA to write in DD
            for (size_t i = 10; i >= 3; i--)
            {
              dd = (dd << 8) + Buffer[i];
            }
            FLASH_Program_DoubleWord(aaaa, dd); // Writing BYTES on memory
            // LOCK flash
            status = HAL_FLASH_Lock();

            // Debug receive MSG and its parameters
            sprintf(txt, "MsgREC: %d -- ByteREC: %d -- ByteperMsg: %d\r\nSF: %d -- dBm: %d -- BW: %d -- Freq: %d Hz\r\n",
                    counter_byte_rx, counter_byte_rx * BUFFER_SIZE,
                    BUFFER_SIZE, LORA_SPREADING_FACTOR, TX_OUTPUT_POWER, LORA_BANDWIDTH, RF_FREQUENCY);
            HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 200);
            if (counter_byte_rx >= sizeof(arr_dd) / sizeof(arr_dd[0]))
            {
            }
          }
          else // valid reception but not a PING as expected
          {
            isMaster = true;
            Radio.Rx(RX_TIMEOUT_VALUE);
          }
        }
        else
        {
          retry_flag++;
          if (retry_flag == 20) // If after waiting 20 message of update there is not an update request then jump
          {
            test_jump();
          }
        }
      }
      State = LOWPOWER;
      break;
    case TX: // NOT USED
      // HAL_UART_Transmit(&huart2, (uint8_t *)"TX\r\n", 4, 200);

      // Indicates on a LED that we have sent a PING [Master]
      // Indicates on a LED that we have sent a PONG [Slave]
      //GpioWrite( &Led2, GpioRead( &Led2 ) ^ 1 );
      Radio.Rx(RX_TIMEOUT_VALUE);
      State = LOWPOWER;
      break;
    case RX_TIMEOUT:
    case RX_ERROR:
      if (isMaster == true)
      {
        //HAL_UART_Transmit(&huart2, (uint8_t *)"MASTER\r\n", 8, 200);
        // Send the next PING frame
        Buffer[0] = 'P';
        Buffer[1] = 'I';
        Buffer[2] = 'N';
        Buffer[3] = 'G';
        for (i = 4; i < BufferSize; i++)
        {
          Buffer[i] = i - 4;
        }
        DelayMs(1);
        Radio.Send(Buffer, BufferSize);
        //sprintf(txt, "BufferERR: %c%c%c%c\r\n", Buffer[0], Buffer[1], Buffer[2], Buffer[3]);
        //        HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 200);
      }
      else
      {
        //  HAL_UART_Transmit(&huart2, (uint8_t *)"SLAVEerr\r\n", 10, 200);
        Radio.Rx(RX_TIMEOUT_VALUE);
      }
      State = LOWPOWER;
      break;
    case TX_TIMEOUT:
      Radio.Rx(RX_TIMEOUT_VALUE);
      State = LOWPOWER;
      break;
    case LOWPOWER:
      break;
    default:
      // Set low power
      break;
    }

    DISABLE_IRQ();
    /* if an interupt has occured after __disable_irq, it is kept pending
     * and cortex will not enter low power anyway  */
    if (State == LOWPOWER)
    {
#ifndef LOW_POWER_DISABLE
      LPM_EnterLowPower();
#endif
    }
    ENABLE_IRQ();
  }
}

void OnTxDone(void)
{
  Radio.Sleep();
  State = TX;
  PRINTF("OnTxDone\n\r");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Radio.Sleep();
  BufferSize = size;
  memcpy(Buffer, payload, BufferSize);
  RssiValue = rssi;
  SnrValue = snr;
  State = RX;
  PRINTF("OnRxDone\n\r");
  PRINTF("RssiValue=%d dBm, SnrValue=%d\n\r", rssi, snr);
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  State = TX_TIMEOUT;

  PRINTF("OnTxTimeout\n\r");
}

void OnRxTimeout(void)
{
  Radio.Sleep();
  State = RX_TIMEOUT;
  PRINTF("OnRxTimeout\n\r");
}

void OnRxError(void)
{
  Radio.Sleep();
  State = RX_ERROR;
  PRINTF("OnRxError\n\r");
}

static void OnledEvent(void *context)
{
  LED_Toggle(LED_BLUE);
  LED_Toggle(LED_RED1);
  LED_Toggle(LED_RED2);
  LED_Toggle(LED_GREEN);

  TimerStart(&timerLed);
}

/**
  * @brief  Program double-word (64-bit) at a specified address.
  * @param  Address specifies the address to be programmed.
  * @param  Data specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data)
{
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address)); // Check the parameters
  SET_BIT(FLASH->CR, FLASH_CR_PG);                 // Set PG bit
  *(__IO uint32_t *)Address = (uint32_t)Data;      // Program first word
  /* Barrier to ensure programming is performed in 2 steps, in right order
    (independently of compiler optimization behavior) */
  __ISB();
  *(__IO uint32_t *)(Address + 4U) = (uint32_t)(Data >> 32); // Program second word
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
