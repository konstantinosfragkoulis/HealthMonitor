/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <math.h>

#include "tusb.h"
#include "usb_cdc.h"

#include "st1vafe6ax.h"
#include "data_types.h"
#include "ecg_dsp.h"
#include "imu_dsp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* --- Telemetry protocol --- */
#define TELEMETRY_HEADER    0xAABBU
#define EVENT_HEADER        0xCCDDU

/* --- FIFO sizes (must be powers of 2) --- */
#define SENSOR_FIFO_SIZE    16U
#define TELEMETRY_FIFO_SIZE 256U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef handle_GPDMA1_Channel4;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel3;
DMA_HandleTypeDef handle_GPDMA1_Channel2;

TIM_HandleTypeDef htim2;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* USER CODE BEGIN PV */

/*
 * SPI TX buffer for reading 6 bytes of accelerometer output.
 *
 * The first byte is the register address with the read bit set.
 * Bytes 1-6 are dummies clocked out while the IMU shifts data back.
 * Registers 0x28..0x2D map to: Z_L, Z_H, Y_L, Y_H, X_L, X_H.
 */
static uint8_t imu_tx[7] =
  { ST1VAFE6AX_REG_OUTZ_L_A | ST1VAFE6AX_READ_BIT, 0, 0, 0, 0, 0, 0 };

volatile uint8_t system_ready = 0;

DataSample_t sensor_fifo[SENSOR_FIFO_SIZE];
volatile uint8_t sensor_fifo_head = 0;
volatile uint8_t sensor_fifo_tail = 0;

TelemetryPacket_t telemetry_fifo[TELEMETRY_FIFO_SIZE];
uint8_t telemetry_fifo_head = 0;
uint8_t telemetry_fifo_tail = 0;

volatile uint8_t spi_finished = 0;
volatile uint8_t adc_finished = 0;
volatile uint8_t dma_in_progress = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_FLASH_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SPI3_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef IMU_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] =
    { reg, value };

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, tx, 2,
  ST1VAFE6AX_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  return status;
}

static HAL_StatusTypeDef IMU_ReadReg(uint8_t reg, uint8_t *value)
{
  uint8_t tx[2] =
    { reg | ST1VAFE6AX_READ_BIT, 0x00 };
  uint8_t rx[2] =
    { 0 };

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2,
  ST1VAFE6AX_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  *value = rx[1];
  return status;
}

void Check_DMA_Complete(void)
{
  if (spi_finished && adc_finished)
  {
    spi_finished = 0;
    adc_finished = 0;

    __DMB();
    sensor_fifo[sensor_fifo_head].status = 2;
    sensor_fifo_head = (sensor_fifo_head + 1) & (SENSOR_FIFO_SIZE - 1);
    dma_in_progress = 0;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  static uint32_t led_timer = 0;
  static uint8_t led_on = 0;
  static uint8_t peak_flag = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_FLASH_Init();
  MX_ICACHE_Init();
  MX_SPI3_Init();
  MX_USB_OTG_HS_PCD_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* ADC calibration */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_Delay(10);

  /* Enable the AD8232 */
  HAL_GPIO_WritePin(GPIOA, ECG_SDN_Pin, GPIO_PIN_SET);

  /* Initialize the IMU */
  HAL_Delay(100);

  uint8_t who_am_i = 0;
  IMU_ReadReg(ST1VAFE6AX_REG_WHO_AM_I, &who_am_i);
  HAL_Delay(10);
  if (who_am_i != ST1VAFE6AX_WHO_AM_I_VALUE) {
    while(1) {
      HAL_GPIO_TogglePin(GPIOB, LED_1_Pin);
      HAL_Delay(200);
    }
  }


  /*
   * CTRL4: Enable pulsed data-ready mode (75 us pulses).
   * The default latched mode caused issues because the DRDY line stays high
   * until the upper output register byte is read, which can miss edges if
   * the main loop doesn't read fast enough.
   */
  IMU_WriteReg(ST1VAFE6AX_REG_CTRL4, ST1VAFE6AX_CTRL4_DRDY_PULSED);
  HAL_Delay(10);

  /* INT1_CTRL: Route accelerometer data-ready to the INT1 pin. */
  IMU_WriteReg(ST1VAFE6AX_REG_INT1_CTRL, ST1VAFE6AX_INT1_DRDY_XL);
  HAL_Delay(10);

  /*
   * CTRL1: High-performance mode (default OP_MODE_XL = 000) with
   * accelerometer ODR = 480 Hz.
   */
  IMU_WriteReg(ST1VAFE6AX_REG_CTRL1, ST1VAFE6AX_ODR_XL_480HZ);
  HAL_Delay(10);

  HAL_TIM_Base_Start(&htim2);

  tusb_init();

  /* Now we can start reading from the IMU and ADC */
  system_ready = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* We must call this regularly to process USB events */
    tud_task();

    if (sensor_fifo_tail != sensor_fifo_head
        && sensor_fifo[sensor_fifo_tail].status == 2)
    {
      __DMB();

      RawECG_t ecg_sample;
      ecg_sample.timestamp = sensor_fifo[sensor_fifo_tail].timestamp;
      ecg_sample.ecg = (int16_t) sensor_fifo[sensor_fifo_tail].adc[1]
          - (int16_t) sensor_fifo[sensor_fifo_tail].adc[2];
      ecg_sample.reserved = 0;

      RawIMU_t imu_sample;
      imu_sample.timestamp = sensor_fifo[sensor_fifo_tail].timestamp;
      imu_sample.x = (int16_t) ((sensor_fifo[sensor_fifo_tail].imu[6] << 8)
          | sensor_fifo[sensor_fifo_tail].imu[5]);
      imu_sample.y = (int16_t) ((sensor_fifo[sensor_fifo_tail].imu[4] << 8)
          | sensor_fifo[sensor_fifo_tail].imu[3]);
      imu_sample.z = (int16_t) ((sensor_fifo[sensor_fifo_tail].imu[2] << 8)
          | sensor_fifo[sensor_fifo_tail].imu[1]);
      imu_sample.reserved = 0;

      /* Release the FIFO slot */
      sensor_fifo[sensor_fifo_tail].status = 0;
      sensor_fifo_tail = (sensor_fifo_tail + 1) & (SENSOR_FIFO_SIZE - 1);

      /* Run DSP pipelines */
      HeartBeatEvent_t latest_beat =
        { 0 };
      uint8_t leads_off = HAL_GPIO_ReadPin(ECG_LOP_GPIO_Port, ECG_LOP_Pin)
          || HAL_GPIO_ReadPin(ECG_LON_GPIO_Port, ECG_LON_Pin);

      if (!leads_off && ECG_Process_Sample(ecg_sample, &latest_beat) == 1)
      {
        HAL_GPIO_WritePin(GPIOB, LED_3_Pin, GPIO_PIN_SET);
        led_timer = HAL_GetTick();
        led_on = 1;
        peak_flag = 1;

        /*
         * Send the biometrics packet immediately.
         * The fields bpm, timestamp, and rmssd are populated by the `ECG_Process_Sample()` function
         */
        if (tud_cdc_connected()
            && tud_cdc_write_available() >= (sizeof(HeartBeatEvent_t)))
        {
          latest_beat.header = EVENT_HEADER;
          latest_beat.reserved = 0;
          latest_beat.crc = 0;
          latest_beat.crc = (uint8_t) HAL_CRC_Calculate(
              &hcrc, (uint32_t*) &latest_beat, sizeof(HeartBeatEvent_t) - 1);

          tud_cdc_write((uint8_t*) &latest_beat, sizeof(HeartBeatEvent_t));
          tud_cdc_write_flush();
        }

      }

      IMU_Process_Sample(imu_sample);

      /* Queue a telemetry frame */
      uint8_t next_telemetry_head = (telemetry_fifo_head + 1)
          & (TELEMETRY_FIFO_SIZE - 1);

      if (next_telemetry_head != telemetry_fifo_tail)
      {
        TelemetryPacket_t *pkt = &telemetry_fifo[telemetry_fifo_head];
        pkt->header = TELEMETRY_HEADER;
        pkt->timestamp = ecg_sample.timestamp;
        pkt->ecg = ecg_sample.ecg;
        pkt->x = imu_sample.x;
        pkt->y = imu_sample.y;
        pkt->z = imu_sample.z;
        pkt->r_peak = peak_flag;
        pkt->crc = 0;
        pkt->crc = (uint8_t) HAL_CRC_Calculate(&hcrc, (uint32_t*) pkt,
                                               sizeof(TelemetryPacket_t) - 1);

        telemetry_fifo_head = next_telemetry_head;
        peak_flag = 0;
      }
      /* If the FIFO is full the telemetry packet is dropped. */
    }

    if (!tud_cdc_connected())
    {
      telemetry_fifo_tail = telemetry_fifo_head;
      peak_flag = 0;
    }
    else
    {
      uint8_t packets_sent = 0;

      while (telemetry_fifo_tail != telemetry_fifo_head)
      {
        if (tud_cdc_write_available() < sizeof(TelemetryPacket_t))
          break;

        uint32_t written = tud_cdc_write(
            (uint8_t*) &telemetry_fifo[telemetry_fifo_tail],
            sizeof(TelemetryPacket_t));

        if (written != sizeof(TelemetryPacket_t))
          break;

        telemetry_fifo_tail = (telemetry_fifo_tail + 1)
            & (TELEMETRY_FIFO_SIZE - 1);
        packets_sent++;

      }

      if (packets_sent > 0)
      {
        tud_cdc_write_flush();
      }
    }

    if (led_on && (50 <= HAL_GetTick() - led_timer))
    {
      HAL_GPIO_WritePin(GPIOB, LED_3_Pin, GPIO_PIN_RESET);
      led_on = 0;
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 20;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_814CYCLES;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_20CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief FLASH Initialization Function
  * @param None
  * @retval None
  */
static void MX_FLASH_Init(void)
{

  /* USER CODE BEGIN FLASH_Init 0 */

  /* USER CODE END FLASH_Init 0 */

  /* USER CODE BEGIN FLASH_Init 1 */

  /* USER CODE END FLASH_Init 1 */
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FLASH_Lock() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FLASH_Init 2 */

  /* USER CODE END FLASH_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel4_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel4_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x7;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi3.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi3.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP2_LPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi3, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Prescaler = 15999;
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
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
  hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IMU_CS_Pin|ECG_SDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_1_Pin|LED_2_Pin|LED_3_Pin|LED_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IMU_INT1_Pin IMU_INT2_Pin */
  GPIO_InitStruct.Pin = IMU_INT1_Pin|IMU_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_CS_Pin ECG_SDN_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin|ECG_SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ECG_LOP_Pin ECG_LON_Pin */
  GPIO_InitStruct.Pin = ECG_LOP_Pin|ECG_LON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Flash_CS_Pin */
  GPIO_InitStruct.Pin = Flash_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Flash_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin LED_3_Pin LED_4_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|LED_3_Pin|LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != IMU_INT1_Pin || !system_ready)
    return;
  if (dma_in_progress)
    return;

  uint8_t head = sensor_fifo_head;
  uint8_t next_head = (head + 1) & (SENSOR_FIFO_SIZE - 1);
  if (next_head == sensor_fifo_tail)
    return; /* FIFO full - drop this sample */

  dma_in_progress = 1;
  spi_finished = 0;
  adc_finished = 0;

  /* We want greater resolution than HAL_GetTick() (1ms) */
  sensor_fifo[head].timestamp = htim2.Instance->CNT;
  sensor_fifo[head].status = 1;

  __DSB();

  HAL_StatusTypeDef spi_status, adc_status;
  /* MU: read 6 bytes starting from OUTZ_L_A with DMA */
  HAL_GPIO_WritePin(GPIOA, IMU_CS_Pin, GPIO_PIN_RESET);
  spi_status = HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*) imu_tx,
                                           (uint8_t*) sensor_fifo[head].imu, 7);

  if (spi_status != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, IMU_CS_Pin, GPIO_PIN_SET);
    memset(sensor_fifo[head].imu, 0, sizeof(sensor_fifo[head].imu));
    spi_finished = 1;
  }

  /*
   * ADC: read 3 channels (Vin divider, AD8232 output, AD8232 REFOUT).
   *
   * HAL_ADC_Start_DMA() expects a uint32_t* pointer, but the destination
   * buffer is uint16_t[3]. This is correct because GPDMA1 Channel 0 is
   * configured for halfword (16-bit) source and destination data widths.
   */
  adc_status = HAL_ADC_Start_DMA(&hadc1, (uint32_t*) sensor_fifo[head].adc, 3);

  if (adc_status != HAL_OK)
  {
    memset(sensor_fifo[head].adc, 0, sizeof(sensor_fifo[head].adc));
    adc_finished = 1;
  }

  /* If both failed, discard the slot */
  if (spi_status != HAL_OK && adc_status != HAL_OK)
  {
    sensor_fifo[head].status = 0;
    dma_in_progress = 0;
  }

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(GPIOA, IMU_CS_Pin, GPIO_PIN_SET);
    spi_finished = 1;
    Check_DMA_Complete();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_finished = 1;
    Check_DMA_Complete();
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(GPIOA, IMU_CS_Pin, GPIO_PIN_SET);
    memset(sensor_fifo[sensor_fifo_head].imu, 0,
           sizeof(sensor_fifo[sensor_fifo_head].imu));
    spi_finished = 1;
    Check_DMA_Complete();
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    memset(sensor_fifo[sensor_fifo_head].adc, 0,
           sizeof(sensor_fifo[sensor_fifo_head].adc));
    adc_finished = 1;
    Check_DMA_Complete();
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
