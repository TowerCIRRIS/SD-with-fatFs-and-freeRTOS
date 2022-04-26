/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
//DMAFIX todo1: Create config functions for rx and tx dma
static HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd);
static HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd);
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
	FRESULT resu; /* FatFs function common result code */
	uint32_t byteswritten;
	uint8_t wtext[] = "Ceci est un exemple d'écriture sur une carte SD!"; /* File write buffer */
	uint8_t rtext[_MAX_SS];/* File read buffer */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SAI1_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM15_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_DAC1_Init();
  MX_CAN1_Init();
  MX_FATFS_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

    resu = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
    if( resu != FR_OK)
    	{
    		//Error_Handler();
  	  	if(resu == FR_NO_FILESYSTEM)
  	  	{
  	  		resu = f_mkfs((TCHAR const*)SDPath, FM_FAT32, 0, rtext, sizeof(rtext));
  			if( resu != FR_OK)
  			{
  				Error_Handler();
  			}
  	  	}
  	  	else
  	  	{
  	  		Error_Handler();
  	  	}

    	}

  		resu = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
  		if(resu != FR_OK)
  		{
  			Error_Handler();
  		}
  		else
  		{
  			//Open file for writing (Create)
  			resu = f_open(&SDFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE);
  			if( resu != FR_OK)
  			{
  				Error_Handler();
  			}
  			else
  			{
  				//HAL_GPIO_WritePin(OUT_D0_GPIO_Port, OUT_D0_Pin, 1); // Utiliser pour mesure le temps d'execution avec un analyseur logique
  				//Write to the text file
  				resu = f_write(&SDFile, wtext, strlen((char *)wtext), (void *)&byteswritten);

  				//HAL_GPIO_WritePin(OUT_D0_GPIO_Port, OUT_D0_Pin, 0); // Utiliser pour mesure le temps d'execution avec un analyseur logique

  				if((byteswritten == 0) || (resu != FR_OK))
  				{
  					Error_Handler();
  				}
  				else
  				{
  					f_close(&SDFile);
  				}
  			}

  			f_mount(&SDFatFS, (TCHAR const*)NULL, 0); // unmount
  		}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief Configure the DMA to receive data from the SD card
  * @retval
  *  HAL_ERROR or HAL_OK
  */
static HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd)
{
  static DMA_HandleTypeDef hdma_rx;
  HAL_StatusTypeDef status = HAL_ERROR;

  /* Configure DMA Rx parameters */
  hdma_rx.Init.Request             = DMA_REQUEST_7;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

  hdma_rx.Instance = DMA2_Channel4;

  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmarx, hdma_rx);

  /* Stop any ongoing transfer and reset the state*/
  HAL_DMA_Abort(&hdma_rx);

  /* Deinitialize the Channel for new transfer */
  HAL_DMA_DeInit(&hdma_rx);

  /* Configure the DMA Channel */
  status = HAL_DMA_Init(&hdma_rx);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

  return (status);
}

//DMAFIX todo1: Create config functions for rx and tx dma
/**
  * @brief Configure the DMA to transmit data to the SD card
  * @retval
  *  HAL_ERROR or HAL_OK
  */
static HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd)
{
  static DMA_HandleTypeDef hdma_tx;
  HAL_StatusTypeDef status;

  /* Configure DMA Tx parameters */
  hdma_tx.Init.Request             = DMA_REQUEST_7;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  hdma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

  hdma_tx.Instance = DMA2_Channel4;

  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmatx, hdma_tx);

  /* Stop any ongoing transfer and reset the state*/
  HAL_DMA_Abort(&hdma_tx);

  /* Deinitialize the Channel for new transfer */
  HAL_DMA_DeInit(&hdma_tx);

  /* Configure the DMA Channel */
  status = HAL_DMA_Init(&hdma_tx);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

  return (status);
}

//DMAFIX todo 2: Override DMA write functions
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;

  // Invalidate the dma rx handle
  hsd1.hdmarx = NULL;

  // Prepare the dma channel for a read operation
  sd_state = SD_DMAConfigTx(&hsd1);

  if(sd_state == HAL_OK)
  {
    /* Write block(s) in DMA transfer mode */
    sd_state = HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks);
  }

  if( sd_state == HAL_OK)
  {
    return MSD_OK;
  }
  else
  {
    return MSD_ERROR;
  }

  return sd_state;
}

//DMAFIX todo 3: Override DMA read functions
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;
  /* Invalidate the dma tx handle*/
  hsd1.hdmatx = NULL;

  /* Prepare the dma channel for a read operation */
  sd_state = SD_DMAConfigRx(&hsd1);

  if(sd_state == HAL_OK)
  {
	   /* Read block(s) in DMA transfer mode */
	    sd_state = HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks);
  }

  if( sd_state == HAL_OK)
  {
	return MSD_OK;
  }
  else
  {
	return MSD_ERROR;
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

