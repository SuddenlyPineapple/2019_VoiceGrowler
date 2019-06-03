/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_CS43L22.h"
#include <math.h>
#include <string.h>
#include "ff.h"
#include "stm32f4xx.h"
#include "fft.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE 1
#define SDSIZE 1
#define FFTBUFF 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
int transmit_adc = 1;
int transmit_i2s = 1;
int buffer = 1;
uint32_t data1[SAMPLE];
uint32_t EchoData[15000];
uint32_t ChildvoiceData[3000] = {0};
double ChildvoiceDataFFTIn[3000] = {0};
int ChildvoiceTmp = 0;
uint16_t Speedy[1000] = {0};
uint16_t data_adc[1];
int EchoCounter = 0;
int ChildVoiceCounter = 0;
int Effect_Echo = 0;
int Effect_Overdrive = 0;
int Effect_Childvoice = 0;
int Effect_CBA = 0;
double FFT_Val = 0;
double in[2000] = {0};
double volume;
double old_volume;
uint16_t dataSAMPLE = 0;
//End Defined for CS43L22 ----------------------------------------
//SD CARD VARS
uint8_t SDbuffer[SDSIZE];      	//bufor odczytu i zapisu
uint16_t tempbuff[SDSIZE];
static FATFS FatFs;    				//uchwyt do urz�dzenia FatFs (dysku, karty SD...)
FRESULT fresult;       				//do przechowywania wyniku operacji na bibliotece FatFs
FIL file;                  			//uchwyt do otwartego pliku
WORD bytes_read;           			//liczba odczytanych byte
volatile int startstop=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void resetAudio(){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);

    Effect_Echo = 0;
    Effect_Overdrive = 0;
    Effect_Childvoice = 0;
    Effect_CBA = 0;
    HAL_I2S_DeInit(&hi2s3);
    hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    HAL_I2S_Init(&hi2s3);
}

void EchoRelay(uint16_t *dataADC){
    if (EchoCounter < 5000) {
        EchoData[EchoCounter] = dataADC[0];
    } else if (EchoCounter < 10000) {
        dataADC[0] += EchoData[EchoCounter - 5000] / 2;
        EchoData[EchoCounter] = dataADC[0];
    } else if (EchoCounter <= 14999) {
        dataADC[0] += EchoData[EchoCounter - 5000] / 2;
        EchoData[EchoCounter-10000] = dataADC[0];
    }

    dataADC[0] = dataADC[0]/2;

    if (EchoCounter >= 14999){
        EchoCounter = 5000;
    } else {
        EchoCounter++;
    }
}

void Overdrive(uint16_t *dataADC){
	if( dataADC[0]>2250) dataADC[0]=2250;
	else if( dataADC[0]<500)  dataADC[0]=500;
}

void Childvoice(uint16_t *dataADC){
    ChildvoiceDataFFTIn[ChildVoiceCounter] = dataADC[0];

    Fft_transform(ChildvoiceDataFFTIn, 0, 3000);

    FFT_Val = ChildvoiceDataFFTIn[ChildVoiceCounter];
    for(int a = 2000; a < 2999 ; a++){
            ChildvoiceDataFFTIn[a] *= 0;
    }

    Fft_inverseTransform(0,ChildvoiceDataFFTIn, 3000);

    dataADC[0] = ChildvoiceDataFFTIn[ChildVoiceCounter];

    ChildVoiceCounter++;
    if(ChildVoiceCounter >= 3000) ChildVoiceCounter = 0;
}

void CriminalVoice(uint16_t *dataADC){
//    if(ChildvoiceTmp == 0) ChildvoiceData[0] = dataADC[0];
//
//    ChildvoiceTmp++;
//    if(ChildvoiceTmp >= 8) {
//        ChildvoiceTmp = 0;
//    }
//
//    dataADC[0] = ChildvoiceData[0];
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    transmit_i2s = 1; //useless
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

    HAL_ADC_Stop_DMA(&hadc1);
    data_adc[0] = data1[0];

    if(Effect_Echo == 1) EchoRelay(data_adc);

    if(Effect_Overdrive == 1) Overdrive(data_adc);
    //if(Effect_Childvoice == 1) Childvoice(data_adc);
    //if(Effect_CBA == 1) CriminalVoice(data_adc);


    dataSAMPLE = data_adc[0];
    //data_adc[0] *=3; // Regulacja głośności - powinnismy zrobic zakres -3 do +3/4, gdzie jak zglasniamy to jest np. *3, a jak zciszamy /3 i to wszystko na jakimś potencjometrze

    if(Effect_Childvoice == 1 || Effect_CBA == 1){
        HAL_I2S_Transmit_DMA(&hi2s3, Speedy, 1000);
        HAL_ADC_Start_DMA(&hadc1, Speedy, 1000);
    } else {
        HAL_I2S_Transmit_DMA(&hi2s3, data_adc, SAMPLE);
        HAL_ADC_Start_DMA(&hadc1, data1, SAMPLE);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
            volume = HAL_ADC_GetValue(&hadc2);
            volume = volume/4095;
            if(volume < 0.009) volume = 0;
            CS43_SetVolume(90*volume);
        }
        HAL_ADC_Start(&hadc2);
    }
    if (htim->Instance == TIM3)
    {
        if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3) == GPIO_PIN_RESET) {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
            if(Effect_Echo == 0) {
                resetAudio();
                Effect_Echo = 1;
            }
            else Effect_Echo = 0;
        }
        if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4) == GPIO_PIN_RESET) {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
            if(Effect_Overdrive == 0) {
                resetAudio();
                Effect_Overdrive = 1;
            }
            else Effect_Overdrive = 0;
        }
        if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5) == GPIO_PIN_RESET) {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
            if(Effect_Childvoice == 0) {
                resetAudio();
                Effect_Childvoice = 1;
            }
            else Effect_Childvoice = 0;

            HAL_ADC_Stop_DMA(&hadc1);
            HAL_I2S_DMAStop(&hi2s3);
            HAL_ADC_Start_DMA(&hadc1, Speedy, 1000);
        }
        if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6) == GPIO_PIN_RESET) {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

            if(Effect_CBA == 0) {
                resetAudio();
                Effect_CBA = 1;
                HAL_I2S_DeInit(&hi2s3);
                hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_22K;
                HAL_I2S_Init(&hi2s3);
            }
            else {
                Effect_CBA = 0;
                HAL_I2S_DeInit(&hi2s3);
                hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
                HAL_I2S_Init(&hi2s3);
            }
            HAL_ADC_Stop_DMA(&hadc1);
            HAL_I2S_DMAStop(&hi2s3);
            HAL_ADC_Start_DMA(&hadc1, Speedy, 1000);
        }
        if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7) == GPIO_PIN_RESET) {
        	if(startstop==0){
        		HAL_ADC_Stop_DMA(&hadc1);
        		HAL_I2S_DMAStop(&hi2s3);
        		HAL_I2S_DeInit(&hi2s3);
        		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
        		HAL_StatusTypeDef result = HAL_I2S_Init(&hi2s3);
        		fresult = f_open(&file, "isengard.wav", 1);
        		f_lseek(&file, 44);
        		HAL_TIM_Base_Start_IT(&htim6);
        	}
        	else {
        		HAL_TIM_Base_Stop_IT(&htim6);
        		HAL_I2S_DeInit(&hi2s3);
        		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
        		HAL_StatusTypeDef result = HAL_I2S_Init(&hi2s3);
        		fresult = f_close(&file);
        		HAL_ADC_Start_IT(&hadc1);
        		HAL_ADC_Start_DMA(&hadc1, data1, SAMPLE);
        		startstop=0;
        	}

        }
        if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8) == GPIO_PIN_RESET) {
        	if(startstop==0){
        		HAL_ADC_Stop_DMA(&hadc1);
        		HAL_I2S_DMAStop(&hi2s3);
        		HAL_I2S_DeInit(&hi2s3);
        		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
        		HAL_StatusTypeDef result = HAL_I2S_Init(&hi2s3);
        		fresult = f_open(&file, "laserap.wav", 1);
        		f_lseek(&file, 44);
        		HAL_TIM_Base_Start_IT(&htim6);
        	}
        	else {
        		HAL_TIM_Base_Stop_IT(&htim6);
        		HAL_I2S_DeInit(&hi2s3);
        		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
        		HAL_StatusTypeDef result = HAL_I2S_Init(&hi2s3);
        		fresult = f_close(&file);
        		HAL_ADC_Start_IT(&hadc1);
        		HAL_ADC_Start_DMA(&hadc1, data1, SAMPLE);
        		startstop=0;
        	}
        }
        if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == GPIO_PIN_RESET) {
        	if(startstop==0){
        		HAL_ADC_Stop_DMA(&hadc1);
        		HAL_I2S_DMAStop(&hi2s3);
        		HAL_I2S_DeInit(&hi2s3);
        		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
        		HAL_StatusTypeDef result = HAL_I2S_Init(&hi2s3);
        		fresult = f_open(&file, "snoop.wav", 1);
        		f_lseek(&file, 44);
        		HAL_TIM_Base_Start_IT(&htim6);
        	}
        	else {
        		HAL_TIM_Base_Stop_IT(&htim6);
        		HAL_I2S_DeInit(&hi2s3);
        		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
        		HAL_StatusTypeDef result = HAL_I2S_Init(&hi2s3);
        		fresult = f_close(&file);
        		HAL_ADC_Start_IT(&hadc1);
        		HAL_ADC_Start_DMA(&hadc1, data1, SAMPLE);
        		startstop=0;
        	}
        }
        if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10) == GPIO_PIN_RESET) {
        	if(startstop==0){
        		HAL_ADC_Stop_DMA(&hadc1);
        		HAL_I2S_DMAStop(&hi2s3);
        		HAL_I2S_DeInit(&hi2s3);
        		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
        		HAL_StatusTypeDef result = HAL_I2S_Init(&hi2s3);
        		fresult = f_open(&file, "star.wav", 1);
        		f_lseek(&file, 44);
        		HAL_TIM_Base_Start_IT(&htim6);
        	}
        	else {
        		HAL_TIM_Base_Stop_IT(&htim6);
        		HAL_I2S_DeInit(&hi2s3);
        		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
        		HAL_StatusTypeDef result = HAL_I2S_Init(&hi2s3);
        		fresult = f_close(&file);
        		HAL_ADC_Start_IT(&hadc1);
        		HAL_ADC_Start_DMA(&hadc1, data1, SAMPLE);
        		startstop=0;
        	}
        }
        HAL_TIM_Base_Stop_IT(&htim3);
    }
    if (htim->Instance == TIM6){
    	startstop=1;
    	fresult = f_read(&file, SDbuffer, SDSIZE, &bytes_read);
    	tempbuff[0]=SDbuffer[0]*128;
    	HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)tempbuff, SDSIZE);
    	if(f_eof(&file)){
    		HAL_TIM_Base_Stop_IT(&htim6);
    		HAL_I2S_DeInit(&hi2s3);
    		hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    		HAL_StatusTypeDef result = HAL_I2S_Init(&hi2s3);
    		fresult = f_close(&file);
    		HAL_ADC_Start_IT(&hadc1);
    		HAL_ADC_Start_DMA(&hadc1, data1, SAMPLE);
    		startstop=0;
    	}
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
    HAL_TIM_Base_Start_IT(&htim3);
}
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  CS43_Init(hi2c1, MODE_I2S); //MODE_ANALOG
  CS43_SetVolume(80); //0 - 100,, 40 - MAX bo inaczej zjada za duzo pradu
  CS43_Enable_RightLeft(CS43_RIGHT_LEFT);
  CS43_Start();

  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_TIM_Base_Start_IT(&htim2);

  htim2.Instance->ARR = 139 ;
  htim2.Instance->PSC = 59999;
//  HAL_I2S_DeInit(&hi2s3);
//  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
//  HAL_StatusTypeDef result = HAL_I2S_Init(&hi2s3);

  HAL_ADC_Start_IT(&hadc1);

  //ADC to DMA
  HAL_ADC_Start_DMA(&hadc1, data1, SAMPLE);
  //SD Init
  fresult = f_mount(&FatFs, "",1);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  htim2.Init.Prescaler = 139;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 499;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5109;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 PE5 PE6 
                           PE7 PE8 PE9 PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
