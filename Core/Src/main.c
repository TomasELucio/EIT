/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Firmware EIT con filtro de mediana/promedio y fase estable
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // Requerido para qsort
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
#define DAC_SINE_SAMPLES 8

const uint16_t SineWave_8Points[DAC_SINE_SAMPLES] = {
    2048, /* Angle   0°: 1.65V (Offset central VDD/2) */
    2912, /* Angle  45°: 2.34V */
    3269, /* Angle  90°: 2.63V (Pico Máximo) */
    2912, /* Angle 135°: 2.34V */
    2048, /* Angle 180°: 1.65V */
    1184, /* Angle 225°: 0.96V */
     827, /* Angle 270°: 0.67V (Pico Mínimo) */
    1184  /* Angle 315°: 0.96V */
};

#define ADC_BUF_LEN 1024
#define FFT_SIZE    1024
#define TARGET_BIN  128 /* (56250 Hz / 450000 Hz) * 1024 = 128 */

uint16_t adc_buffer[ADC_BUF_LEN];
volatile uint8_t adc_ready_flag = 0;

float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t hanning_window[FFT_SIZE];

arm_rfft_fast_instance_f32 fft_handler;

/* Variables para almacenar los datos procesados */
float32_t real_component = 0.0f;
float32_t imag_component = 0.0f;
float32_t magnitude_raw   = 0.0f;
float32_t phase_rad       = 0.0f;
float32_t phase_deg       = 0.0f;

/* Retardo sistemático del AFE */
const float32_t DELTA_PHASE_CALIB = 124.75f * (M_PI / 180.0f);

/* Arreglos para filtrado por mediana/promedio en ~500 ms (220 muestras) */
#define AVG_SAMPLES 220
float32_t mag_history[AVG_SAMPLES];
float32_t phase_history[AVG_SAMPLES];
uint16_t sample_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void); /* Declaración explícita global para adc.c, dac.c, tim.c, etc. */
/* USER CODE BEGIN PFP */
int compare_floats(const void *a, const void *b);
/* USER CODE END PFP */

/* Private user code --------------------------------------------------------- */
/* USER CODE BEGIN 0 */
/* Función auxiliar para ordenar arreglos de menor a mayor */
int compare_floats(const void *a, const void *b)
{
    float32_t diff = *(float32_t*)a - *(float32_t*)b;
    if (diff < 0.0f) return -1;
    if (diff > 0.0f) return 1;
    return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  /* 1. Precalcular Ventana de Hanning */
  for (uint16_t i = 0; i < FFT_SIZE; i++)
  {
      hanning_window[i] = 0.5f * (1.0f - cosf((2.0f * M_PI * i) / (FFT_SIZE - 1)));
  }

  /* 2. Inicializar RFFT de 1024 puntos */
  arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);

  /* 3. Iniciar Temporizador a 450 kHz */
  HAL_TIM_Base_Start(&htim2);

  /* 4. Iniciar DAC por DMA */
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)SineWave_8Points, 8, DAC_ALIGN_12B_R);

  /* 5. Iniciar ADC por DMA (1024 muestras) */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (adc_ready_flag)
      {
          adc_ready_flag = 0;

          /* 1. Centrar muestras e imbuir Ventana de Hanning */
          for (uint16_t i = 0; i < FFT_SIZE; i++)
          {
              float32_t sample_centered = ((float32_t)adc_buffer[i]) - 2048.0f;
              fft_input[i] = sample_centered * hanning_window[i];
          }

          /* 2. FFT de 1024 puntos */
          arm_rfft_fast_f32(&fft_handler, fft_input, fft_output, 0);

          /* 3. Extracción del BIN 128 (56.25 kHz) */
          real_component = fft_output[2 * TARGET_BIN];
          imag_component = fft_output[2 * TARGET_BIN + 1];

          /* 4. Magnitud y Fase */
          magnitude_raw = sqrtf(real_component * real_component + imag_component * imag_component);
          phase_rad = atan2f(imag_component, real_component) - DELTA_PHASE_CALIB;
          phase_deg = phase_rad * (180.0f / M_PI);

          /* NORMALIZACIÓN CIRCULAR DE FASE AL RANGO [-180, +180] */
          while (phase_deg > 180.0f)  phase_deg -= 360.0f;
          while (phase_deg < -180.0f) phase_deg += 360.0f;

          /* Almacenar en histórico para el filtro por mediana */
          mag_history[sample_count] = magnitude_raw;
          phase_history[sample_count] = phase_deg;
          sample_count++;

          /* Al completar ~500 ms (220 ráfagas) */
          if (sample_count >= AVG_SAMPLES)
          {
              /* Ordenar históricamente */
              qsort(mag_history, AVG_SAMPLES, sizeof(float32_t), compare_floats);
              qsort(phase_history, AVG_SAMPLES, sizeof(float32_t), compare_floats);

              /* Promediar el 70% central descartando picos (outliers) */
              uint16_t start_idx = (uint16_t)(AVG_SAMPLES * 0.15f);
              uint16_t end_idx   = (uint16_t)(AVG_SAMPLES * 0.85f);
              uint16_t valid_count = end_idx - start_idx;

              float32_t mag_sum = 0.0f;
              float32_t phase_sum = 0.0f;

              for (uint16_t i = start_idx; i < end_idx; i++)
              {
                  mag_sum += mag_history[i];
                  phase_sum += phase_history[i];
              }

              float32_t mag_clean = mag_sum / valid_count;
              float32_t phase_clean = phase_sum / valid_count;

              /* Transmitir trama limpia cada 500 ms por USART2 */
              char uart_buf[64];
              int len = sprintf(uart_buf, "%.2f,%.2f\r\n", mag_clean, phase_clean);
              HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 10);

              sample_count = 0;
          }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        adc_ready_flag = 1; /* Señalizar al bucle principal */
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
