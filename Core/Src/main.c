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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "../DBC/autonomous_t26.h"
#include "../DBC/powertrain_t26.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- ADC ---
// DMA (circular) keeps ADC_OVERSAMPLE full sweeps of the ADC1 sequence in RAM, averaged when read.
// One sweep = 5 x (480 + 12) cycles @ 24 MHz ~ 102 us -> buffer holds the last ~1.6 ms of samples.
#define ADC_CHANNEL_COUNT      5
#define ADC_OVERSAMPLE         16
#define ADC_FULL_SCALE         4095.0f

// Sequence order, must match ADC1 ranks in the .ioc
#define ADC_IDX_APPS1          0   // PA7  - ADC1_IN7
#define ADC_IDX_APPS2          1   // PB0  - ADC1_IN8
#define ADC_IDX_BRAKE_PRESSURE 2   // PB1  - ADC1_IN9
#define ADC_IDX_TEMPSENSOR     3
#define ADC_IDX_VREFINT        4

// --- FRONT BRAKE PRESSURE SENSOR ---
// TODO: placeholder values, confirm against sensor datasheet and board divider
#define BRK_SENSOR_V_MIN       0.5f    // Sensor output at 0 bar [V]
#define BRK_SENSOR_V_MAX       4.5f    // Sensor output at full scale [V]
#define BRK_SENSOR_P_MAX_BAR   200.0f  // Full scale pressure [bar]
#define BRK_DIVIDER_RATIO      0.66f   // V_pin / V_sensor of the input divider
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t time_ms = 0;

// ADC
extern DMA_HandleTypeDef hdma_adc1;
volatile uint16_t adc_dma_buf[ADC_OVERSAMPLE * ADC_CHANNEL_COUNT];

float vdda_v = 3.3f;              // Measured analog supply, from VREFINT
float brake_pressure_bar = 0.0f;  // Front brake pressure (PB1)

// --- WHEEL SPEED SENSORS CONFIGURATION ---
typedef struct {
    volatile uint16_t last_capture;     // Store timer reading from previous tooth
    volatile uint16_t delta_counts;    // Time interval between 2 teeth (in microseconds)
    volatile float rpm;                 // Calculated speed in RPM
    volatile uint8_t first_capture;     // Flag to ignore first pulse at startup
    volatile uint32_t last_pulse_ms;    // Timestamp (in ms) of the last detected tooth
} WheelSensor;

// Instances for Wheel Sensors
WheelSensor sensor_wheel_1 = {0, 0, 0.0f, 1, 0}; // Connected to PC8 (TIM3_CH3)
WheelSensor sensor_wheel_2 = {0, 0, 0.0f, 1, 0}; // Connected to PC9 (TIM3_CH4)

// Tasks Prototypes
void execute_immediate_tasks(void);
void execute_10ms_tasks(void);
void execute_50ms_tasks(void);
void execute_100ms_tasks(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

float MeasureWheelRPM(uint16_t delta_us);
float ADC_Average(uint32_t channel_idx);
void UpdateAnalogInputs(void);
void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t *data, uint32_t dlc);
void SendAPPS(void);
void SendAQT1(void);

int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*) data, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
    if (htim->Instance == TIM7) {
        time_ms++;
    }
}

// Hardware Capture Interrupt: Triggered automatically on rising edges (PC8 & PC9)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {

        // --- SENSOR 1: PC8 (TIM3 Channel 3) ---
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            uint16_t current_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

            if (!sensor_wheel_1.first_capture) {
                // 16-bit subtraction handles counter overflow (65535 -> 0) automatically
                sensor_wheel_1.delta_counts = (uint16_t)(current_capture - sensor_wheel_1.last_capture);

                if (sensor_wheel_1.delta_counts > 0) {
                    sensor_wheel_1.rpm = MeasureWheelRPM(sensor_wheel_1.delta_counts);
                    sensor_wheel_1.last_pulse_ms = time_ms; // Save timestamp of last pulse
                }
            } else {
                sensor_wheel_1.first_capture = 0;
            }
            sensor_wheel_1.last_capture = current_capture;
        }

        // --- SENSOR 2: PC9 (TIM3 Channel 4) ---
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            uint16_t current_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

            if (!sensor_wheel_2.first_capture) {
                sensor_wheel_2.delta_counts = (uint16_t)(current_capture - sensor_wheel_2.last_capture);

                if (sensor_wheel_2.delta_counts > 0) {
                    sensor_wheel_2.rpm = MeasureWheelRPM(sensor_wheel_2.delta_counts);
                    sensor_wheel_2.last_pulse_ms = time_ms; // Save timestamp of last pulse
                }
            } else {
                sensor_wheel_2.first_capture = 0;
            }
            sensor_wheel_2.last_capture = current_capture;
        }
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);

  // Start Input Capture for both wheel speed sensors
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3); // PC8
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4); // PC9

  // Start ADC scan, DMA keeps adc_dma_buf updated forever (circular)
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_OVERSAMPLE * ADC_CHANNEL_COUNT) != HAL_OK) {
      Error_Handler();
  }
  // Nothing to do on half/full transfer, drop those IRQs (they would fire every ~0.8 ms)
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        // Execute immediate tasks
        execute_immediate_tasks();

        static uint32_t previus_tick_10ms = 0;
        static uint32_t previus_tick_50ms = 0;
        static uint32_t previus_tick_100ms = 0;

        // Execute 10ms Tasks
        if (time_ms - previus_tick_10ms >= 10) {
            execute_10ms_tasks();
            previus_tick_10ms = time_ms;
        }

        // Execute 50ms Tasks
        if (time_ms - previus_tick_50ms >= 50) {
            execute_50ms_tasks();
            previus_tick_50ms = time_ms;
        }

        // Execute 100ms Tasks
        if (time_ms - previus_tick_100ms >= 100) {
            execute_100ms_tasks();
            previus_tick_100ms = time_ms;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void execute_immediate_tasks() {

}

void execute_10ms_tasks() {
    HAL_IWDG_Refresh(&hiwdg);

    // Standstill Detection: Force 0 RPM if no pulse received in the last 300ms
    if (time_ms - sensor_wheel_1.last_pulse_ms > 300) {
        sensor_wheel_1.rpm = 0.0f;
    }

    if (time_ms - sensor_wheel_2.last_pulse_ms > 300) {
        sensor_wheel_2.rpm = 0.0f;
    }

    UpdateAnalogInputs();

    SendAPPS();   // CAN2 - powertrain
    SendAQT1();   // CAN1 - autonomous
}

void execute_50ms_tasks() {

}

void execute_100ms_tasks() {
    HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin); // HEARTBEAT

    // Sensors debug
    printf("Wheel 1: %.1f RPM | Wheel 2: %.1f RPM\r\n", sensor_wheel_1.rpm, sensor_wheel_2.rpm);
    int32_t mcu_temp_c = __LL_ADC_CALC_TEMPERATURE((uint32_t)(vdda_v * 1000.0f),
                                                   (uint32_t)ADC_Average(ADC_IDX_TEMPSENSOR),
                                                   LL_ADC_RESOLUTION_12B);
    printf("APPS1: %.1f | APPS2: %.1f | Brake: %.1f bar | VDDA: %.3f V | MCU: %ld C\r\n",
           ADC_Average(ADC_IDX_APPS1), ADC_Average(ADC_IDX_APPS2), brake_pressure_bar, vdda_v, mcu_temp_c);
}

// CAN

// Queues a standard-ID data frame. If all 3 TX mailboxes are still pending (bus-off, no ACK,
// cable unplugged) the stale frames are aborted so fresh data goes out once the bus is back.
void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t *data, uint32_t dlc)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;
    header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        return;
    }

    (void)HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

// APPS_ADC_Raw (0x50) on CAN2 - powertrain: averaged ADC counts, scaling done by the VCU
void SendAPPS(void)
{
    struct powertrain_t26_apps_adc_raw_t msg = {0};
    uint8_t data[8];

    msg.apps1_raw = powertrain_t26_apps_adc_raw_apps1_raw_encode(ADC_Average(ADC_IDX_APPS1));
    msg.apps2_raw = powertrain_t26_apps_adc_raw_apps2_raw_encode(ADC_Average(ADC_IDX_APPS2));

    int len = powertrain_t26_apps_adc_raw_pack(data, &msg, sizeof(data));
    if (len > 0) {
        CAN_Send(&hcan2, POWERTRAIN_T26_APPS_ADC_RAW_FRAME_ID, data, (uint32_t)len);
    }
}

// AQT1 (0x710) on CAN1 - autonomous: front brake pressure
void SendAQT1(void)
{
    struct autonomous_t26_aqt1_t msg = {0};
    uint8_t data[8];

    msg.frt_brk_press = autonomous_t26_aqt1_frt_brk_press_encode(brake_pressure_bar);
    msg.res = 0;   // Not wired to this board
    msg.bots = 0;  // Not wired to this board

    int len = autonomous_t26_aqt1_pack(data, &msg, sizeof(data));
    if (len > 0) {
        CAN_Send(&hcan1, AUTONOMOUS_T26_AQT1_FRAME_ID, data, (uint32_t)len);
    }
}

// Measurements

// Average of the last ADC_OVERSAMPLE samples of one sequence channel, in ADC counts (0..4095)
float ADC_Average(uint32_t channel_idx)
{
    uint32_t sum = 0;

    for (uint32_t i = channel_idx; i < ADC_OVERSAMPLE * ADC_CHANNEL_COUNT; i += ADC_CHANNEL_COUNT) {
        sum += adc_dma_buf[i];
    }

    return (float)sum / ADC_OVERSAMPLE;
}

// Converts averaged ADC counts to physical values
void UpdateAnalogInputs(void)
{
    // Real VDDA from the factory-calibrated internal reference, corrects every pin voltage
    uint32_t vrefint = (uint32_t)ADC_Average(ADC_IDX_VREFINT);
    if (vrefint > 0) {
        vdda_v = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vrefint, LL_ADC_RESOLUTION_12B) / 1000.0f;
    }

    // Front brake pressure: pin voltage -> sensor voltage -> bar, clamped to sensor range
    float v_sensor = (ADC_Average(ADC_IDX_BRAKE_PRESSURE) * vdda_v / ADC_FULL_SCALE) / BRK_DIVIDER_RATIO;
    float pressure = (v_sensor - BRK_SENSOR_V_MIN) * BRK_SENSOR_P_MAX_BAR / (BRK_SENSOR_V_MAX - BRK_SENSOR_V_MIN);

    if (pressure < 0.0f) {
        pressure = 0.0f;
    } else if (pressure > BRK_SENSOR_P_MAX_BAR) {
        pressure = BRK_SENSOR_P_MAX_BAR;
    }
    brake_pressure_bar = pressure;
}

// Calculates Wheel RPM based on time between teeth (microseconds)
float MeasureWheelRPM(uint16_t delta_us)
{
    const float TIMER_FREQ_HZ = 1000000.0f; // 1 MHz Timer Clock -> 1 us resolution
    const float NUMBER_OF_TEETH = 20.0f;   // Disc teeth count

    if (delta_us == 0) {
        return 0.0f;
    }

    // Formula: RPM = (60 sec * 1,000,000 us) / (20 teeth * delta_us)
    return (60.0f * TIMER_FREQ_HZ) / (NUMBER_OF_TEETH * (float)delta_us);
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
    while (1) {
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
