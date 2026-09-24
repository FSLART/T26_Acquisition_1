/**
 * @file    analog.c
 * @brief   Analog inputs: APPS1, APPS2, front brake pressure, MCU temperature, VDDA.
 *          ADC1 scans continuously into a DMA buffer, values are averaged on Analog_Update().
 */
#include "analog.h"
#include "adc.h"

// DMA (circular) keeps ADC_OVERSAMPLE full sweeps of the ADC1 sequence in RAM, averaged when read.
// One sweep = 5 x (480 + 12) cycles @ 24 MHz ~ 102 us -> buffer holds the last ~1.6 ms of samples.
#define ADC_CHANNEL_COUNT       5
#define ADC_OVERSAMPLE          16
#define ADC_FULL_SCALE          4095.0f

// Sequence order, must match ADC1 ranks in the .ioc
#define ADC_IDX_APPS1           0   // PA7  - ADC1_IN7
#define ADC_IDX_APPS2           1   // PB0  - ADC1_IN8
#define ADC_IDX_BRAKE_PRESSURE  2   // PB1  - ADC1_IN9
#define ADC_IDX_TEMPSENSOR      3
#define ADC_IDX_VREFINT         4

// --- FRONT BRAKE PRESSURE SENSOR ---
// TODO: placeholder values, confirm against sensor datasheet and board divider
#define BRK_SENSOR_V_MIN        0.5f    // Sensor output at 0 bar [V]
#define BRK_SENSOR_V_MAX        4.5f    // Sensor output at full scale [V]
#define BRK_SENSOR_P_MAX_BAR    200.0f  // Full scale pressure [bar]
#define BRK_DIVIDER_RATIO       0.66f   // V_pin / V_sensor of the input divider

extern DMA_HandleTypeDef hdma_adc1;

static volatile uint16_t adc_dma_buf[ADC_OVERSAMPLE * ADC_CHANNEL_COUNT];

static float apps1_counts = 0.0f;
static float apps2_counts = 0.0f;
static float brake_pressure_bar = 0.0f;
static float vdda_v = 3.3f;
static int32_t mcu_temp_c = 0;

void Analog_Init(void)
{
    // Start ADC scan, DMA keeps adc_dma_buf updated forever (circular)
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_OVERSAMPLE * ADC_CHANNEL_COUNT) != HAL_OK) {
        Error_Handler();
    }
    // Nothing to do on half/full transfer, drop those IRQs (they would fire every ~0.8 ms)
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT);
}

// Average of the last ADC_OVERSAMPLE samples of one sequence channel, in ADC counts (0..4095)
static float Analog_Average(uint32_t channel_idx)
{
    uint32_t sum = 0;

    for (uint32_t i = channel_idx; i < ADC_OVERSAMPLE * ADC_CHANNEL_COUNT; i += ADC_CHANNEL_COUNT) {
        sum += adc_dma_buf[i];
    }

    return (float)sum / ADC_OVERSAMPLE;
}

// Pin voltage -> sensor voltage -> bar, clamped to sensor range
static float Analog_BrakePressureFromCounts(float counts)
{
    float v_sensor = (counts * vdda_v / ADC_FULL_SCALE) / BRK_DIVIDER_RATIO;
    float pressure = (v_sensor - BRK_SENSOR_V_MIN) * BRK_SENSOR_P_MAX_BAR / (BRK_SENSOR_V_MAX - BRK_SENSOR_V_MIN);

    if (pressure < 0.0f) {
        return 0.0f;
    }
    if (pressure > BRK_SENSOR_P_MAX_BAR) {
        return BRK_SENSOR_P_MAX_BAR;
    }
    return pressure;
}

void Analog_Update(void)
{
    // Real VDDA from the factory-calibrated internal reference, corrects every pin voltage
    uint32_t vrefint = (uint32_t)Analog_Average(ADC_IDX_VREFINT);
    if (vrefint > 0) {
        uint32_t vdda_mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vrefint, LL_ADC_RESOLUTION_12B);
        vdda_v = vdda_mv / 1000.0f;
        mcu_temp_c = __LL_ADC_CALC_TEMPERATURE(vdda_mv, (uint32_t)Analog_Average(ADC_IDX_TEMPSENSOR),
                                               LL_ADC_RESOLUTION_12B);
    }

    apps1_counts = Analog_Average(ADC_IDX_APPS1);
    apps2_counts = Analog_Average(ADC_IDX_APPS2);
    brake_pressure_bar = Analog_BrakePressureFromCounts(Analog_Average(ADC_IDX_BRAKE_PRESSURE));
}

float Analog_GetApps1Counts(void)
{
    return apps1_counts;
}

float Analog_GetApps2Counts(void)
{
    return apps2_counts;
}

float Analog_GetBrakePressureBar(void)
{
    return brake_pressure_bar;
}

float Analog_GetVdda(void)
{
    return vdda_v;
}

int32_t Analog_GetMcuTempC(void)
{
    return mcu_temp_c;
}
