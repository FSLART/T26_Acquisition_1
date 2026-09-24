/**
 * @file    analog.c
 * @brief   Analog inputs: APPS1, APPS2, front brake pressure, MCU temperature, VDDA.
 *          ADC1 scans continuously into a DMA buffer. Every Analog_Update() (10 ms) the last
 *          ADC_OVERSAMPLE DMA samples of each channel are averaged (no extra moving average:
 *          the 100 ms window of the original firmware made the APPS lag and step).
 */
#include "analog.h"
#include "adc.h"

// DMA (circular) keeps ADC_OVERSAMPLE full sweeps of the ADC1 sequence in RAM.
// One sweep = 5 x (480 + 12) cycles @ 24 MHz ~ 102 us -> buffer holds the last ~1.6 ms of samples.
#define ADC_CHANNEL_COUNT       5
#define ADC_OVERSAMPLE          16
#define ADC_FULL_SCALE          4095.0f

// Sequence order, must match ADC1 ranks in the .ioc
#define ADC_IDX_APPS1           0   // PA7  - ADC1_IN7
#define ADC_IDX_APPS2           1   // PB0  - ADC1_IN8
#define ADC_IDX_BRK_PRESS       2   // PB1  - ADC1_IN9
#define ADC_IDX_TEMPSENSOR      3
#define ADC_IDX_VREFINT         4
#define ADC_SENSOR_COUNT        3   // External sensors (first 3 ranks)

// VDDA measured from VREFINT is only trusted inside this range, otherwise nominal 3.3 V is used
#define VDDA_NOMINAL_V          3.3f
#define VDDA_MIN_V              2.9f
#define VDDA_MAX_V              3.6f

// --- FRONT BRAKE PRESSURE SENSOR (values from the original firmware) ---
// 5 V ratiometric sensor, 0.5 V = 0 bar, 4.5 V = 140 bar, into a 0.667 divider to the ADC pin
#define BRK_DIVIDER_RATIO       0.667f      // V_pin / V_sensor
#define BRK_SENSOR_SUPPLY_V     5.0f        // Sensor output clamp [V]
#define BRK_SENSOR_OFFSET_V     0.5f        // Sensor output at 0 bar [V]
#define BRK_SENSOR_SENSITIVITY  0.02857f    // [V/bar] = 4.0 V / 140 bar
#define BRK_SENSOR_MAX_BAR      140.0f      // Full scale [bar]

extern DMA_HandleTypeDef hdma_adc1;

static volatile uint16_t adc_dma_buf[ADC_OVERSAMPLE * ADC_CHANNEL_COUNT];

static float filtered[ADC_SENSOR_COUNT];

static float brake_pressure_bar = 0.0f;
static float vdda_v = VDDA_NOMINAL_V;
static int32_t mcu_temp_c = 0;

void Analog_Init(void) {
	// Start ADC scan, DMA keeps adc_dma_buf updated forever (circular)
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_dma_buf, ADC_OVERSAMPLE * ADC_CHANNEL_COUNT) != HAL_OK) {
		Error_Handler();
	}
	// Nothing to do on half/full transfer, drop those IRQs (they would fire every ~0.8 ms)
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT);
}

// Average of the last ADC_OVERSAMPLE DMA samples of one sequence channel, in ADC counts (0..4095)
static float Analog_OversampleAverage(uint32_t channel_idx) {
	uint32_t sum = 0;

	for (uint32_t i = channel_idx; i < ADC_OVERSAMPLE * ADC_CHANNEL_COUNT; i += ADC_CHANNEL_COUNT) {
		sum += adc_dma_buf[i];
	}

	return (float) sum / ADC_OVERSAMPLE;
}

// Sensor voltage -> bar, clamped to the sensor range (same steps as the original firmware)
static float Analog_BrakePressureFromCounts(float counts) {
	float volts = (counts * vdda_v / ADC_FULL_SCALE) / BRK_DIVIDER_RATIO;

	if (volts < 0.0f) {
		volts = 0.0f;
	} else if (volts > BRK_SENSOR_SUPPLY_V) {
		volts = BRK_SENSOR_SUPPLY_V;
	}

	float pressure = 0.0f;
	if (volts > BRK_SENSOR_OFFSET_V) {
		pressure = (volts - BRK_SENSOR_OFFSET_V) / BRK_SENSOR_SENSITIVITY;
	}

	if (pressure > BRK_SENSOR_MAX_BAR) {
		pressure = BRK_SENSOR_MAX_BAR;
	}
	return pressure;
}

void Analog_Update(void) {
	// Real VDDA from the factory-calibrated internal reference, corrects the brake sensor voltage
	uint32_t vrefint = (uint32_t) Analog_OversampleAverage(ADC_IDX_VREFINT);
	if (vrefint > 0) {
		uint32_t vdda_mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vrefint, LL_ADC_RESOLUTION_12B);
		float measured = vdda_mv / 1000.0f;

		vdda_v = (measured >= VDDA_MIN_V && measured <= VDDA_MAX_V) ? measured : VDDA_NOMINAL_V;
		mcu_temp_c = __LL_ADC_CALC_TEMPERATURE(vdda_mv, (uint32_t )Analog_OversampleAverage(ADC_IDX_TEMPSENSOR),
				LL_ADC_RESOLUTION_12B);
	}

	for (uint32_t ch = 0; ch < ADC_SENSOR_COUNT; ch++) {
		filtered[ch] = Analog_OversampleAverage(ch);
	}
	brake_pressure_bar = Analog_BrakePressureFromCounts(filtered[ADC_IDX_BRK_PRESS]);
}

float Analog_GetApps1Counts(void) {
	return filtered[ADC_IDX_APPS1];
}

float Analog_GetApps2Counts(void) {
	return filtered[ADC_IDX_APPS2];
}

float Analog_GetBrakeCounts(void) {
	return filtered[ADC_IDX_BRK_PRESS];
}

float Analog_GetBrakePressureBar(void) {
	return brake_pressure_bar;
}

float Analog_GetVdda(void) {
	return vdda_v;
}

int32_t Analog_GetMcuTempC(void) {
	return mcu_temp_c;
}
