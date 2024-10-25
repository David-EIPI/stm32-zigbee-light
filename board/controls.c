/*
 * controls.c
 *
 *  Created on: Aug 31, 2024
 *      Author: shir
 */

#include <math.h>

#include "controls.h"
#include "limits.h"
#include "main.h"

#define LOW_PASS_BITS 7

#define ADC_LOW_PASS 128 // a coefficient in AMBIENT_RANGE

#define AVG_SLOPE (430)
#define V_AT_25C  (164)
#define V_REF_INT (1210)

struct controlsRec controls = { 0 };


/* Slide switches */
static uint32_t switch1_gpio_pin[] = {
		SW1_P1_Pin,
		SW1_P2_Pin,
		SW1_P4_Pin,
		SW1_P5_Pin,
};

static GPIO_TypeDef * switch1_gpio_port[] = {
		SW1_P1_GPIO_Port,
		SW1_P2_GPIO_Port,
		SW1_P4_GPIO_Port,
		SW1_P5_GPIO_Port,
};

static uint32_t switch2_gpio_pin[] = {
		SW2_P1_Pin,
		SW2_P2_Pin,
		SW2_P4_Pin,
		SW2_P5_Pin,
};

static GPIO_TypeDef * switch2_gpio_port[] = {
		SW2_P1_GPIO_Port,
		SW2_P2_GPIO_Port,
		SW2_P4_GPIO_Port,
		SW2_P5_GPIO_Port,
};


static uint32_t sw1_states[lengthof(switch1_gpio_pin)] = { 0 };
static uint32_t sw2_states[lengthof(switch1_gpio_pin)] = { 0 };

/*
 * @var Analog inputs: ambient light, sensitivity pot
 *      and on-chip voltage and temperature sensors
 *  */
volatile uint16_t adcdmavals[4] = {0};

static int adc_updated = 1;

/*
 * @var Motion sensor data register
 * */
unsigned dl_data[2] = { 0 };
unsigned motion_sensor_dl_cnt = 0;

/*
 * @var Motion sensor configuration register
 * */
union {
	struct {
		unsigned count_mode:1;      // count with (0) or without (1) BPF sign change
		unsigned reserved2:1;       // Reserved: Must be set to dec 0
		unsigned hpf_cut_off:1;     // 0: 0.4 Hz 1: 0.2 Hz
		unsigned reserved1:2;       // Reserved: Must be set to dec 2
		unsigned signal_source:2;   // 0: PIR (BPF) 1: PIR (LPF) 2: Reserved 3: Temperature Sensor
		unsigned operation_mode:2;  // 0: Forced Readout 1: Interrupt Readout 2: Wake Up 3: Reserved
		unsigned window_time:2;     // = 2 s + [Reg Val] · 2 s
		unsigned pulse_counter:2;   // = 1 + [Reg Val]
		unsigned blind_time:4;      // = 0.5 s + [Reg Val] · 0.5 s
		unsigned threshold:8;       // Detection threshold on BPF value
	};
	unsigned bits;
} config_register = {
		.count_mode = 0,
		.reserved2 = 0,
		.hpf_cut_off = 0,
		.reserved1 = 2,
		.signal_source = 0,
		.operation_mode = 2,
		.window_time = 0,
		.pulse_counter = 0,
		.blind_time = 0,
		.threshold = 200,
};



LL_GPIO_InitTypeDef init_input = {
		  .Pin = DLINK_Pin,
		  .Mode = LL_GPIO_MODE_INPUT,
		  .Pull = LL_GPIO_PULL_NO,
};

LL_GPIO_InitTypeDef init_output = {
		  .Pin = DLINK_Pin,
		  .Mode = LL_GPIO_MODE_OUTPUT,
		  .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		  .Pull = LL_GPIO_PULL_NO,
		  .Speed = LL_GPIO_SPEED_FREQ_HIGH,
};


/*
 * @brief Read PYD1588 data bytes.
 *        Here we do not use this data, but reading it resets the DIRECT LINE pin.
 *        This pin is set high upon positive motion detection and is not reset automatically.
 * */
void read_dl_data(void)
{
	dl_data[0] = 0;
	dl_data[1] = 0;

	LL_GPIO_SetOutputPin(DLINK_GPIO_Port, DLINK_Pin);

	LL_GPIO_Init(DLINK_GPIO_Port, &init_output);
	LL_GPIO_SetPinSpeed(DLINK_GPIO_Port, DLINK_Pin, LL_GPIO_SPEED_FREQ_HIGH);
	delay_us(120); // forced pulse

	int i;
	for (i = 0; i < 15; i++) {
		LL_GPIO_ResetOutputPin(DLINK_GPIO_Port, DLINK_Pin);
		delay_us(1);
		LL_GPIO_SetOutputPin(DLINK_GPIO_Port, DLINK_Pin);
		delay_us(1);
		LL_GPIO_SetPinMode(DLINK_GPIO_Port, DLINK_Pin, LL_GPIO_MODE_INPUT);
		delay_us(5);
		int bit = LL_GPIO_IsInputPinSet(DLINK_GPIO_Port, DLINK_Pin);
		dl_data[1] = (dl_data[1] << 1) | bit;
		LL_GPIO_SetPinMode(DLINK_GPIO_Port, DLINK_Pin, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(DLINK_GPIO_Port, DLINK_Pin, LL_GPIO_SPEED_FREQ_HIGH);
	}
	for (i = 0; i < 25; i++) {
		LL_GPIO_ResetOutputPin(DLINK_GPIO_Port, DLINK_Pin);
		delay_us(1);
		LL_GPIO_SetOutputPin(DLINK_GPIO_Port, DLINK_Pin);
		delay_us(1);
		LL_GPIO_SetPinMode(DLINK_GPIO_Port, DLINK_Pin, LL_GPIO_MODE_INPUT);
		delay_us(5);
		int bit = LL_GPIO_IsInputPinSet(DLINK_GPIO_Port, DLINK_Pin);
		dl_data[0] = (dl_data[0] << 1) | bit;
		LL_GPIO_SetPinMode(DLINK_GPIO_Port, DLINK_Pin, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(DLINK_GPIO_Port, DLINK_Pin, LL_GPIO_SPEED_FREQ_HIGH);
	}
	LL_GPIO_ResetOutputPin(DLINK_GPIO_Port, DLINK_Pin);
	delay_us(1250); // end
	LL_GPIO_Init(DLINK_GPIO_Port, &init_input);
}

/*
 * @brief Update the PYD1588 configuration.
 *        This transmits the configuration register contents
 *        including the possibly updated motion detection threshold.
 * */
void motion_send_configuration(uint8_t threshold)
{
	config_register.threshold = threshold;

	int mask = (1 << 24);

	int i;
	for (i = 0; i < 25; i++) {
		LL_GPIO_SetOutputPin(SERIN_GPIO_Port, SERIN_Pin);

		if (((config_register.bits & mask) != 0))
			LL_GPIO_SetOutputPin(SERIN_GPIO_Port, SERIN_Pin);
		else
			LL_GPIO_ResetOutputPin(SERIN_GPIO_Port, SERIN_Pin);

		delay_us(80);
		LL_GPIO_ResetOutputPin(SERIN_GPIO_Port, SERIN_Pin);
		mask >>= 1;
	}
}


/*
 * @brief Integer low-pass filter with (1 << LOW_PASS_BITS) transition steps.
 * */
uint32_t inputLowPassFilter(uint32_t current, uint32_t newval)
{
	unsigned result = current;
	if (result > 0) result -= 1;
	result += 2 * (!!newval);
	result &= (1 << LOW_PASS_BITS) - 1;
	return result;
}

/*
 * @brief Schmitt trigger at 1/4 and 3/4 thresholds.
 * */
uint32_t inputSchmittTrigger(unsigned oldval, unsigned level)
{
	unsigned result = oldval;
	if ((result != 0) && (level < (1 << LOW_PASS_BITS)/4) ) {
		result = 0;
	}
	if ((result == 0) && (level > (1 << LOW_PASS_BITS) - (1 << LOW_PASS_BITS)/4) ) {
		result = 1;
	}
	return result;
}

/*
 * a[n] = 40 * log2(n)
 * This will be divided by 30 at runtime, 30/40 = 0.75 = k = photoresistor constant
 */
static unsigned log_table[16] = {
		0, 0, 40, 63, 80, 93, 103, 112, 120, 127, 133, 138, 143, 148, 152, 156
};

static int32_t ambientSensor = 0;

static void read_temperature(void)
{
	int vref = (V_REF_INT * 4095) / adcdmavals[3];
//	int vsense = (vref * adcdmavals[2]) / 4095;
//	int tC = ((V_AT_25C - vsense) * 1000) / (AVG_SLOPE/10) + 250;
	int tC = 100* __LL_ADC_CALC_TEMPERATURE(vref, adcdmavals[2], LL_ADC_RESOLUTION_12B);
	controls.adc_temp = tC;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
/* Read the ambient brightness */
/* Bring the raw value to range */
	int adc_range = (1 + __HAL_ADC_DIGITAL_SCALE(ADC_GET_RESOLUTION(hadc)));
	int adc_val = adcdmavals[0];

	if (adc_val >= adc_range)
		adc_val = adc_range-1;
	/* Apply the LPF */
	adc_val = ambientSensor * (adc_range - ADC_LOW_PASS)
			+ (adc_val *  ADC_LOW_PASS) +
			((adc_val > ambientSensor) ? (adc_range-1) : 0);

	ambientSensor = adc_val / adc_range;
	controls.adc_voltage = ambientSensor;

/*
V = ADC *3.3 / 4095
R = 100k * (3.3 / V - 1)
I = (12000 ** (1/0.75) * 10 lux) / (R ** (1/0.75))
  = 2.74e6 / (R ** (1.0/0.75))

 */
  int32_t resistance = (100000 * 4095) / ambientSensor - 100000;
  if (resistance < 1)
	  resistance = 1;

  int res_log2 = 31 - __builtin_clz(resistance);
  if (res_log2 < 4)
	  res_log2 = log_table[resistance & 0xf];
  else
	  res_log2 = 40 * (res_log2 - 3) + log_table[(resistance >> (res_log2 - 3)) & 0xf];

  /*
   * res_log2 = 40 * log2(resistance)
   * illum = (30 * log2(R_10lux) - res_log2) / 3;
   * R_10lux = 274.7kOhm
   * Add 5 to stay in positive range, that is result is 10 * log2(Illuminance * 2^5)
   */
  int illum = ((30 * 5) + 642 - res_log2) / 3;
  if (illum < 0) illum = 0;
  controls.log2illum = illum;
  /* Read the motion sensitivity setting potentiometer */
/* Bring the raw value to range */
	controls.dbg = adcdmavals[1];
	adc_val = (adcdmavals[1] * (MSENSITIVITY_RANGE-1) + adc_range / 2) / adc_range;
	controls.sensitivity = adc_val;

	read_temperature();
	adc_updated = 1;
}

/*
 * @brief Read pin states with basic noise and jitter filtering.
 *        Also check the motion sensor and initiate ADC reading.
 * */
void readBoardControls(ADC_HandleTypeDef *hadc1)
{
	unsigned i;

	unsigned sw_pos = controls.slide_sw1;
	for (i = 0; i < lengthof(switch1_gpio_pin); i++) {

		sw1_states[i] = inputLowPassFilter(sw1_states[i],
				!LL_GPIO_IsInputPinSet(switch1_gpio_port[i], switch1_gpio_pin[i]) );

		unsigned oldval = controls.slide_sw1 == i + 1;

		if (inputSchmittTrigger(oldval, sw1_states[i]))
			sw_pos = i + 1;
	}
	controls.slide_sw1 = (uint8_t)sw_pos;

	sw_pos = controls.slide_sw2;
	for (i = 0; i < lengthof(switch2_gpio_pin); i++) {

		sw2_states[i] = inputLowPassFilter(sw2_states[i],
				!LL_GPIO_IsInputPinSet(switch2_gpio_port[i], switch2_gpio_pin[i]) );

		unsigned oldval = controls.slide_sw2 == i + 1;

		if (inputSchmittTrigger(oldval, sw2_states[i]))
			sw_pos = i + 1;

	}
	controls.slide_sw2 = (uint8_t)sw_pos;

	if (adc_updated) {
		adc_updated = 0;
		HAL_ADC_Start_DMA(hadc1, (uint32_t*)&adcdmavals[0], hadc1->Init.NbrOfConversion);
	}

	unsigned motion_detected = LL_GPIO_IsInputPinSet(DLINK_GPIO_Port, DLINK_Pin);
	controls.motion = motion_detected;
	if (motion_detected) {
		read_dl_data();
#if 0
		motion_sensor_dl_cnt += 1;
		if (motion_sensor_dl_cnt > 1000 || config_register.threshold != controls.sensitivity) {
			config_register.threshold = controls.sensitivity;
			motion_send_configuration();
		}
#endif
	}

}
