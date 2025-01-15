
#include "welder.h"
// #include "pcf8574.h"
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stdio.h"
#include "z_flash_W25QXXX.h"
// #include "led_control.h"

// #define DEBUG
#define true 1
#define false 0

// DC welding params given in A
uint8_t base_current, background_current, crater_current, start_current;

// AC welding params given in Hz and Duty % respectfully; Goes from 0 = H to 100 = 99%; 0 = DC; Duty behaves as duty
uint8_t ac_frequency, ac_duty;
float pulse_frequency;	// Pulse settings given in Hz
uint8_t pulse_duty;		// Pulse duty in %
// Slope settings given in mS
uint16_t upslope_time, downslope_time;
// Gas settings given in mS
uint16_t preflow_time, postflow_time;

const char *menu_string[11] = {"Preflow", "Start Curr", "Upslope", "Base Curr", "Downslope", "Crater", "Postflow", "Pulse Freq", "Pulse Duty", "AC Freq", "AC Duty"};  // Strings for menu
const char *mode_string[5] = {"DC", "AC", "PUL", "MMA", "TIG"};																										   // Mode strings for idle screen
const uint16_t current_threshold = 300;	 // Threshold at which we can tell that the arc is established given in raw ADC values
const uint16_t ignition_timeout = 500;	 // Time after which the arc should be going in mS
const uint8_t max_current = 200;		 // Amps
const uint8_t min_current = 5;			 // Amps
// const uint16_t debounce_time = 2;                      // mS
const uint16_t ignition_current = 5;  // Amps
const uint16_t crater_time = 100;	  // mS

uint8_t torch_trigger = false;	  // Flag for torch button
uint8_t encoder_button = false;	  // Flag for encoder button
uint8_t pedal_connected = false;  // Flag for detecting welding pedal
uint32_t adc_buffer[2];			  // ADC CH0 - pedal, ADC CH1 - current sense
uint8_t enc_press_time = 0;		  // Time that the encoder is pressed by [10mS]

enum State {  // States of the welding process
	idle,
	preflow,
	ignition,
	upslope,
	welding,
	downslope,
	crater,
	postflow
};

enum TriggerType {	// 2T and 4T trigger type
	_2T = 2,
	_4T = 4
};

enum WeldingMode {	// Available welding modes - beware to NOT use AC and DC at the same time
	DC = 1 << 0,
	AC = 1 << 1,
	Pulse = 1 << 2,
	MMA = 1 << 3
};

enum TriggerType trigger_type = _2T;  // 2T as default cus why not
enum WeldingMode welding_mode = DC;	  // DC as default
volatile enum State state = idle;	  // Starting state

void welder() {
	// Some default settings
	preflow_time = 1200;
	postflow_time = 2000;
	upslope_time = 1500;
	downslope_time = 2000;
	start_current = 5;
	base_current = 80;
	background_current = 50;
	crater_current = 5;
	pedal_connected = false;

	welding_mode |= Pulse;
	// welding_mode |= AC;
	welding_mode |= DC;
	pulse_duty = 20;
	pulse_frequency = 2;

	ac_frequency = 100;
	ac_duty = 70;

	// Init peripethials
	HAL_GPIO_WritePin(GAS_GPIO_Port, GAS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);

	ssd1306_Init();
	ssd1306_Fill(0);
	ssd1306_UpdateScreen();

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	 // Encoder timer

	HAL_ADC_Start_DMA(&hadc1, adc_buffer, 2);

	set_pwm_frequency(ac_frequency);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	// HAL_TIM_PWM_Start_IT(pwm_tim, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim11);

	while (1) {	 // main program loop
		state_machine();
	}
}

void idle_menu() {
	int16_t encoder_counter = __HAL_TIM_GET_COUNTER(&htim2);  // Get encoder "position"
	static int16_t enc_last = 0;							  // Last encoder position
	static uint8_t menu = 0;								  // Bool flag if we are in the manu
	static uint32_t last_millis = 0;						  // Last millis for blinking and stuff
	char str[10];											  // Buffer for display

#ifdef DEBUG
	ssd1306_Fill(0);
	ssd1306_SetCursor(10, 0);
	snprintf(str, sizeof(str), "%d", adc_buffer[0]);
	ssd1306_WriteString(str, Font_11x18, 1);
	ssd1306_SetCursor(80, 0);
	snprintf(str, sizeof(str), "%d", adc_buffer[1]);
	ssd1306_WriteString(str, Font_11x18, 1);

	ssd1306_SetCursor(10, 30);
	snprintf(str, sizeof(str), "%d", encoder_counter);
	ssd1306_WriteString(str, Font_11x18, 1);
	ssd1306_SetCursor(80, 30);
	snprintf(str, sizeof(str), "%d", state);
	ssd1306_WriteString(str, Font_11x18, 1);

#else
	ssd1306_Fill(0);				  // Clear display
	if (encoder_button && !menu) {	  // Check for encoder button and menu state
		menu = 1;					  // Menu = 0 - idle; Menu = 1 - detecting long press; Menu = 2 - sub_menu; Menu = 3 - main_menu
		last_millis = HAL_GetTick();  // Reset timer for menu timeout
		encoder_button = false;		  // Clear encoder button flag
		enc_press_time = 0;
	}

	if (state == idle) {															// Ignore encoder and button input when welding
		if (menu == 1 && HAL_GPIO_ReadPin(ENC_BUTTON_GPIO_Port, ENC_BUTTON_Pin)) {	// Go into sub menu at button release
			enc_press_time = 0;
			menu = 2;
		} else if (menu == 1 && enc_press_time > 20) {	// Detect long press
			enc_press_time = 0;
			menu = 3;
		}
	}

	if (menu == 2 && (welding_mode & MMA)) menu = 0;
	if (menu == 2 && state == idle) {
		if (sub_menu(&enc_last, &encoder_counter, &last_millis)) menu = 0;
	} else {  // Idle screen if not in the menu

		ssd1306_SetCursor(0, 0);  // Display trigger type
		snprintf(str, sizeof(str), "%dT", trigger_type);
		ssd1306_WriteString(str, Font_11x18, 1);

		ssd1306_SetCursor(64 - 11, 0);	// Display welding mode (AC, DC)
		snprintf(str, sizeof(str), mode_string[(welding_mode & 3) - 1]);
		ssd1306_WriteString(str, Font_11x18, 1);

		ssd1306_SetCursor(128 - (11 * 3), 0);
		uint8_t i = (welding_mode & Pulse) ? 2 : ((welding_mode & MMA) ? 3 : 4);
		snprintf(str, sizeof(str), mode_string[i]);
		ssd1306_WriteString(str, Font_11x18, 1);

		if (menu == 3 && state == idle) {
			if (main_menu(&enc_last, &encoder_counter, &last_millis)) menu = 0;
		} else if (enc_last != encoder_counter && encoder_counter % 2 == 0) {
			if (enc_last < encoder_counter)
				base_current += (encoder_counter - enc_last) / 2;
			else
				base_current -= (enc_last - encoder_counter) / 2;
			base_current = (base_current > max_current) ? max_current : (base_current < min_current) ? min_current : base_current;
			enc_last = encoder_counter;	 // Reset last encoder value
		}

		if (state == idle) {
			snprintf(str, sizeof(str), "%dA", (int)base_current);  // Display base current
		} else {
			snprintf(str, sizeof(str), "%dA", (int)adc_buffer[1]);	// Display ADC measured current
		}
		ssd1306_SetCursor(64 - (16 * strlen(str) / 2), 48 - (26 / 2));
		ssd1306_WriteString(str, Font_16x26, 1);
	}
#endif
	ssd1306_UpdateScreen();
}

uint8_t main_menu(int16_t *enc_last, int16_t *encoder_counter, uint32_t *last_millis) {
	static uint8_t set = false;	 // Button state
	static uint8_t menu_number = 1;
	char str[10];
	if (encoder_button && HAL_GPIO_ReadPin(ENC_BUTTON_GPIO_Port, ENC_BUTTON_Pin)) {
		set ^= 1;
		encoder_button = 0;
		enc_press_time = 0;
	} else if (enc_press_time > 20) {
		encoder_button = 0;
		enc_press_time = 0;
		set = 0;
		return 1;
	}
	if (*enc_last != *encoder_counter && *encoder_counter % 2 == 0 && !set) {  // Go through the menu
		menu_number += (*enc_last - *encoder_counter > 0) ? -1 : +1;
		*last_millis = HAL_GetTick();  // Reset timeout timer
		*enc_last = *encoder_counter;
	}
	switch (menu_number) {
		case 0:
			menu_number = 3;
			break;

		case 1:
			if (set) {
				if (update_val(enc_last, encoder_counter)) trigger_type ^= 6;
				ssd1306_SetCursor(0, 0);  // Display trigger type
				snprintf(str, sizeof(str), "%dT", trigger_type);

				if (HAL_GetTick() - *last_millis >= 500) {
					ssd1306_WriteString(str, Font_11x18, 0);
					if (HAL_GetTick() - *last_millis >= 1000) *last_millis = HAL_GetTick();
				} else {
					ssd1306_WriteString(str, Font_11x18, 1);
				}
			} else {
				ssd1306_FillRectangle(0, 16, 22, 18, 1);
			}
			break;

		case 2:
			if (set) {
				if (update_val(enc_last, encoder_counter)) welding_mode ^= (1 << 0 | 1 << 1);
				ssd1306_SetCursor(64 - 11, 0);	// Display welding mode (AC, DC)
				snprintf(str, sizeof(str), mode_string[(welding_mode & 3) - 1]);

				if (HAL_GetTick() - *last_millis >= 500) {
					ssd1306_WriteString(str, Font_11x18, 0);
					if (HAL_GetTick() - *last_millis >= 1000) *last_millis = HAL_GetTick();
				} else {
					ssd1306_WriteString(str, Font_11x18, 1);
				}
			} else {
				ssd1306_FillRectangle(64 - 11, 16, 64 - 11 + 22, 18, 1);
			}
			break;

		case 3:
			if (set) {
				int8_t i = (welding_mode >> 2) & 0b11;
				int8_t j = (update_val(enc_last, encoder_counter) / 2);
				if (j != 0) {
					i += j;
					if (i >= 3) i = 0;
					if (i < 0) i = 2;
					welding_mode = (welding_mode & ~(0b11 << 2)) | ((i & 0b11) << 2);
				}

				ssd1306_SetCursor(128 - (11 * 3), 0);
				uint8_t k = (welding_mode & Pulse) ? 2 : ((welding_mode & MMA) ? 3 : 4);
				snprintf(str, sizeof(str), mode_string[k]);

				if (HAL_GetTick() - *last_millis >= 500) {
					ssd1306_WriteString(str, Font_11x18, 0);
					if (HAL_GetTick() - *last_millis >= 1000) *last_millis = HAL_GetTick();
				} else {
					ssd1306_WriteString(str, Font_11x18, 1);
				}
			} else {
				ssd1306_FillRectangle(128, 16, 128 - 33, 18, 1);
			}
			break;

		default:
			menu_number = 1;
			break;
	}
	if (HAL_GetTick() - *last_millis > 10000) {	 // 10 second timeout
		menu_number = 0;						 // Reset menu number
		return 1;
	}
	return 0;
}

uint8_t sub_menu(int16_t *enc_last, int16_t *encoder_counter, uint32_t *last_millis) {
	static int menu_number = 1;	 // Menu number
	static uint8_t set = false;	 // Button state

	if (encoder_button && HAL_GPIO_ReadPin(ENC_BUTTON_GPIO_Port, ENC_BUTTON_Pin)) {
		set ^= 1;
		encoder_button = 0;
		enc_press_time = 0;
	} else if (enc_press_time > 20) {
		encoder_button = 0;
		enc_press_time = 0;
		set = 0;
		menu_number = 1;
		return 1;
	}

	if (*enc_last != *encoder_counter && *encoder_counter % 2 == 0 && !set) {  // Go through the menu
		uint8_t last_menu = menu_number;
		menu_number += (*enc_last - *encoder_counter > 0) ? -1 : +1;
		if (last_menu > menu_number) {	// Going backward through menu
			if (menu_number == 0) menu_number = 11;
			if (!(welding_mode & AC) && (menu_number == 10 || menu_number == 11)) menu_number = 9;
			if (!(welding_mode & Pulse) && (menu_number == 9 || menu_number == 8)) menu_number = 7;
		} else {
			if (!(welding_mode & Pulse) && (menu_number == 9 || menu_number == 8)) menu_number = 10;
			if (!(welding_mode & AC) && (menu_number == 10 || menu_number == 11)) menu_number = 1;
		}
		// switch (menu_number) {
		// 	case 0:
		// 		if (welding_mode & Pulse)
		// 			menu_number = 9;
		// 		else if (welding_mode & AC)
		// 			menu_number = 11;
		// 		else
		// 			menu_number = 7;
		// 		break;
		// 	case 8:	 // Pulse
		// 		if (!(welding_mode & Pulse)) menu_number += 2;
		// 	case 9:	 // Pulse
		// 		if (!(welding_mode & Pulse)) menu_number++;
		// 		break;
		// 	case 10:  // AC
		// 		if (!(welding_mode & AC)) menu_number += 2;
		// 	case 11:  // AC
		// 		if (!(welding_mode & AC)) menu_number = 1;
		// 	default:
		// 		menu_number = 0;
		// 		break;
		// }
		// if (menu_number <= 0) menu_number = sizeof(menu_string) / sizeof(menu_string[0]);
		*last_millis = HAL_GetTick();
		*enc_last = *encoder_counter;
	}

	switch (menu_number) {	// Matching menu number to function

		case 1:	 // Preflow setting
			if (set) { preflow_time += update_val(enc_last, encoder_counter) * 50; }
			ssd1306_write_as_val(preflow_time, 'S');
			break;

		case 2:	 // Start current setting
			if (set) { start_current += update_val(enc_last, encoder_counter) / 2; }
			ssd1306_write_as_val(start_current, 'A');
			break;

		case 3:	 // Upslope time setting
			if (set) { upslope_time += update_val(enc_last, encoder_counter) * 50; }
			ssd1306_write_as_val(upslope_time, 'S');
			break;

		case 4:	 // Base current setting
			if (set) { base_current += update_val(enc_last, encoder_counter) / 2; }
			ssd1306_write_as_val(base_current, 'A');
			break;

		case 5:	 // Downslope time setting
			if (set) { downslope_time += update_val(enc_last, encoder_counter) * 50; }
			ssd1306_write_as_val(downslope_time, 'S');
			break;

		case 6:	 // Crater current setting
			if (set) { crater_current += update_val(enc_last, encoder_counter) / 2; }
			ssd1306_write_as_val(crater_current, 'A');
			break;

		case 7:	 // Postflow time setting
			if (set) { postflow_time += update_val(enc_last, encoder_counter) * 50; }
			ssd1306_write_as_val(postflow_time, 'S');
			break;
		case 8:	 // Pulse Frequency setting
			if (set) pulse_frequency += (float)update_val(enc_last, encoder_counter) * 0.05;
			ssd1306_write_as_val((uint16_t)(pulse_frequency * 1000), 'h');
			break;
		case 9:	 // Pulse Duty setting
			if (set) pulse_duty += update_val(enc_last, encoder_counter) / 2;
			ssd1306_write_as_val(pulse_duty, '%');
			break;
		case 10:
			if (set) ac_frequency += update_val(enc_last, encoder_counter) / 2;
			ssd1306_write_as_val(ac_frequency, 'H');
			break;
		case 11:
			if (set) ac_duty += update_val(enc_last, encoder_counter) / 2;
			ssd1306_write_as_val(ac_duty, '%');
			break;

		default:
			menu_number = 1;
			break;
	}

	if (set) {
		blink_menu_name(menu_string[menu_number - 1], last_millis);
	} else {
		ssd1306_SetCursor(64 - (11 * strlen(menu_string[menu_number - 1]) / 2), 0);
		ssd1306_WriteString((char *)menu_string[menu_number - 1], Font_11x18, 1);
	}

	if (HAL_GetTick() - *last_millis > 10000) {	 // 10 second timeout
		menu_number = 1;						 // Reset menu number
		return 1;
	}
	return 0;
}

void blink_menu_name(const char *arr, uint32_t *last_millis) {
	uint8_t str_len = strlen(arr);
	ssd1306_SetCursor(64 - 11 * str_len / 2, 0);
	if (HAL_GetTick() - *last_millis >= 500) {
		ssd1306_WriteString((char *)arr, Font_11x18, 0);
		if (HAL_GetTick() - *last_millis >= 1000) *last_millis = HAL_GetTick();
	} else {
		ssd1306_WriteString((char *)arr, Font_11x18, 1);
	}
}

void ssd1306_write_as_val(const uint16_t val, const char ch) {
	char str[10];
	uint16_t c;
	uint16_t b;

	if (ch == 'S' || ch == 'h') {
		c = val / 1000;
		b = (val - c * 1000) / 100;
		if (ch == 'h')
			snprintf(str, sizeof(str), "%d.%dHz", c, b);
		else
			snprintf(str, sizeof(str), "%d.%d%c", c, b, ch);
	} else if (ch == 'A') {
		snprintf(str, sizeof(str), "%dA", val);
	} else if (ch == 'H') {
		snprintf(str, sizeof(str), "%dHz", val);
	} else {
		snprintf(str, sizeof(str), "%d%c", val, ch);
	}

	uint8_t str_len = strlen(str);
	ssd1306_SetCursor(64 - (16 * str_len / 2), 32);
	ssd1306_WriteString(str, Font_16x26, 1);
}

int16_t update_val(int16_t *enc_last, int16_t *encoder_counter) {
	int16_t ret = 0;
	if (*enc_last != *encoder_counter && *encoder_counter % 2 == 0) {
		ret = *encoder_counter - *enc_last;
		*enc_last = *encoder_counter;
	}
	return ret;
}

uint8_t strlen(char *str) {
	uint8_t str_len = 0;
	// Loop until we encounter the null character
	while (str[str_len] != '\0') { str_len++; }
	return str_len;
}

void state_machine() {	// Basic state machine for welding processes
	idle_menu();
	uint16_t set_curr;
	switch (state) {
		case idle:
			if (start_condition()) { state++; }
			HAL_GPIO_WritePin(GAS_GPIO_Port, GAS_Pin, GPIO_PIN_RESET);			  // Close gas at idle
			HAL_GPIO_WritePin(IGNITION_GPIO_Port, IGNITION_Pin, GPIO_PIN_RESET);  // Disable igniter
			HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);  // Do not shut down inverter to let the fan run
			set_current(0);														  // Set current to 0
			break;
		case preflow:
			if (stop_condition()) {
				NB_Delay(0, 1);	 // Reset non blocking delay
				state = idle;
				break;
			}
			HAL_GPIO_WritePin(GAS_GPIO_Port, GAS_Pin, GPIO_PIN_SET);
			if (welding_mode & AC) {
				set_pwm_frequency(ac_frequency);
				set_pwm_duty(ac_duty);
			} else {
				set_pwm_duty(0);  // Setting this to 0 causes the pwm pin to be constant value
			}
			if (NB_Delay(preflow_time, 0))	// open argon and after delay go to next state
				state++;
			break;
		case ignition:
			if (stop_condition()) {
				NB_Delay(0, 1);
				state = idle;
				break;
			}
			HAL_GPIO_WritePin(IGNITION_GPIO_Port, IGNITION_Pin, GPIO_PIN_SET);	// Enable igniter
			set_current(ignition_current);										// Set low ignition current
			if (NB_Delay(ignition_timeout, 0)) {
				if (adc_buffer[1] >= current_threshold) {
					state++;
				} else
					state = idle;
			}
			break;
		case upslope:
			if (stop_condition()) {
				NB_Delay(0, 1);
				LinearTransition(0, 0, 0, 1);
				state = crater;
				break;
			}
			set_curr = LinearTransition(start_current, base_current, upslope_time, 0);
			set_current(set_curr);
			if (set_curr == base_current) state++;
			break;
		case welding:
			if (stop_condition()) { state = downslope; }
			weld();
			break;
		case downslope:
			if (start_condition()) {
				LinearTransition(0, 0, 0, 1);
				state = welding;
				break;
			}
			// Begin to gradually lower the current
			set_curr = LinearTransition(base_current, crater_current, downslope_time, 0);
			set_current(set_curr);
			if (set_curr == crater_current) state++;
			break;
		case crater:
			if (start_condition()) {
				NB_Delay(0, 1);
				state = upslope;
				break;
			}
			if (NB_Delay(crater_time, 0)) state++;
			break;
		case postflow:
			if (start_condition()) {
				NB_Delay(0, 1);
				state = ignition;
				break;
			}
			set_current(0);	 // Extinguish the arc
			HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_SET);
			if (NB_Delay(postflow_time, 0)) state = idle;
			break;

		default:  // Failsafe
			state = idle;
			break;
	}
}

void weld() {
	static uint16_t set_curr = 0;
	static uint32_t prev_millis = 0;

	set_curr = base_current;

	if (welding_mode & Pulse) {
		float period = 1000.0 / pulse_frequency;
		if (HAL_GetTick() - prev_millis >= (uint32_t)period) {
			prev_millis = HAL_GetTick();
			set_curr = base_current;
		} else if (HAL_GetTick() - prev_millis >= (uint16_t)(((float)pulse_duty / 100.0) * period)) {
			set_curr = background_current;
		}
	}

	if (pedal_connected && trigger_type == _4T) { set_curr = set_curr * (float)adc_buffer[0] / 4095.0; }
	set_current(set_curr);
}

void set_current(uint8_t current) {
	if (current > max_current) current = max_current;

	if (current == 0) {
		HAL_TIM_PWM_Stop_IT(&htim10, TIM_CHANNEL_1);
	} else {
		HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1);
		uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim10);
		uint32_t pwm_compare = (current * pwm_period) / max_current;
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm_compare);
	}
}

void set_pwm_frequency(uint32_t freq) {
	uint32_t pwm_period = 1000000 / freq;
	__HAL_TIM_SET_AUTORELOAD(&htim4, pwm_period - 1);
}

void set_pwm_duty(uint32_t duty) {
	uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim4);
	uint32_t pwm_compare = (duty * pwm_period) / 100;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_compare - 1);
}

uint16_t LinearTransition(uint16_t a, uint16_t b, uint32_t t_total, uint8_t reset) {
	static uint32_t startTime = 0;	// Store the start time
	static uint8_t started = 0;		// State to track if the transition started
	uint16_t result = a;			// Default to the initial value

	if (reset) {
		started = 0;
		return result;
	}

	if (!started) {
		startTime = HAL_GetTick();	// Record the start time
		started = 1;
	}

	// Calculate elapsed time
	uint32_t elapsedTime = HAL_GetTick() - startTime;

	if (elapsedTime < t_total) {
		// Calculate interpolated value (works for both increase and decrease)
		result = a + (int16_t)(((float)(b - a) / (float)t_total) * (float)elapsedTime);
	} else {
		// Transition complete
		result = b;
		started = 0;  // Reset for the next call
	}

	return result;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	static uint8_t ttrig = 0;
	static uint8_t etrig = 0;
	if (htim->Instance == TIM11) {
		if (HAL_GPIO_ReadPin(TORCH_BUTTON_GPIO_Port, TORCH_BUTTON_Pin) == 0) {
			switch (ttrig) {
				case 0:
					ttrig = 1;
					break;
				case 1:
					torch_trigger = 1;
					ttrig = 2;
					break;
				default:

					break;
			}
		} else {
			ttrig = 0;
		}
		if (HAL_GPIO_ReadPin(ENC_BUTTON_GPIO_Port, ENC_BUTTON_Pin) == 0) {
			enc_press_time++;
			switch (etrig) {
				case 0:
					etrig = 1;
					break;
				case 1:
					encoder_button = 1;
					etrig = 2;
					break;
				default:

					break;
			}
		} else {
			etrig = 0;
		}
	}
}

uint8_t NB_Delay(uint32_t delay_time, uint8_t reset) {
	static uint32_t startTime = 0;	// Store the start time
	static uint8_t started = 0;		// State to track if the transition started

	if (reset) {
		started = 0;
		return 1;
	}

	if (!started) {
		startTime = HAL_GetTick();	// Record the start time
		started = 1;
	}

	if (HAL_GetTick() - startTime < delay_time) { return 0; }
	started = 0;
	return 1;
}

uint8_t stop_condition() {	// If any of these are true - go into another stage
	if ((trigger_type == _2T && HAL_GPIO_ReadPin(TORCH_BUTTON_GPIO_Port, TORCH_BUTTON_Pin) == 1) || (trigger_type == _4T && torch_trigger == 1)) {
		torch_trigger = 0;
		return 1;
	}
	if (adc_buffer[0] <= 50 && pedal_connected == 1 && trigger_type == _4T) return 1;
	return 0;
}

uint8_t start_condition() {
	if ((trigger_type == _2T && HAL_GPIO_ReadPin(TORCH_BUTTON_GPIO_Port, TORCH_BUTTON_Pin) == 0) || (trigger_type == _4T && torch_trigger == 1)) {
		torch_trigger = 0;
		return 1;
	}
	if (adc_buffer[0] >= 4095 - 500 && pedal_connected == 1 && trigger_type == _4T) return 1;  // If pedal almost fully pressed start welding
	return 0;
}

// Optional encoder overflow handling
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim == &htimX) {
//         // Handle overflow
//     }
// }
