#include "main.h"

void welder();
void idle_menu();
void set_pwm_frequency(uint32_t freq);
void set_pwm_duty(uint32_t duty);
void state_machine();
void set_current(uint8_t current);
void weld();
void default_settings();
uint16_t LinearTransition(uint16_t a, uint16_t b, uint32_t t_total, uint8_t reset);
uint8_t NB_Delay(uint32_t delay_time, uint8_t reset);
uint8_t start_condition();
uint8_t stop_condition();
uint8_t main_menu(int16_t* enc_last, int16_t* encoder_counter, uint32_t* last_millis);
uint8_t sub_menu(int16_t* enc_last, int16_t* encoder_counter, uint32_t* last_millis);
int16_t update_val(int16_t* enc_last, int16_t* encoder_counter);
// void ssd1306_write_time_S(const uint16_t val);
void ssd1306_write_as_val(const uint16_t val, const char ch);
void blink_menu_name(const char* arr, uint32_t* last_millis);
// void ssd1306_write_amps(const uint16_t val);
uint8_t strlen(char* str);