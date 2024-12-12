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