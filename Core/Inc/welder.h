#include "main.h"
/// @brief main program startpoint
void welder();
/// @brief Display idle menu
void idle_menu();
/// @brief Set H-Bridge PWM frequency; Goes from 0 = H to 100 = 99%; 0 = DC;
/// @param freq PWM frequency
void set_pwm_frequency(uint32_t freq);
/// @brief Set H-Bridge PWM duty cycle
/// @param duty Duty cycle from 0 to 100%
void set_pwm_duty(uint32_t duty);
/// @brief State machine controlling welding process stages
void state_machine();
/// @brief Set inverter current
/// @param current Current to set; 0 to max_current
void set_current(uint8_t current);
/// @brief Main function controlling pulse and pedal inputs during welding
void weld();
/// @brief Linearly interpolate from a to b in time t_total
/// @param a Interpolation start
/// @param b Interpolation end
/// @param t_total Interpolation time
/// @param reset Reset the timer
/// @return Return Interpolated value based on a,b and time
uint16_t LinearTransition(uint16_t a, uint16_t b, uint32_t t_total, uint8_t reset);
/// @brief Non blocking delay
/// @param delay_time Amount of delay
/// @param reset Tiemr reset
/// @return Returns 1 when completed
uint8_t NB_Delay(uint32_t delay_time, uint8_t reset);
/// @brief Function checking if the start condition for welding is met based on trigger type and pedal input
/// @return Returns 1 if the start condition is detected
uint8_t start_condition();
/// @brief Function checking if the stop condition for welding is met based on trigger type and pedal input
/// @return Returns 1 if the stop condition is detected
uint8_t stop_condition();
/// @brief Menu allowing for setting most of the welding parameters
/// @param enc_last Last read encoder value
/// @param encoder_counter Latest encoder value
/// @param last_millis Last HAL_GetTick() time
/// @return Returns 1 when user wants to exit the menu or when timeout
uint8_t main_menu(int16_t* enc_last, int16_t* encoder_counter, uint32_t* last_millis);
/// @brief Manu allowing to set trigger type, AC/DC mode, MMA/TIG/PULSE welding
/// @param enc_last Last encoder value
/// @param encoder_counter Latest encoder value
/// @param last_millis Last HAL_GetTick() time
/// @return Returns 1 when user wants to exit the menu or when timeout
uint8_t sub_menu(int16_t* enc_last, int16_t* encoder_counter, uint32_t* last_millis);
/// @brief Updates value based on encoder knob
/// @param enc_last Last encoder value
/// @param encoder_counter Latest encoder value
/// @return Value by which the knob has been rotated * 2
int16_t update_val(int16_t* enc_last, int16_t* encoder_counter);
/// @brief Writes valeu at the middle of the display with unit
/// @param val Value to display
/// @param ch Unit: S - seconds with 1 decimal place; h - Hz with 1 decimal place; A - amps with no decimal; H - Hz with no decimal; Everything else - no decimal place with the character provided
void ssd1306_write_as_val(const uint16_t val, const char ch);
/// @brief Displays and blinks menu name at the top of display
/// @param arr String to display
/// @param last_millis Period of blinking
void blink_menu_name(const char* arr, uint32_t* last_millis);
/// @brief Calculate string length
/// @param str Input string
/// @return Returns string length
uint8_t strlen_(char* str);
/// @brief Saves settings with the provided offset. The offset has to be aligned to 4 bytes
/// @param offset Offset by which to write to SPI memory
void save_settings(uint16_t offset);
/// @brief Reads settings from the SPI memory. It does NOT heck for absurd values
/// @param offset Offset to start reading memory
void read_settings(uint16_t offset);
/// @brief Menu for preset welding configs
/// @param enc Value by which to increment menu
/// @param read_save Variable for submenu: 0 - list all configs; 1 - ask to read / write; 2 - confirm rea / write
/// @return Returns 1 when done saving / reading
uint8_t preset_menu(int8_t enc, uint8_t read_save);
/// @brief Constrains value between min and max
/// @param min Min value
/// @param max Max value
/// @param val Value to constrain
/// @return Returns constrained value
int32_t constrain(int32_t min, int32_t max, int32_t val);
/// @brief Constrains float value between min and max
/// @param min Min value
/// @param max Mxa value
/// @param val Value to constrain
/// @return Returns constrained value
float constrain_F(float min, float max, float val);