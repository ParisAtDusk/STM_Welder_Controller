#include "main.h"

//  Even numbered LEDs have their anodes connected to rows and cathodes to columns pins. The odd ones are antiparalled to them
#define PREFLOW_LED         1<<0
#define START_AMPS_LED      1<<1
#define UPSLOPE_LED         1<<2
#define WELDING_AMPS_LED    1<<3
#define PULSE_AMPS_LED      1<<4
#define PULSE_DUTY_LED      1<<5
#define PULSE_FREQ_LED      1<<6
#define AC_FREQ_LED         1<<7
#define AC_BALANCE_LED      1<<8
#define DOWNSLOPE_LED       1<<9
#define CRATER_LED          1<<10
#define POSTFLOW_LED        1<<11

#define ROW_OFFSET          6
#define COL_OFFSET          2

#define COL_0               PREFLOW_LED | START_AMPS_LED | PULSE_FREQ_LED | AC_FREQ_LED                                         // LEDS 0,1,6,7 are connecoted to column 0
#define COL_1               UPSLOPE_LED | WELDING_AMPS_LED | AC_BALANCE_LED | DOWNSLOPE_LED                                     // LEDS 2,3,8,9 are connecoted to column 1
#define COL_2               PULSE_AMPS_LED | PULSE_DUTY_LED | CRATER_LED | POSTFLOW_LED                                         // LEDS 4,5,10,11 are connecoted to column 2
#define ROW_0               PREFLOW_LED | START_AMPS_LED | UPSLOPE_LED | WELDING_AMPS_LED | PULSE_AMPS_LED | PULSE_DUTY_LED     // Those LEDs are connected to this row
#define ROW_1               PULSE_FREQ_LED | AC_FREQ_LED | AC_BALANCE_LED | DOWNSLOPE_LED | CRATER_LED | POSTFLOW_LED           // Those LEDs are connected to this row

void write_led(uint16_t led);
void leds_off(void);
void set_led(uint16_t led);
void reset_led(uint16_t led);
void update_leds(void);

