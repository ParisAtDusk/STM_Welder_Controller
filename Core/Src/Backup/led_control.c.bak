#include "led_control.h"

uint16_t leds = 0;
const uint8_t cols[3] = {COL_0, COL_1, COL_2};
const uint8_t rows[2] = {ROW_0, ROW_1};
const GPIO_TypeDef *col_port[3] = {LED_COL_0_GPIO_Port, LED_COL_1_GPIO_Port, LED_COL_2_GPIO_Port};
const uint16_t col_pin[3] = {LED_COL_0_Pin, LED_COL_1_Pin, LED_COL_2_Pin};
const GPIO_TypeDef *row_port[2] = {LED_ROW_0_GPIO_Port, LED_ROW_1_GPIO_Port};
const uint16_t row_pin[2] = {LED_ROW_0_Pin, LED_ROW_1_Pin};

void leds_off(void){
    leds = 0;
    HAL_GPIO_WritePin(LED_COL_0_GPIO_Port,LED_COL_0_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_COL_1_GPIO_Port,LED_COL_1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_COL_2_GPIO_Port,LED_COL_2_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_ROW_0_GPIO_Port,LED_ROW_0_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_ROW_1_GPIO_Port,LED_ROW_1_Pin,GPIO_PIN_RESET);
}

void write_led(uint16_t led){       // Used for debugging
    leds = led;
}

void set_led(uint16_t led){
    leds |= led;
}

void reset_led(uint16_t led){
    leds &= ~led;
}

void update_leds(){
    static uint8_t isEven = 0;
    static uint8_t led = 0;
    uint8_t row = led / 3;      // 3 leds columns
    uint8_t col = led % 3;  
    // For every pin check if the corresponding LED has to be turned on

    // if(isEven && (led == 0 || led == 6))

    // TODO: Make this work
    if((leds & cols[col]) && (leds & rows[row])) {
        HAL_GPIO_WritePin((GPIO_TypeDef *)col_port[col],col_pin[col],isEven^1);
        HAL_GPIO_WritePin((GPIO_TypeDef *)row_port[row],row_pin[row],isEven);
        HAL_GPIO_WritePin((GPIO_TypeDef *)row_port[1-row],row_pin[1-row],isEven^1);
    }else{
        HAL_GPIO_WritePin((GPIO_TypeDef *)col_port[col],col_pin[col],isEven);
        HAL_GPIO_WritePin((GPIO_TypeDef *)row_port[row],row_pin[row],isEven);
        HAL_GPIO_WritePin((GPIO_TypeDef *)row_port[1-row],row_pin[1-row],isEven);
    }

    led++;
    if(led == 12){
        led = 0;
        isEven ^= 1;
    }

    // if(COL_0 && (ROW_0 | ROW_1)) // Any of the 0167 leds are on


    // switch (led)
    // {
    // case 0:
    //     HAL_GPIO_WritePin(LED_COL_0_GPIO_Port,LED_COL_0_Pin,(COL_0 | leds) && (isEven^1));
    //     HAL_GPIO_WritePin(LED_ROW_0_GPIO_Port,LED_ROW_0_Pin,(ROW_0 | leds) && isEven);
    //     if(leds | ROW_1) HAL_GPIO_WritePin(LED_ROW_1_GPIO_Port,LED_ROW_1_Pin,isEven ^ 1);
    //     break;
    
    // default:
    //     break;
    // }

    // // GPIO_PIN_SET and GPIO_PIN_RESET are defined as '1' and '0' respectively
    // if(leds | COL_1) HAL_GPIO_WritePin(LED_COL_1_GPIO_Port,LED_COL_1_Pin,isEven);
    // if(leds | COL_2) HAL_GPIO_WritePin(LED_COL_2_GPIO_Port,LED_COL_2_Pin,isEven);
    
    
    // First turn on even leds then in the next cycle the odd ones 
    // isEven ^= 1;
}



