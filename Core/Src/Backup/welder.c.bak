
#include "welder.h"
// #include "pcf8574.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stdio.h"
#include "main.h"
// #include "led_control.h"

// #define DEBUG

// DC welding params given in A
uint8_t base_current, background_current, crater_current, start_current;

// AC welding params given in Hz and Duty % respectfully
uint8_t ac_frequency, ac_duty;  // Goes from 0 = H to 100 = 99%; 0 = DC; Duty behaves as duty

// Pulse settings given in Hz and Duty % respectfully
float pulse_frequency;
uint8_t pulse_duty;

// Slope settings given in mS
uint16_t upslope_time, downslope_time;

// Gas settings given in mS
uint16_t preflow_time, postflow_time;

const char *menu_string[2] = {"Preflow", "Upslope"};
const char *mode_string[3] = {"DC", "AC", "Pul"};
const uint16_t current_threshold = 300; // Threshold at which we can tell that the arc is established 
const uint16_t ignition_timeout = 500;  // Time after which the arc should be going
const uint8_t max_current = 200;        // Amps
// const uint16_t debounce_time = 2;      // mS
const uint16_t ignition_current = 5;    // Amps
const uint16_t crater_time = 100;       // mS

uint8_t torch_trigger = 0;
uint8_t encoder_button = 0;
uint8_t pedal_connected = 0;
uint32_t adc_buffer[2];         // ADC CH0 - pedal, ADC CH1 - current sense
uint32_t start_tick = 0;
uint32_t deb_current_millis = 0;
uint32_t deb_prev_millis = 0;



enum State{
    idle,
    preflow,
    ignition,
    upslope,
    welding,
    downslope,
    crater,
    postflow
};

enum TriggerType
{
    _2T = 2,
    _4T = 4
};

enum WeldingMode
{
    DC = 1 << 0,
    AC = 1 << 1,
    Pulse = 1 << 2
};

enum TriggerType trigger_type = _4T; // 2T as default cus why not
enum WeldingMode welding_mode = 0;  // DC as default
volatile enum State state = idle;

// PCF8574_HandleTypeDef pcf8574_1;
// PCF8574_HandleTypeDef pcf8574_2;


void welder()
{
    // Some default settings
    preflow_time = 1200;
    postflow_time = 2000;
    upslope_time = 3000;
    downslope_time = 3000;
    start_current = 5;
    base_current = 200;
    background_current = 120;
    crater_current = 5;
    pedal_connected = 1;

    // welding_mode |= Pulse;
    welding_mode |= AC;
    // welding_mode |= DC;
    pulse_duty = 20;
    pulse_frequency = 3;

    ac_frequency = 100;
    ac_duty = 70;

    // Init peripethials
    HAL_GPIO_WritePin(GAS_GPIO_Port,GAS_Pin, GPIO_PIN_SET);
    // leds_off();
    
    // if(1){
    //     for(uint16_t i = 0; i<4096; i++){
    //         write_led(i);
    //         HAL_Delay(500);
    //     }
    // }

    // Initialize the PCF8574 with the desired address (A0 = 0, A1 = 0, A2 = 0)
    // PCF8574_Init(&pcf8574_1, &hi2c1, 0, 0, 0);
    // PCF8574_Init(&pcf8574_2, &hi2c1, 0, 0, 1);

    // PCF8574_WritePort(&pcf8574_1, 0x00);
    // PCF8574_WritePort(&pcf8574_2, 0x00);

    ssd1306_Init();
    ssd1306_Fill(0);
    ssd1306_UpdateScreen();

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Encoder timer

    HAL_ADC_Start_DMA(&hadc1, adc_buffer, 2);

    set_pwm_frequency(ac_frequency);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    // HAL_TIM_PWM_Start_IT(pwm_tim, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim11);

    while (1) // main program loop
    {
        state_machine();
    }
}


void idle_menu(){
    int16_t encoder_counter = __HAL_TIM_GET_COUNTER(&htim2);
    static int16_t enc_last = 0;
    static uint8_t menu = 0;
    static uint32_t last_millis = 0;
    char str[10];

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
    ssd1306_Fill(0);
        if(encoder_button && !menu){
            menu = 1;
            last_millis = HAL_GetTick();
            encoder_button = 0;
        }

        if(menu){
            static int i = 1;
            uint8_t a = 0;
            static uint8_t set = 0;
            if(encoder_button){
                    set ^= 1;
                    encoder_button = 0;
                }
                
            if(enc_last != encoder_counter && encoder_counter % 2 == 0 && !set){
                if(enc_last < encoder_counter) i++;
                else i--;
                last_millis = HAL_GetTick();
                enc_last = encoder_counter;
            }

            switch (i)
            {
            case 1:
                
                uint16_t c = preflow_time/1000;
                uint16_t b = (preflow_time - c*1000)/100;
                snprintf(str,sizeof(str),"%d.%dS",c, b);
                
                for(int j = 0; j<10; j++) if(str[j] != 0) a++;
                ssd1306_SetCursor(64-(16*a/2), 32);
                ssd1306_WriteString(str, Font_16x26, 1);

                if(enc_last != encoder_counter && encoder_counter % 2 == 0 && set){
                    if(enc_last < encoder_counter) preflow_time += (encoder_counter - enc_last)*50;
                    else preflow_time -= (enc_last - encoder_counter)*50;
                    enc_last = encoder_counter;
                }        
                if(HAL_GetTick() - last_millis >= 500 && set){
                    // ssd1306_SetCursor(0, 32);
                    // ssd1306_WriteString(str, Font_16x26, 1);
                    ssd1306_SetCursor(64-11*3.5,0);
                    ssd1306_WriteString("Preflow", Font_11x18,0);
                    if(HAL_GetTick() - last_millis >= 1000) last_millis = HAL_GetTick();
                }else{
                    ssd1306_SetCursor(64-11*3.5,0);
                    ssd1306_WriteString("Preflow", Font_11x18,1);
                }
                break;
            
                case 2:
                ssd1306_SetCursor(64-11*3.5,0);
                ssd1306_WriteString("Upslope", Font_11x18,1);
                snprintf(str,sizeof(str),"%dmS",upslope_time);
                for(int j = 0; j<10; j++) if(str[j] != 0) a++;
                ssd1306_SetCursor(64-(16*a/2), 32);
                ssd1306_WriteString(str, Font_16x26, 1);

                if(enc_last != encoder_counter && encoder_counter % 2 == 0 && set){
                    if(enc_last < encoder_counter) upslope_time += (encoder_counter - enc_last)*50;
                    else upslope_time -= (enc_last - encoder_counter)*50;
                    enc_last = encoder_counter;
                }        
                if(HAL_GetTick() - last_millis >= 500 && set){
                    // ssd1306_SetCursor(0, 32);
                    // ssd1306_WriteString(str, Font_16x26, 1);
                    ssd1306_SetCursor(64-11*3.5,0);
                    ssd1306_WriteString("Upslope", Font_11x18,0);
                    if(HAL_GetTick() - last_millis >= 1000) last_millis = HAL_GetTick();
                }else{
                    ssd1306_SetCursor(64-11*3.5,0);
                    ssd1306_WriteString("Upslope", Font_11x18,1);
                }
                break;

                break;
            
            default:
                i = 1;
                break;
            }
            if(HAL_GetTick() - last_millis > 10000) {
                menu = 0;
                i = 1;
            }
        }

        // if(clicked){
        //     static int8_t i = 0;
        //     static uint8_t set = 0;

        //     if(enc_last != encoder_counter && encoder_counter % 2 == 0 && !set){
        //         if(enc_last < encoder_counter) i++;
        //         else i--;
        //         enc_last = encoder_counter;
        //     }

        //     if(encoder_button){
        //         set ^= 1;
        //         encoder_button = 0;
        //     }

        //     switch (i)
        //     {
        //     case -1: i = 2; break; 
        //     case 0:
        //         // ssd1306_Line(0,20,18,20,1);
        //         ssd1306_FillRectangle(0,20,18,22,1);
        //         if(enc_last != encoder_counter && encoder_counter % 2 == 0 && set){
        //             trigger_type ^= _2T&_4T;
        //             enc_last = encoder_counter;
        //         }
        //         break;
        //     case 1:
        //         ssd1306_FillRectangle(54,20,54+18,22,1);
        //         if(enc_last != encoder_counter && encoder_counter % 2 == 0 && set){
        //             welding_mode ^= (1<<0 | 1<<1);
        //             enc_last = encoder_counter;
        //         }
        //         break;
        //     case 2:
        //         if(set){
        //             set = 0;
        //             i = 0;
        //             clicked = 0;
        //         }
        //     break;
        //     default:
        //         clicked = 0;
        //         i = 0;
        //         break;
        //     }
        // }
        
        else{
            ssd1306_SetCursor(0,0);
            snprintf(str,sizeof(str), "%dT", trigger_type);
            ssd1306_WriteString(str, Font_11x18, 1);

            ssd1306_SetCursor(64-11,0);
            snprintf(str,sizeof(str), mode_string[(welding_mode & 3) - 1]);
            ssd1306_WriteString(str, Font_11x18, 1);

            if(welding_mode & Pulse){
                ssd1306_SetCursor(128-(11*3),0);
                snprintf(str,sizeof(str), mode_string[2]);
                ssd1306_WriteString(str, Font_11x18, 1);
            }

            snprintf(str,sizeof(str), "%d", (int)adc_buffer[0]);
            int b = adc_buffer[0];
            uint8_t a = 0;
            while(b > 0) {a++; b /= 10;};
            ssd1306_SetCursor(64-(16*a/2), 48 - (26/2));
            ssd1306_WriteString(str, Font_16x26, 1);
        }
    #endif
    ssd1306_UpdateScreen();
}

void state_machine()    // Basic state machine for welding processes
{
    idle_menu();
    uint16_t set_curr;
    switch (state)
    {
    case idle:
        if(start_condition()){
            state++;
        }
        HAL_GPIO_WritePin(GAS_GPIO_Port, GAS_Pin, GPIO_PIN_RESET);  // Close gas at idle
        HAL_GPIO_WritePin(IGNITION_GPIO_Port, IGNITION_Pin, GPIO_PIN_RESET);    // Disable igniter
        set_current(0);                                             // Set current to 0
        break;
    case preflow:   
        if(stop_condition()){
            NB_Delay(0,1); // Reset non blocking delay
            state = idle;
            break;
        }
        HAL_GPIO_WritePin(GAS_GPIO_Port, GAS_Pin, GPIO_PIN_SET);
        if(welding_mode & AC){
            set_pwm_frequency(ac_frequency);
            set_pwm_duty(ac_duty);
        } else{
            set_pwm_duty(0);        // Setting this to 0 causes the pwm pin to be constant value
        }
        if(NB_Delay(preflow_time,0))  // open argon and after delay go to next state
            state++;
        break;
    case ignition:
        if(stop_condition()){
            NB_Delay(0,1);
            state = idle;
            break;
        }
        HAL_GPIO_WritePin(IGNITION_GPIO_Port, IGNITION_Pin, GPIO_PIN_SET);    // Enable igniter
        set_current(ignition_current);                // Set low ignition current
        if(NB_Delay(ignition_timeout,0)){
            if(adc_buffer[1] >= current_threshold){
                state++;
            } else
                state = idle;
        }
        break;
    case upslope:
        if(stop_condition()){
            NB_Delay(0,1);
            LinearTransition(0,0,0,1);
            state = crater;
            break;
        }
        set_curr = LinearTransition(start_current, base_current, upslope_time, 0);
        set_current(set_curr);
        if(set_curr == base_current)
            state++;
        break;
    case welding:
        if(stop_condition()){
            state = downslope;
        }
        weld();
        break;
    case downslope:
        if(start_condition()){
            LinearTransition(0,0,0,1);
            state = welding;
            break;
        }
        // Begin to gradually lower the current
        set_curr = LinearTransition(base_current, crater_current, downslope_time, 0);
        set_current(set_curr);
        if(set_curr == crater_current)
            state++;
        break;
    case crater:
        if(start_condition()){
            NB_Delay(0,1);
            state = upslope;
            break;
        }
        if(NB_Delay(crater_time,0))
            state ++;
        break;
    case postflow:
        if(start_condition()){
            NB_Delay(0,1);
            state = ignition;
            break;
        }
        set_current(0);             // Extinguish the arc
        if(NB_Delay(postflow_time,0))
            state = idle;
        break;
    

    default:    // Failsafe
        state = idle;
        break;
    }
}

void weld(){
    static uint16_t set_curr = 0;
    static uint32_t prev_millis = 0;

    set_curr = base_current;

    if(welding_mode & Pulse){
        float period = 1000.0/pulse_frequency;
        if(HAL_GetTick() - prev_millis >= (uint32_t)period){
            prev_millis = HAL_GetTick();
            set_curr = base_current;
        } else if(HAL_GetTick() - prev_millis >= (uint16_t)(((float)pulse_duty/100.0)*period)){
            set_curr = background_current;
        }
    }

    if(pedal_connected && trigger_type == _4T){
        set_curr = set_curr * (float)adc_buffer[0]/4095.0;
    }
    set_current(set_curr);
}

void set_current(uint8_t current){
    if(current > max_current) current = max_current;
    
    if (current == 0){
        HAL_TIM_PWM_Stop_IT(&htim10, TIM_CHANNEL_1);
    }else{
        HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1);
        uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim10);
        uint32_t pwm_compare = (current * pwm_period) / max_current;
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm_compare);
    }
}

void set_pwm_frequency(uint32_t freq)
{
    uint32_t pwm_period = 1000000 / freq;
    __HAL_TIM_SET_AUTORELOAD(&htim4, pwm_period - 1);
}

void set_pwm_duty(uint32_t duty)
{
    uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim4);
    uint32_t pwm_compare = (duty * pwm_period) / 100;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_compare - 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // deb_current_millis = HAL_GetTick();
    // if(deb_current_millis - deb_prev_millis < debounce_time) return;
    // deb_prev_millis = deb_current_millis;


    // switch (GPIO_Pin)
    // {
    // case GPIO_PIN_4: // Do stuff for encoder button
    //     break;
    // case GPIO_PIN_5: // Do stuff for torch button
    //     if (trigger_type == _4T)  // Button is held down
    //     {

    //         switch (state)
    //         {
    //         case preflow:
    //             state = idle;
    //             break;
    //         case ignition:
    //             state = idle;
    //             break;
    //         case upslope:
    //             state = crater;
    //             break;
    //         case welding:
    //             state = downslope;
    //             break;
    //         case downslope:
    //             state = upslope;
    //             break;
    //         case crater:
    //             state = ignition;
    //             break;
    //         case postflow:
    //             state = ignition;
    //             break;
    //         case idle:
    //             state = preflow;
    //             break;
    //         default:
    //             break;
    //         }
    //     }
    // default: // Don't do shit
    //     break;
    // }
}

uint16_t LinearTransition(uint16_t a, uint16_t b, uint32_t t_total, uint8_t reset) {
    static uint32_t startTime = 0;   // Store the start time
    static uint8_t started = 0;     // State to track if the transition started
    uint16_t result = a;               // Default to the initial value

    if(reset){
        started = 0;
        return result;
    }

    if (!started) {
        startTime = HAL_GetTick();  // Record the start time
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
        if(HAL_GPIO_ReadPin(TORCH_BUTTON_GPIO_Port, TORCH_BUTTON_Pin) == 0) {
            switch (ttrig)
            {
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
        }else{
            ttrig = 0;
        }
        if(HAL_GPIO_ReadPin(ENC_BUTTON_GPIO_Port, ENC_BUTTON_Pin) == 0) {
            switch (etrig)
            {
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
        }else {
            etrig = 0;
        }
    }
}

uint8_t NB_Delay(uint32_t delay_time, uint8_t reset){
    static uint32_t startTime = 0;   // Store the start time
    static uint8_t started = 0;     // State to track if the transition started

    if(reset){
        started = 0;
        return 1;
    }

    if (!started) {
        startTime = HAL_GetTick();  // Record the start time
        started = 1;
    }

    if(HAL_GetTick() - startTime < delay_time){
        return 0;
    }
    started = 0;
    return 1;
}

uint8_t stop_condition(){         // If any of these are true - go into another stage
    if((trigger_type == _2T && HAL_GPIO_ReadPin(TORCH_BUTTON_GPIO_Port,TORCH_BUTTON_Pin) == 1) || (trigger_type == _4T && torch_trigger == 1)){
        torch_trigger = 0;
        return 1;
    }
    if(adc_buffer[0] <= 50 && pedal_connected == 1 && trigger_type == _4T) return 1;
    return 0;
}

uint8_t start_condition(){
    if((trigger_type == _2T && HAL_GPIO_ReadPin(TORCH_BUTTON_GPIO_Port,TORCH_BUTTON_Pin) == 0) || (trigger_type == _4T && torch_trigger == 1)){
        torch_trigger = 0;
        return 1;
    }
    if(adc_buffer[0] >= 4095 - 500 && pedal_connected == 1 && trigger_type == _4T) return 1;       // If pedal almost fully pressed start welding
    return 0;
}



// Optional encoder overflow handling
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim == &htimX) {
//         // Handle overflow
//     }
// }
