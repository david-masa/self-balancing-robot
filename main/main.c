#include <stdio.h>
#include "freertos/FreeRTOS.h"             
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define PWMA_PIN    GPIO_NUM_4             //orange
#define AIN2_PIN    GPIO_NUM_5             //blue
#define AIN1_PIN    GPIO_NUM_6             //gray
#define STBY_PIN    GPIO_NUM_7             //green

#define BIN1_PIN    GPIO_NUM_15            //white
#define BIN2_PIN    GPIO_NUM_16            //purple
#define PWMB_PIN    GPIO_NUM_17            //yellow

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ    5000
#define LEDC_RES_BITS   LEDC_TIMER_8_BIT

#define LEDC_CH_A       LEDC_CHANNEL_0
#define LEDC_CH_B       LEDC_CHANNEL_1

//モータードライバーのピンを初期化する関数
static void motors_gpio_init(void)
{
    //モータードライバーのピンを初期化するためにリセット
    gpio_reset_pin(AIN1_PIN);                         
    gpio_reset_pin(AIN2_PIN);                         
    gpio_reset_pin(BIN1_PIN);                       
    gpio_reset_pin(BIN2_PIN);
    gpio_reset_pin(STBY_PIN);

    //モータードライバーのピンを出力モードに設定
    gpio_set_direction(AIN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(AIN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BIN2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(STBY_PIN, GPIO_MODE_OUTPUT);

    //モータードライバーを有効化するためにSTBYピンをHIGHに設定
    gpio_set_level(STBY_PIN, 1);                      
}

//モータードライバーのPWMを初期化する関数
static void motors_pwm_init(void)
{
    ledc_timer_config_t timer_conf = {          //PWMのタイマーを設定
        .speed_mode       = LEDC_MODE,          //LEDCのスピードモードを設定
        .timer_num        = LEDC_TIMER,         //LEDCのタイマー番号を設定
        .duty_resolution  = LEDC_RES_BITS,      //LEDCのデューティ解像度を設定
        .freq_hz          = LEDC_FREQ_HZ,       //LEDCの周波数を設定
        .clk_cfg          = LEDC_AUTO_CLK,      //LEDCのクロックソースを自動選択に設定
    };
    ledc_timer_config(&timer_conf);             //PWMのタイマーを設定

    ledc_channel_config_t ch_a = {              //PWMのチャンネルAを設定
        .gpio_num       = PWMA_PIN,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CH_A,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0,
    };
    ledc_channel_config(&ch_a);  

    ledc_channel_config_t ch_b = {              //PWMのチャンネルBを設定
        .gpio_num       = PWMB_PIN,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CH_B,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0,
    };
    ledc_channel_config(&ch_b);
}

static void set_motor_right(int direction, uint8_t duty)    //モータードライバーの右側のモーターを制御する関数
{
    if(direction > 0){                          //正転の場合はAIN1をHIGHに設定
        gpio_set_level(AIN1_PIN, 1);
    } else {
        gpio_set_level(AIN1_PIN, 0);            //逆転または停止の場合はAIN1をLOWに設定
    }

    if(direction < 0){                          //逆転の場合はAIN2をHIGHに設定  
        gpio_set_level(AIN2_PIN, 1);
    } else {
        gpio_set_level(AIN2_PIN, 0);            //正転または停止の場合はAIN2をLOWに設定
    }

    uint8_t use_duty;                           //防御的プログラミングとして、directionの値が0の場合はdutyを0に設定する
    if(direction != 0){
        use_duty = duty;
    } else {
        use_duty = 0;
    }
    ledc_set_duty(LEDC_MODE, LEDC_CH_A, use_duty);  //PWMのデューティを設定
    ledc_update_duty(LEDC_MODE, LEDC_CH_A);         //PWMのデューティを更新
}

static void set_motor_left(int direction, uint8_t duty)     //モータードライバーの左側のモーターを制御する関数
{
    if(direction > 0){                          //正転の場合はBIN1をHIGHに設定
        gpio_set_level(BIN1_PIN, 1);
    } else {
        gpio_set_level(BIN1_PIN, 0);            //逆転または停止の場合はBIN1をLOWに設定
    }

    if(direction < 0){                          //逆転の場合はBIN2をHIGHに設定  
        gpio_set_level(BIN2_PIN, 1);
    } else {
        gpio_set_level(BIN2_PIN, 0);            //正転または停止の場合はBIN2をLOWに設定
    }

    uint8_t use_duty;                           //防御的プログラミングとして、directionの値が0の場合はdutyを0に設定する
    if(direction != 0){
        use_duty = duty;
    } else {
        use_duty = 0;
    }
    ledc_set_duty(LEDC_MODE, LEDC_CH_B, use_duty);  //PWMのデューティを設定
    ledc_update_duty(LEDC_MODE, LEDC_CH_B);         //PWMのデューティを更新
}