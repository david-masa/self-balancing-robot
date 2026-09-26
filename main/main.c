#include <stdio.h>
#include "freertos/FreeRTOS.h"             
#include "freertos/task.h"
#include "driver/gpio.h"

#define PWMA_PIN    GPIO_NUM_4           　  //orange
#define AIN2_PIN    GPIO_NUM_5          　   //blue
#define AIN1_PIN    GPIO_NUM_6           　  //gray

#define STBY_PIN    GPIO_NUM_7           　  //green

#define BIN1_PIN    GPIO_NUM_15         　   //white
#define BIN2_PIN    GPIO_NUM_16             //purple
#define PWMB_PIN    GPIO_NUM_17             //yellow

//モータードライバーのピンを初期化する関数
static void motor_gpio_init(void)
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

