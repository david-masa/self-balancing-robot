#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define PWMA_PIN    GPIO_NUM_4             //orange
#define AIN2_PIN    GPIO_NUM_5             //blue
#define AIN1_PIN    GPIO_NUM_6             //gray

#define STBY_PIN    GPIO_NUM_7              //green

#define BIN1_PIN    GPIO_NUM_15             //white
#define BIN2_PIN    GPIO_NUM_16             //purple
#define PWMB_PIN    GPIO_NUM_17             //yellow