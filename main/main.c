#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PWMA_PIN    GPIO_NUM_1              //orange
#define AIN2_PIN    GPIO_NUM_2              //blue
#define AIN1_PIN    GPIO_NUM_42             //gray

#define STBY_PIN    GPIO_NUM_41             //green

#define BIN1_PIN    GPIO_NUM_40             //white
#define BIN2_PIN    GPIO_NUM_39             //purple
#define PWMB_PIN    GPIO_NUM_38             //yellow