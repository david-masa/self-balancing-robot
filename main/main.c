#include <stdio.h>
#include "freertos/FreeRTOS.h"               //FreeRTOSの基本機能         
#include "freertos/task.h"                   //タスクを操作するため。
#include "driver/gpio.h"                     //GPIOピンの入出力操作ができるようにするため。
#include "driver/ledc.h"                     //PWMを使えるようにするため。LED用だがモーターなどにも応用可能で、MCPMWよりも簡素な機能。
#include "driver/i2c_master.h"               //I2C通信に必要で、ESP-IDFでは新しいヘッダー。
#include <math.h>                            //数学関数を使えるようにするため。atan2f(),fabsf()など。
#include <stdbool.h>                         //bool,true,falseを使えるようにするため。
#include "esp_timer.h"                       //int64_t time = esp_timer_get_time();で起動からの経過時間u秒が取得可能。

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
#define LEDC_RES_BITS   LEDC_TIMER_8_BIT        //dutyは0~255の範囲

#define LEDC_CH_A       LEDC_CHANNEL_0
#define LEDC_CH_B       LEDC_CHANNEL_1

#define I2C_SCL_PIN         GPIO_NUM_8      //blue
#define I2C_SDA_PIN         GPIO_NUM_9      //green

#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         100000      // 標準モード 100kHz

#define MPU6050_ADDR        0x68        // MPU6050のI2Cアドレス(AD0=LOWの場合)
#define MPU6050_WHO_AM_I    0x75        // WHO_AM_Iレジスタのアドレス
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_ACCEL_XOUT_H  0x3B

#define ACCEL_SCALE   16384.0f    // ±2g設定のとき 16384 LSB/g
#define GYRO_SCALE    131.0f      // ±250dps設定のとき 131 LSB/(deg/s)
#define RAD2DEG       57.29578f

#define LOOP_PERIOD_MS   10       // 制御周期 10ms = 100Hz
#define ALPHA            0.98f    // 相補フィルタ: ジャイロを信頼する割合
#define TARGET_ANGLE     -0.6f     // 直立時の角度(deg)。
#define IMU_SIGN         1.0f     // 前に傾いたとき角度がプラスにならなければ -1.0f
#define MOTOR_DIR        1        // 前に傾いたとき車輪が逆回転なら -1

#define KP               70.0f    // Pゲイン:どれくらい傾いているか？ 1度の傾きにつきduty 20
#define KI               0.0f     // Iゲイン:どれくらい傾きが残っているか？ 最初は0
#define KD               1.5f     // Dゲイン:どれくらいの勢いで倒れているか？ 角速度(deg/s)にかける
#define INTEGRAL_MAX     50.0f    // 積分値の上限(アンチワインドアップ)

#define MOTOR_MIN_DUTY   60       // モーターが回り始める最低duty
#define MOTOR_MAX_DUTY   255      // VM=3.7Vのとき約3.0V相当の上限

#define FALL_ANGLE       60.0f    // この角度を超えたら倒れたとみなして停止
#define ARM_ANGLE        3.0f     // この角度以内に戻ったら制御を開始

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

static i2c_master_dev_handle_t mpu_dev;    // MPU6050との通信ハンドル

typedef struct {
    float ax, ay, az;     // 加速度 (g)
    float gx, gy, gz;     // 角速度 (deg/s)
} imu_data_t;

// ---------- MPU6050 通信 ----------

// レジスタに1バイト書き込む
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(mpu_dev, buf, 2, 100);
}

// レジスタから連続で読み出す
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(mpu_dev, &reg, 1, data, len, 100);
}

// I2Cバスの初期化とMPU6050の設定
static void mpu6050_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port           = I2C_PORT,
        .sda_io_num         = I2C_SDA_PIN,
        .scl_io_num         = I2C_SCL_PIN,
        .clk_source         = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt  = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&bus_config, &bus_handle);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    i2c_master_bus_add_device(bus_handle, &dev_config, &mpu_dev);

    uint8_t who = 0;
    mpu6050_read_regs(MPU6050_WHO_AM_I, &who, 1);
    printf("WHO_AM_I = 0x%02X (期待値: 0x68)\n", who);

    mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00);     // スリープ解除
    vTaskDelay(100 / portTICK_PERIOD_MS);
    mpu6050_write_reg(MPU6050_CONFIG, 0x03);         // ローパスフィルタ 約44Hz
    mpu6050_write_reg(MPU6050_GYRO_CONFIG, 0x00);    // ジャイロ ±250deg/s
    mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x00);   // 加速度 ±2g
}

// 加速度・角速度を1回読み出して物理量に変換する
static bool mpu6050_read(imu_data_t *d)
{
    uint8_t raw[14];
    if (mpu6050_read_regs(MPU6050_ACCEL_XOUT_H, raw, 14) != ESP_OK) {
        return false;
    }

    // 上位バイトと下位バイトを合体して符号付き16bitにする
    int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
    // raw[6], raw[7] は温度なので使わない
    int16_t gx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    d->ax = ax / ACCEL_SCALE;
    d->ay = ay / ACCEL_SCALE;
    d->az = az / ACCEL_SCALE;
    d->gx = gx / GYRO_SCALE;
    d->gy = gy / GYRO_SCALE;
    d->gz = gz / GYRO_SCALE;
    return true;
}

// ---------- 角度の計算 ----------

// 加速度から求めた傾き角(acc_angle)と、傾き方向の角速度(rate)を取り出す。
static void imu_to_angle(const imu_data_t *d, float *acc_angle, float *rate)
{
    *acc_angle = IMU_SIGN * atan2f(-d->ax, d->az) * RAD2DEG;
    *rate      = IMU_SIGN * d->gy;
}

// ---------- モーター駆動 ----------

static void motors_stop(void)
{
    set_motor_right(0, 0);
    set_motor_left(0, 0);
}

// PID出力uを、モーターの向きとdutyに変換して駆動する
static void drive_motors(float u)
{
    float mag = fabsf(u);
    if (mag < 1.0f) {                       // 出力がほぼ0なら止める
        motors_stop();
        return;
    }

    int direction = (u > 0) ? MOTOR_DIR : -MOTOR_DIR;

    float duty_f = mag + MOTOR_MIN_DUTY;    // 回り始める分を上乗せする
    if (duty_f > MOTOR_MAX_DUTY) {
        duty_f = MOTOR_MAX_DUTY;
    }
    uint8_t duty = (uint8_t)duty_f;

    set_motor_right(direction, duty);
    set_motor_left(-direction, duty);       // 左モーターは向きが逆に付いているので反転
}

void app_main(void)
{
    motors_gpio_init();
    motors_pwm_init();
    motors_stop();
    mpu6050_init();

    imu_data_t imu;
    float acc_angle, rate;

    // ===== ジャイロのゼロ点補正（起動直後、機体を動かさず静止させておく） =====
    printf("ジャイロ補正中。機体を動かさないでください...\n");
    float rate_bias = 0.0f;
    const int CALIB_COUNT = 200;
    int ok_count = 0;
    for (int i = 0; i < CALIB_COUNT; i++) {
        if (mpu6050_read(&imu)) {
            imu_to_angle(&imu, &acc_angle, &rate);
            rate_bias += rate;
            ok_count++;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (ok_count > 0) {
        rate_bias /= ok_count;
    }
    printf("補正完了 rate_bias = %.3f deg/s\n", rate_bias);

    // ===== 初期角度は加速度から求める =====
    mpu6050_read(&imu);
    imu_to_angle(&imu, &acc_angle, &rate);
    float angle = acc_angle;

    float integral = 0.0f;
    bool  armed = false;
    int   print_count = 0;

    TickType_t last_wake = xTaskGetTickCount();
    int64_t prev_us = esp_timer_get_time();

    while (1) {
        // --- 経過時間 dt (秒) を実測する ---
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - prev_us) / 1000000.0f;
        prev_us = now_us;

        // --- センサー読み出し ---
        if (!mpu6050_read(&imu)) {          // 読み出し失敗時は安全のため停止
            motors_stop();
            vTaskDelayUntil(&last_wake, LOOP_PERIOD_MS / portTICK_PERIOD_MS);
            continue;
        }
        imu_to_angle(&imu, &acc_angle, &rate);
        rate -= rate_bias;

        // --- 相補フィルタ: ジャイロの積分と加速度角を混ぜる ---
        angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * acc_angle;

        // --- 偏差 ---
        float error = angle - TARGET_ANGLE;

        if (!armed) {
            // 待機中: 直立に近づくまでモーターは止めたまま
            motors_stop();
            integral = 0.0f;
            if (fabsf(error) < ARM_ANGLE) {
                armed = true;
            }
        } else if (fabsf(error) > FALL_ANGLE) {
            // 倒れた: 停止して待機状態に戻る
            motors_stop();
            armed = false;
        } else {
            // ===== PID制御 =====
            integral += error * dt;
            if (integral >  INTEGRAL_MAX) integral =  INTEGRAL_MAX;
            if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

            float u = KP * error + KI * integral + KD * rate;
            drive_motors(u);
        }

        // --- デバッグ表示（毎回出すと遅くなるので20回に1回） ---
        print_count++;
        if (print_count >= 20) {
            print_count = 0;
            printf("angle=%.2f  acc=%.2f  rate=%.2f  armed=%d\n",
                   angle, acc_angle, rate, armed);
        }

        vTaskDelayUntil(&last_wake, LOOP_PERIOD_MS / portTICK_PERIOD_MS);
    }
}