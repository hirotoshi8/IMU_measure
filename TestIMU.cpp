// TestIMU.hpp
//   ArduinoをC/C++で実装するに当たって必須のヘッダファイル
//
#ifndef STD_ARDUINO_HPP
#define STD_ARDUINO_HPP

#include "Arduino.h"    // ここでArduino.hをインクルードする

// Arduinoで必須な関数をここで宣言しておく。
void setup();
void loop();
#endif

/* API for the IMU observation */
#include "IMU.hpp"
//#include "IMU_simple.hpp"
#include "self_timer.hpp"

/* Main Process */
static IMU_Measure arduino_imu;
static SelfTimer timer_5ms;

/* SelfTimer settings */
const int delta_t = 5; // 5[ms]

/* Initialization */
void setup()
{
    // Serial Port Setting
    Serial.begin(115200);

    // IMU sensor configulation
    //imu_config();
    arduino_imu.config();
    arduino_imu.caliblation();
    
    // Initialize timer
    timer_5ms.create(delta_t);
    timer_5ms.start();

    return;
}

/* Main Process */
void loop()
{
    // Receive the Acc and Gyro data from IMU
    //imu_receive();
    arduino_imu.measure();

    /* 定周期ハンドラ */
    if(!timer_5ms.is_over()) return;
    /* Reset */
    timer_5ms.reset();
    /* Execute Application */
    arduino_imu.calculateAngle(((float)delta_t)/1000);
    /* Debug */
    arduino_imu.debug_print();

    return;
}
