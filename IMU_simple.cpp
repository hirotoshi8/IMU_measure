/*
 * entrypoint.cpp
 * setup関数、loop関数の実装
 */
// HelloWorldCpp.hpp
//   ArduinoをC/C++で実装するに当たって必須のヘッダファイル
//
#ifndef STD_ARDUINO_HPP
#define STD_ARDUINO_HPP

#include "Arduino.h"    // ここでArduino.hをインクルードする

// Arduinoで必須な関数をここで宣言しておく。
//void setup();
//void loop();
#endif

#include "Wire.h"
#include "IMU_simple.hpp"

/* Definition of IMU Address */
#define MPU6050_WHO_AM_I (0x75)
#define MPU_ADDRESS (0x68)
#define MPU6050_PWR_MGMT_1 (0x6B)

#define MPU_ACC_CONFIG (0x1C)
#define MPU_GYRO_CONFIG (0x1B)
#define MPU_LPF_CONFIG (0x1A)

#define MPU_ACCEL_XOUT (0x3B)

/* Local variables */
static float acc_x, acc_y, acc_z;

/***** Helper Functoion *****/
/* Serial Print function for debug */
void imu_debug_print(void){
    // Using Serial Plotter on the Arduino IDE
    Serial.print("acc_x"); Serial.print(",");
    Serial.print(acc_x); Serial.print(",");
    Serial.print("acc_y"); Serial.print(",");
    Serial.print(acc_y); Serial.print(",");
    Serial.print("acc_z"); Serial.print(",");
    Serial.print(acc_z); Serial.print("");
    Serial.println("");
}


/* Print Error Message */
void imu_error_msg(int error_no){
    /* Error message */
    if(0 == error_no){
        // Serial.println("Success");
    }else if(1 == error_no){
        Serial.println("Receiver buffer over flow");
    }else if(2 == error_no){
        Serial.println("Error: NAC from Slave address message");
    }else if(3 == error_no){
        Serial.println("Error: NAC from Data");
    }else{
        Serial.println("Error: Dont know");
    }
    return;
}


/* API for the IMU */
/* IMU configulation on initialization */
void imu_config(void){

    // Connte to I2C bus as Master
    Wire.begin();
    
    // 動作モードの読み出し
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();


    // Initial Set: ACC sensor
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(MPU_ACC_CONFIG);
    Wire.write(AccScaleSetting);
    int ret_acc = Wire.endTransmission();
    imu_error_msg(ret_acc);

    // Initial Set: Gyro sensor
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(MPU_GYRO_CONFIG);
    Wire.write(GyroScaleSetting);
    int ret_gyro = Wire.endTransmission();
    imu_error_msg(ret_gyro);

    // Initial Set: LPF
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(MPU_LPF_CONFIG);
    Wire.write(LpfConfig);
    int ret_lpf = Wire.endTransmission();
    imu_error_msg(ret_lpf);

    return;
}

/* IMU Receiver */
void imu_receive(void){
    const int16_t dataLength = 14;
    int16_t axRaw, ayRaw, azRaw;
    int16_t gxRaw, gyRaw, gzRaw;
    int16_t Temperture;

    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(MPU_ACCEL_XOUT);
    int ret = Wire.endTransmission(false);
    // true: default. Realease the connection
    // false: Not Realease the connection
    
    // Check the message error
    imu_error_msg(ret);

    // Requeset Receive message from IMU with I2C
    Wire.requestFrom(MPU_ADDRESS, dataLength, true);
    
    while(Wire.available() < dataLength);
    axRaw = (((int16_t)Wire.read() & 0xff) << 8) | (Wire.read() & 0xff);
    ayRaw = (((int16_t)Wire.read() & 0xff) << 8) | (Wire.read() & 0xff);
    azRaw = (((int16_t)Wire.read() & 0xff) << 8) | (Wire.read() & 0xff);
        
    Temperture = (((int16_t)Wire.read() & 0xff) << 8) | (Wire.read() & 0xff);
    gxRaw = (((int16_t)Wire.read() & 0xff) << 8) | (Wire.read() & 0xff);
    gyRaw = (((int16_t)Wire.read() & 0xff) << 8) | (Wire.read() & 0xff);
    gzRaw = (((int16_t)Wire.read() & 0xff) << 8) | (Wire.read() & 0xff);

    // Convert to the physical value
    acc_x = (float)axRaw * AccScaleFactor;
    acc_y = (float)ayRaw * AccScaleFactor;
    acc_z = (float)azRaw * AccScaleFactor;

    // Debug code
#if DEBUG
    imu_debug_print();
#endif
    return;
}

