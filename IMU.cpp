/*
 * IMU.cpp
  */
#ifndef STD_ARDUINO_HPP
#define STD_ARDUINO_HPP
#include "Arduino.h"    // ここでArduino.hをインクルードする。Serial通信に必要
#endif

#include "Wire.h"
#include "IMU.hpp"
#include "self_timer.hpp"
#include "calculator.hpp"

#include <math.h>
#define RAD2DEG(x) ((x)*57.29578)

/* Definition IMU Address */
#define MPU6050_WHO_AM_I (0x75)
#define MPU_ADDRESS (0x68)
#define MPU6050_PWR_MGMT_1 (0x6B)

#define MPU_ACC_CONFIG (0x1C)
#define MPU_GYRO_CONFIG (0x1B)
#define MPU_LPF_CONFIG (0x1A)

#define MPU_ACCEL_XOUT (0x3B)

/* IMU dimension */
#define IMU_DIMENSION (3)

/* Declare the local funcfion */
static calculator acc_calc[IMU_DIMENSION];
static calculator gyro_calc[IMU_DIMENSION];

/* physical value calculated as the result */
static float acc_offset[IMU_DIMENSION];
static float gyro_offset[IMU_DIMENSION];
static float acc_vec[IMU_DIMENSION];
static float gyro_vec[IMU_DIMENSION];
static float angle_vec[IMU_DIMENSION];

static int dimension;

/* Test variables */
static float angle_vec_acc[IMU_DIMENSION];

/* Constructor */
IMU_Measure::IMU_Measure(){
    dimension = IMU_DIMENSION;

    for(int dim_i = 0; dim_i < dimension; dim_i++){
        acc_vec[dim_i] = 0;
        gyro_vec[dim_i] = 0;
        angle_vec[dim_i]= 0;

        acc_offset[dim_i] = 0;
        gyro_offset[dim_i] = 0;
    }
};
/* DeConstructor */
IMU_Measure::~IMU_Measure(){};

/* Member Function */
void IMU_Measure::config(void){

    // Connte to I2C bus as Master
    Wire.begin();
    
    // 動作モードの読み出し
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(IMU_POWER_MANAGEMENT_CONFIG);
    Wire.endTransmission();

    // Initial Set: ACC sensor
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(MPU_ACC_CONFIG);
    Wire.write(AccScaleSetting);
    int ret_acc = Wire.endTransmission();
    // Check the message error    
    print_error_msg(ret_acc);

    // Initial Set: Gyro sensor
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(MPU_GYRO_CONFIG);
    Wire.write(GyroScaleSetting);
    int ret_gyro = Wire.endTransmission();
    // Check the message error    
    print_error_msg(ret_gyro);

    // Initial Set: LPF
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(MPU_LPF_CONFIG);
    Wire.write(LpfConfig);
    int ret_lpf = Wire.endTransmission();
    // Check the message error
    print_error_msg(ret_lpf);

    return;
}

int IMU_Measure::caliblation(void){
    
    const int deltaT_ms = 1; // [msec]
    SelfTimer calib_timer;
    calib_timer.create(deltaT_ms);
    calib_timer.start();

    // caliblation duration
    int is_calib_duration_elapsed = 1000; // 1[s]
    // caliblation
    while(is_calib_duration_elapsed > 0){

        /* Free Loop as Back ground task */

        if(!calib_timer.is_over()) continue;
        calib_timer.reset();
        is_calib_duration_elapsed--;
        /* get the acc & gyro data from IMU */
        this->measure();
        this->stockData();

        // Debug
        // this->print_debug_gyro();
        // Serial.print("acc_offset"); Serial.print(",");
        // Serial.print(acc_calc[0].mean); Serial.print(",");
        // Serial.print("gyro_offset"); Serial.print(",");
        // Serial.print(gyro_calc[0].mean);Serial.print(",");
        // Serial.println("");
    }
    // update
    this->calculateOffset();
    // Debug
    // Serial.print(acc_offset[0]);
    // Serial.print(gyro_offset[0]);

    // Check the Error caliblation
    int ret = this->is_caliblated();
    return ret;
}


int IMU_Measure::is_caliblated(void){
    return (int)true;
}

void IMU_Measure::calculateOffset(void){
    for(int dim_i = 0; dim_i < dimension; dim_i++){
        acc_offset[dim_i] = (-1) * acc_calc[dim_i].get_result_mean();
        gyro_offset[dim_i]= (-1) * gyro_calc[dim_i].get_result_mean();
    }
}

void IMU_Measure::stockData(void){

    for(int dim_i = 0; dim_i < dimension; dim_i++){

        acc_calc[dim_i].calculateOnlineAverage(acc_vec[dim_i]);
        acc_calc[dim_i].calculateOnlineVar(acc_vec[dim_i]);
        gyro_calc[dim_i].calculateOnlineAverage(gyro_vec[dim_i]);
        gyro_calc[dim_i].calculateOnlineVar(gyro_vec[dim_i]);

        acc_calc[dim_i].addCount();
        gyro_calc[dim_i].addCount();
    }
    // Debug
    // this->print_debug_vec(gyro_vec);

}

void IMU_Measure::measure(void){
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
    print_error_msg(ret);

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
    acc_vec[0] = (float)axRaw * AccScaleFactor + acc_offset[0]; //x
    acc_vec[1] = (float)ayRaw * AccScaleFactor + acc_offset[1]; //y
    acc_vec[2] = (float)azRaw * AccScaleFactor /* + acc_offset[2]*/; //z

    gyro_vec[0] = (float)gxRaw * GyroScaleFactor + gyro_offset[0]; //x
    gyro_vec[1] = (float)gyRaw * GyroScaleFactor + gyro_offset[1]; //y
    gyro_vec[2] = (float)gzRaw * GyroScaleFactor + gyro_offset[2]; //z    

    return;
}

void IMU_Measure::calculateAngle(float deltaT_sec){

    /* Calculation Angle with Gyro */
   for(int dim_i = 0; dim_i < dimension; dim_i++){
       angle_vec[dim_i] += gyro_vec[dim_i] * deltaT_sec;
    }

    /* Calculation Angle with ACC */
    float squre_acc[IMU_DIMENSION] = {0};
    for(int dim_i = 0; dim_i < dimension; dim_i++){
        squre_acc[dim_i] = acc_vec[dim_i]*acc_vec[dim_i];
    }
    // calculate angle
    angle_vec_acc[0] = atan2(acc_vec[1], sqrt(squre_acc[2] + squre_acc[0]));
    angle_vec_acc[1] = atan2(acc_vec[0], sqrt(squre_acc[1] + squre_acc[2]));
    angle_vec_acc[2] = atan2(acc_vec[2], sqrt(squre_acc[0] + squre_acc[1]));
    // convert radian to degree
    angle_vec_acc[0] = RAD2DEG(angle_vec_acc[0]);
    angle_vec_acc[1] = RAD2DEG(angle_vec_acc[1]);
    angle_vec_acc[2] = RAD2DEG(angle_vec_acc[2]);
}

/* Helper function */
void IMU_Measure::debug_print(void){
    // Debug code
#if DEBUG_ACC
    // print_debug_acc();
    char label_name[] = "acc";
    print_debug_vec(acc_vec, label_name);
#endif
#if DEBUG_GYRO
    // print_debug_gyro();
    char label_name[] = "gyro";
    print_debug_vec(gyro_vec, label_name);
#endif
#if DEBUG_ANGLE
    // print_debug_angle();
    char label_name[] = "angle";
    print_debug_vec(angle_vec, label_name);
#endif
#if DEBUG_ANGLE_ACC
    // print_debug_angle();
    char label_name[] = "angle_acc";
    print_debug_vec(angle_vec_acc, label_name);
#endif
    return;
}

void IMU_Measure::print_debug_vec(float* vec, char* label){
    // Using Serial Plotter on the Arduino IDE
    // Serial.print(label); Serial.print(":");
    Serial.print("x"); Serial.print(",");
    Serial.print(vec[0]); Serial.print(",");
    Serial.print("y"); Serial.print(",");
    Serial.print(vec[1]); Serial.print(",");
    Serial.print("z"); Serial.print(",");
    Serial.print(vec[2]); Serial.print(",");
    Serial.println("");
}

#if 0
void IMU_Measure::print_debug_acc(void){
    // Using Serial Plotter on the Arduino IDE
    Serial.print("acc_x"); Serial.print(",");
    Serial.print(acc_vec[0]); Serial.print(",");
    Serial.print("acc_y"); Serial.print(",");
    Serial.print(acc_vec[1]); Serial.print(",");
    Serial.print("acc_z"); Serial.print(",");
    Serial.print(acc_vec[2]); Serial.print(",");
    Serial.println("");
}

void IMU_Measure::print_debug_gyro(void){
    // Using Serial Plotter on the Arduino IDE
    Serial.print("gyro_x"); Serial.print(",");
    Serial.print(gyro_vec[0]); Serial.print(",");
    Serial.print("gyro_y"); Serial.print(",");
    Serial.print(gyro_vec[1]); Serial.print(",");
    Serial.print("gyro_z"); Serial.print(",");
    Serial.print(gyro_vec[2]); Serial.print(",");
    Serial.println("");
}

void IMU_Measure::print_debug_angle(void){
    // Using Serial Plotter on the Arduino IDE
    Serial.print("angle_x"); Serial.print(",");
    Serial.print(angle_vec[0]); Serial.print(",");
    Serial.print("angle_y"); Serial.print(",");
    Serial.print(angle_vec[1]); Serial.print(",");
    Serial.print("angle_z"); Serial.print(",");
    Serial.print(angle_vec[2]); Serial.print(",");
    Serial.println("");
}

#endif

void IMU_Measure::print_error_msg(int error_no){
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
