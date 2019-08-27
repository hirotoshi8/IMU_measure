#ifndef CALCULATOR_HPP
#define CALCULATOR_HPP

#ifndef STD_ARDUINO_HPP
#define STD_ARDUINO_HPP
#include "Arduino.h"    // ここでArduino.hをインクルードする
#endif

class calculator
{
public:
    calculator():mean(0),var(0),count(0),squre_mean(0){};
    ~calculator(){};

    /* Calculaton with Online */
    void calculateOnlineAverage(float measurement){
        mean = (mean*count + measurement) / (count+1);
        // Debug
        // Serial.print("measurement");Serial.print(",");
        // Serial.print(measurement); Serial.print(",");
        // Serial.print("count"); Serial.print(",");
        // Serial.print(count); Serial.print(",");
        // Serial.print("mean"); Serial.println(",");
        // Serial.print(mean); Serial.println(",");
    };
    void calculateOnlineVar(float measurement){
        float squre_measurement = measurement * measurement;
        squre_mean = (squre_mean*count + squre_measurement) / (float)(count+1);
        var = squre_mean - (mean*mean);
        // Debug
        // Serial.print("measurement");Serial.print(",");
        // Serial.print(measurement,4); Serial.print(",");
        // Serial.print("squre_mean"); Serial.print(",");
        // Serial.print(squre_mean,4); Serial.print(",");
        // Serial.print("mean"); Serial.print(",");
        // Serial.print(mean,4); Serial.print(",");
        // Serial.print("count"); Serial.print(",");
        // Serial.print(count); Serial.print(",");
        // Serial.print("var"); Serial.print(",");
        // Serial.print(var,6); Serial.println(",");
    };
    void addCount(void){
        count++;
    };

    /* Calculaton with Offline */
    void calculateAverage(float* measurement, int length){
        float sum;
        for(int index = 0; index < length; index++){
            sum += measurement[index];
        }
        mean = sum / length;
    };
    void calculateVar(float* measurement, int length){
    };

    float get_result_mean(void){
        return this->mean;
    };

    float get_result_var(void){
        return this->var;
    };

private:
    float mean;
    float var;
    float squre_mean;
    int count;
};

#endif