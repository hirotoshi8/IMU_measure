#ifndef IMU_HPP
#define IMU_HPP

/* Debug Settings */
#define DEBUG_ACC (0)
#define DEBUG_GYRO (0)
#define DEBUG_ANGLE (0)
#define DEBUG_ANGLE_ACC (1)


/* API for the IMU */
class IMU_Measure
{
public:
    IMU_Measure();
    ~IMU_Measure();

    // Cofigulation on initialization
    void config(void);

    // Measure imu
    void measure(void);

    // Calculate Angle
    void calculateAngle(float deltaT_sec);

    // Caliblation
    int caliblation(void);

    /* Debug Print API */
    void debug_print(void);

private:
    /* caliblation Check */
    int is_caliblated(void);
    void stockData(void);
    void calculateOffset(void);

    /* Debug Print Helper */
    void print_debug_vec(float* vec, char* label);

    /* Check the error */
    void print_error_msg(int error_no);
};

// 加速度センサの設定
const char AccScaleSetting = 0b00000000;
//                           ||||||||- Not used
//                           |||||||-- Not used
//                           ||||||--- Not used
//                           |||||---- FS_SEL: 00:2[g] -> 16384[LSB/g]
//                           ||||-----         01:4[g] -> 8192[LSB/g]
//                           |||               10:8[g] -> 4096[LSB/g]
//                           |||               11:16[g]-> 2048[LSB/g]
//                           |||------ ZA_ST: Self-Test
//                           ||------- YA_ST: Self-Test
//                           |-------- XA_ST: Self-Test
const float AccScaleFactor = 1/16384.0f;

// Gyroセンサの設定
const char GyroScaleSetting = 0b00000000;
//                              ||||||||- Not used
//                              |||||||-- Not used
//                              ||||||--- Not used
//                              |||||---- FS_SEL: 00:250[deg/s]  -> 131 [LSB/(deg/s)]
//                              ||||-----         01:500[deg/s]  -> 65.5[LSB/(deg/s)]
//                              |||               10:1000[deg/s] -> 32.8[LSB/(deg/s)]
//                              |||               11:2000[deg/s] -> 16.4[LSB/(deg/s)]
//                              |||------ ZG_ST
//                              ||------- XG_ST
//                              |------- YG_ST
const float GyroScaleFactor = 1/131.0f;

// LPFの設定
const char LpfConfig = 0b00000100;
//                       ||||||||- DLPF_CONFIG
//                       |||||||-- 000(0):ACC  ->260[Hz](Delay:0[ms])
//                       ||||||---        Gyro ->256[Hz](Delay:0.98[ms])
//                       |||||     001(1):ACC  ->184[Hz](Delay:2.0[ms])
//                       |||||            Gyro ->188[Hz](Delay:1.9[ms])
//                       |||||     010(2):ACC  ->94[Hz](Delay:3.0[ms])
//                       |||||            Gyro ->98[Hz](Delay:2.8[ms])
//                       |||||     011(3):ACC  ->44[Hz](Delay:4.9[ms])
//                       |||||            Gyro ->42[Hz](Delay:4.8[ms])
//                       |||||     100(4):ACC  ->21[Hz](Delay:8.5[ms])
//                       |||||            Gyro ->20[Hz](Delay:8.3[ms])
//                       |||||     101(5):ACC  ->10[Hz](Delay:13.8[ms])
//                       |||||            Gyro ->10[Hz](Delay:13.4[ms])
//                       |||||     110(6):ACC  ->5[Hz](Delay:19.0[ms])
//                       |||||            Gyro ->5[Hz](Delay:18.6[ms])
//                       |||||     111(7):Researved                                        Gyro ->256[Hz](Delay:0.98[ms])
//                       |||||---- EXT_SYNC_SET
//                       ||||----- 
//                       |||------ 
//                       ||------- Not used
//                       |-------- Not used

// IMUの設定
const char IMU_POWER_MANAGEMENT_CONFIG = 0b00000000;
//                        ||||||||- CLK_SEL
//                        |||||||-- 
//                        ||||||--- 
//                        |||||---- TEMP_DIS
//                        ||||----- Not used
//                        |||------ CYCLE
//                        ||------- SLEEP
//                        |-------- DEVICE_RESET
#endif