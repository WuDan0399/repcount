#include "mbed.h"
#include "LSM9DS1.h"
#include "mbed_power_mgmt.h"
#include "mbed_wait_api.h"
#include "medianfilter.h"
#include "ns_timer.h"
#include <algorithm>

#include <cmath>
#include <cstdio>
#include <math.h>
#include <time.h>
#include <vector>
#include <string>
#include <chrono>
#include <deque>

#define PI 3.14159265358979323846

LSM9DS1 imu(PF_0,PF_1);
DigitalOut pwron2(PF_4);
DigitalOut pwron1(PG_15);
DigitalOut shutTof(PG_1);
DigitalOut pwronTof(PF_5);
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccSensitivty, GyroSensitivty;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime, currentTime, previousTime;
float baseValue;
float timeCounter;
int repsCount;

bool peak, trough, update;

clock_t timer;

std::deque<float> gyroz_window;

float average();
void repsCounter();
float getTime();

int main()
{
    pwron2 = 1;
    pwron1 = 1;
    shutTof = 1;
    pwronTof = 1;

    printf("pwron1 is:%d\r\n", pwron1.is_connected());
    printf("pwron1 read:%d\r\n", pwron1.read());
    printf("pwron2 is :%d\r\n", pwron2.is_connected());
    printf("pwron2 read:%d\r\n", pwron2.read());
    printf("stis:%d\r\n", shutTof.is_connected());
    printf("str:%d\r\n", shutTof.read());
    printf("ptis:%d\r\n", pwronTof.is_connected());
    printf("ptr:%d\r\n", pwronTof.read());
    
    imu.begin();
    imu.calibration();

    int drift_counter = 0;
    float base_counter = 0;
    baseValue = 0;
    while(1)
    {
        //process accel data;
        // imu.readAccel();
        // AccSensitivty = 2.0 / 32768.0;
        // AccX = imu.ax_raw * AccSensitivty;
        // AccY = imu.ay_raw * AccSensitivty;
        // AccZ = imu.az_raw * AccSensitivty;

        // //printf("%f %f %f\n",AccX, AccY, AccZ);
        // //printf("%d %d %d\n",imu.ax_raw, imu.ay_raw, imu.az_raw);
        // //Calculate Roll and Pitch from accel data.
        // accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
        // accAngleY = (atan(-1 * AccX/ sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);

        //printf("%f %f\n", accAngleX, accAngleY);
        //process gyro data;
        previousTime = currentTime;
        currentTime = getTime();
        elapsedTime = (currentTime - previousTime);
        imu.readGyro();

        //convert raw data to degree per second
        GyroX = imu.gx_raw * 70*0.001;
        GyroY = imu.gy_raw * 70*0.001;
        GyroZ = imu.gz_raw * 70*0.001;

        //obtain the orientation
        gyroAngleX = gyroAngleX + GyroX * elapsedTime;
        gyroAngleY = gyroAngleY + GyroY * elapsedTime;
        gyroAngleZ = gyroAngleZ + GyroZ * elapsedTime;
        gyroAngleX = fmod(gyroAngleX,360);
        gyroAngleY = fmod(gyroAngleY,360);
        gyroAngleZ = fmod(gyroAngleZ,360);

        //minimmise drift when stationary
        if (drift_counter >= 25)
        {
            gyroAngleZ = floor(gyroAngleZ);
            drift_counter = 0;
        }
        //printf("%f %f %f %f\n", gyroAngleX, gyroAngleY, gyroAngleZ, elapsedTime);
        printf("%f %f %d\n", gyroAngleZ, baseValue, 0);
        //printf("%f\n", (float)((float)timer/CLOCKS_PER_SEC));
        //printf("%d %d %d %d %d %d\n", imu.gx_raw, imu.gy_raw, imu.gz_raw,imu.ax_raw,imu.ay_raw, imu.az_raw);
        
        //weighted update of base value
        if ((getTime() - base_counter) >= 2 && gyroz_window.size() >= 200)
        {
            baseValue = 0.6 * baseValue + 0.4 * average();
            base_counter = getTime();
            //printf("Current baseValue: %f\n", baseValue);
        }

        repsCounter();



        gyroz_window.push_front(gyroAngleZ);
        if (gyroz_window.size() > 200)
        {
            gyroz_window.pop_back();
        }
        drift_counter += 1;
        wait_us(40);
    }
}

int max_pos = 0;
int min_pos = 0;
int max_position = 0; //position of the previous trough, else set to fix number.
int min_position = 0;
bool reps_flag = false;

float average()
{
    //get the current average value
    float sum;
    for(int i = 0; i < gyroz_window.size(); i++)
    {
        sum += gyroz_window[i];
        //printf("%f",gyroz_window[i]);
    }
    //printf("size %d", gyroz_window.size());
    return sum / gyroz_window.size();
}

void repsCounter()
{
    if(abs(gyroAngleZ - baseValue) >= 70 && abs(gyroAngleZ - baseValue) <= 200 && peak == false)
    {
        //start time counter
        peak = true;
        printf("Peak value: %f\n", gyroAngleZ);
        timeCounter = getTime();
    }

    if(abs(gyroAngleZ - baseValue) < 40 && peak == true)
    {
        //within 3 second
        float currTime = 0;
        currTime = getTime();
        if(currTime - timeCounter < 3 && currTime - timeCounter > 0.5)
        {
            trough = true;
            printf("Trough value: %f\n", gyroAngleZ);
        }
        else
        {
            peak = false;
        }
    }

    if(peak == true && trough == true)
    {
        peak = false;
        trough = false;
        repsCount += 1;
        printf("%d\n", repsCount);
    }
}

float getTime()
{
    float current = 0;
    timer = clock();
    current = (float)timer/CLOCKS_PER_SEC;
    return current;
}

// int max()
// {
//     int temp_max = -10000;
//     for (int i =0 ; i < gyroz_window.size(); i++)
//     {
//         if (gyroz_window[i] > temp_max)
//         {
//             temp_max =  gyroz_window[i];
//         }
//         else 
//         {
//             max_position = i;
//             for(int j = 0; j < max_position; j++)
//             {
//                 gyroz_window.pop_back();
//             }
//             break;
//         }
//     }
//     return temp_max;
// }

// int min()
// {
//     int temp_min = 10000;
//     for (int i =0 ; i < gyroz_window.size(); i++)
//     {
//         if (gyroz_window[i] < temp_min)
//             temp_min =  gyroz_window[i];
//     }
//     return temp_min;
// }