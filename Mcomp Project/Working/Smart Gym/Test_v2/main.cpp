#include "mbed.h"
#include "LSM9DS1.h"
#include "mbed_power_mgmt.h"

#include <cstdint>
#include <math.h>
#include <time.h>
#include <chrono>
#include <deque>
#include <cmath>

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
float peakTimeCounter, troughTimeCounter;
int repsCount;

bool peak, trough, update;

clock_t timer;

std::deque<int32_t> gyroz_window;

float getAverage();
void repsCounter();
float getTime();
bool checkStationary();
int getDirection();

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
    float correction_timer = 0;
    float main_timer_delay = 0;
    float avg_value = 0;
    float timer_prev = 0;
    bool reset_flag = false;
    bool stationary_flag;
    float stationary_timer;
    baseValue = 0;

    while(1)
    {
        while(getTime() - main_timer_delay < 0.020)
        {
            //delay

        }
        main_timer_delay = getTime();

        //process accel data;
        imu.readAccel();
        AccSensitivty = 2.0 / 32768.0;
        // AccX = imu.ax_raw * AccSensitivty;
        // AccY = imu.ay_raw * AccSensitivty;
        AccZ = imu.az_raw * AccSensitivty;

        //printf("%f %f %f\n",AccX, AccY, AccZ);
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
        // GyroX = imu.gx_raw * 70*0.001;
        // GyroY = imu.gy_raw * 70*0.001;
        GyroZ = imu.gz_raw * 70*0.001;

        //obtain the orientation
        // gyroAngleX = gyroAngleX + GyroX * elapsedTime;
        // gyroAngleY = gyroAngleY + GyroY * elapsedTime;
        gyroAngleZ = gyroAngleZ + GyroZ * elapsedTime;

        //minimmise drift
        if (drift_counter >= 25)
        {
            gyroAngleZ = floor(gyroAngleZ);
            drift_counter = 0;
        }
        //printf("%f %f %f %f\n", gyroAngleX, gyroAngleY, gyroAngleZ, elapsedTime);
        printf("%f %f %f\n", gyroAngleZ, baseValue, avg_value);
        //printf("%f\n", (float)((float)timer/CLOCKS_PER_SEC));
        //printf("%d %d %d %d %d %d\n", imu.gx_raw, imu.gy_raw, imu.gz_raw,imu.ax_raw,imu.ay_raw, imu.az_raw);
        stationary_flag = checkStationary();

        //update reference line (baseValue) every 1s
        if (getTime() - correction_timer > 1 && gyroz_window.size() >= 80)
        {
            avg_value = getAverage();
            if(stationary_flag == true)
            {
                baseValue = avg_value;
            }
            else 
            {
                if(avg_value <= baseValue)
                {
                    //waveform below reference line
                    if(abs(baseValue - avg_value) >= 40)
                    {
                        //avg value too far from reference line, move reference line closer to avg value
                        baseValue = baseValue - (abs(baseValue - avg_value) - 40);
                    }
                    else if(abs(baseValue - avg_value) < 30)
                    {
                        //avg value too close to reference line, move referencee line further away from avg value
                        baseValue = baseValue + 32 - abs(baseValue - avg_value);
                    }
                }
                else
                {
                    //waveform above reference line
                    if(abs(baseValue - avg_value) >= 40)
                    {
                        //avg value too far from reference line, move reference line closer to avg value
                        baseValue = baseValue + (abs(baseValue - avg_value) - 40);
                    }
                    else if(abs(baseValue - avg_value) < 30)
                    {
                        //avg value too close to reference line, move referencee line further away from avg value
                        baseValue = baseValue - (32 - abs(baseValue - avg_value));
                    }
                }
            }

            //printf("Current baseValue: %f\n", baseValue);
            correction_timer = getTime();
        }

        //check for stationary
        if(stationary_flag == false)
        {
            stationary_timer = getTime();
            reset_flag = false;
        }

        //reset
        if(getTime() - stationary_timer > 5 && reset_flag == false)
        {
            printf("Resetting reps count\n");
            repsCount = 0;
            gyroAngleZ = 0;
            baseValue = 0;
            gyroz_window.clear();
            stationary_timer = getTime();
            reset_flag = true;
        }

        repsCounter();
        //TODO: error correction(rebase to 0?) if drift too much
        gyroz_window.push_front(int(gyroAngleZ*1000));

        if(gyroz_window.size() > 100)
            {
                while(gyroz_window.size() > 80)
                {
                    gyroz_window.pop_back();
                }
            }
        drift_counter += 1;
    }
}

/*
* Get the average of all values of the data point in gyroz_window.
* @return average of all values.
*/
float getAverage()
{
    int32_t sum = 0;
    float average;
    for(int i = 0; i < gyroz_window.size(); i++)
    {
        sum += gyroz_window[i];
    }

    average = sum / float(gyroz_window.size() * 1000);
    //printf("Average = %f, Size = %d\n", average, gyroz_window.size());
    return average;
}

/*
* Update and print the reps done by the user.
*/
void repsCounter()
{
    float currTime = getTime();
    int direction;

    //peak range
    if(abs(gyroAngleZ - baseValue) >= 70 && abs(gyroAngleZ - baseValue) <= 200 && peak == false)
    {
        //wait for 0.5 second before checking for next reps
        if(currTime - troughTimeCounter > 0.5)
        {
            peak = true;
            //printf("Peak value: %f\n", gyroAngleZ);
            //printf("Base value: %f\n", baseValue);
            peakTimeCounter = getTime();
        }
    }

    //trough range
    if(abs(gyroAngleZ - baseValue) < 30 && peak == true)
    {
        //check for trough within 2 second
        if(currTime - peakTimeCounter < 2 && currTime - peakTimeCounter > 0.4)
        {
        trough = true;
        //printf("Trough value: %f\n", gyroAngleZ);
        //printf("Base value: %f\n", baseValue);
        troughTimeCounter = getTime();
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
        direction = getDirection();
        if(direction == 1)
        {
            //horizontal
            //printf("%c %d\n",'-', repsCount);
        }
        else if(direction == 2)
        {

            //printf("%c %d\n",'|', repsCount);
        }
        //printf("%d\n", repsCount);
    }
}

/*
* Get the reference time wrt start of the program
* @return reference time wrt start of the program
*/
float getTime()
{
    float current = 0;
    timer = clock();
    current = (float)timer/CLOCKS_PER_SEC;
    return current;
}

/*
* Check if the dumbell is in stationary position base on the data points in gyroz_window.
* @return true if all points are close to each other.
* @return false if range of value is large.
*/
bool checkStationary()
{    
    float max = -10000000; 
    float min = 10000000;
    float diff = 0;
    for(int i = 0; i < gyroz_window.size(); i++)
    {
        if(gyroz_window[i] > max)
        {
            max = gyroz_window[i];
        }
        if(gyroz_window[i] < min)
        {
            min = gyroz_window[i];
        }
    }
    diff = abs(max - min);
    if(diff < 30000) //30 degree * 1000
    {
        return true;
    }
    else
    {
        return false;
    }
}

int getDirection()
{
    int direction = 0;
    if(AccZ > 0.6 || AccZ < -0.6)
    {
        //horizontal
        direction = 1;
    }
    else if(AccZ < 0.4 || AccZ > -0.4)
    {
        //vertical
        direction = 2;
    }
    return direction;
}