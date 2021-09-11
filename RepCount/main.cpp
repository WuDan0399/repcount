/**
* Copyright (C) 2021 National University of Singapore & GovTech
* @author Ye Tong
*/

#include "mbed.h"
#include "LSM9DS1.h"
#include "mbed_power_mgmt.h"

#include <cstdint>
#include <time.h>
#include <deque>
#include <string>
#include "rep_counter_helper.h"


LSM9DS1 imu(PF_0,PF_1);
DigitalOut pwron2(PF_4);
DigitalOut pwron1(PG_15);
DigitalOut shutTof(PG_1);
DigitalOut pwronTof(PF_5);

PinName TX = PC_4;
PinName RX = PC_5;

static BufferedSerial serial_comm(TX, RX, 115200);
float ElapsedTime, CurrentTime, PreviousTime;
clock_t timer;

//messages
const char reset_message[] = "Resetting reps count\n";
const char normal_rep_count_message[] = "Angular rep count mode\n";
const char vert_rep_count_message[] = "Vertical rep count mode\n";

rep_counter_helper rep_counter;

void initSetup();
float getTime();

int main()
{      
    int AngularDirection;
    float MainTimerDelay = 0;

    initSetup();
    imu.begin();
    imu.calibration();

    while(1)
    {
        while(getTime() - MainTimerDelay < 0.020)
        {
            //delay

        }
        MainTimerDelay = getTime();

        //preprocess accel data;
        imu.readAccel();
        imu.readGyro();
        rep_counter.AccSensitivity = 2.0 / 32768.0; //from IMU datasheet
        rep_counter.GyroSensitivity = 70 * 0.001; //from IMU datasheet
        rep_counter.Prepossessing(imu.ax_raw, imu.ay_raw, imu.az_raw, imu.gx_raw, imu.gy_raw, imu.gz_raw);

        //select_acc_reference
        rep_counter.UpdateAccReference(getTime());

        //process gyro data;
        PreviousTime = CurrentTime;
        CurrentTime = getTime();
        ElapsedTime = CurrentTime - PreviousTime;

        //get current gyro orientation
        rep_counter.UpdateOrientation(ElapsedTime);

        rep_counter.StationaryFlag = rep_counter.CheckStationary(true);

        //update reference line (baseValue) every 1s
        rep_counter.UpdateGyroReference(getTime());

        //check for stationary
        if(rep_counter.StationaryFlag == false)
        {
            rep_counter.StationaryTimer = getTime();
            rep_counter.ResetFlag = false;
        }

        //reset
        if (rep_counter.ResetCheck(getTime()) == true)
        {
            serial_comm.write(reset_message, sizeof(reset_message));
        }

        //default mode or angular mode
        if(rep_counter.RepCountMode == 0 || rep_counter.RepCountMode == 1)
        {
        AngularDirection = rep_counter.AngularMotion(getTime());
        if(AngularDirection == 1)
        {
            //horizontal
            string message_LR = "- " + to_string(rep_counter.AngularRepsCount) + "\n";
            char message[message_LR.length() + 1];
            strcpy(message, message_LR.c_str());
            serial_comm.write(message, sizeof(message));
            //printf("%c %d\n",'-', repsCount);
        }
        else if(AngularDirection == 2)
        {
            string message_UD = "| " + to_string(rep_counter.AngularRepsCount) + "\n";
            char message[message_UD.length() + 1];
            strcpy(message, message_UD.c_str());
            serial_comm.write(message, sizeof(message));
            //printf("%c %d\n",'|', repsCount);
        }
        }
        
        //default mode or vertical mode
        if(rep_counter.RepCountMode == 0 || rep_counter.RepCountMode == 2)
        {
            if(rep_counter.VerticalMotion(getTime()) == true)
            {
                string message_s = "|| " + to_string(rep_counter.VertRepsCount) + "\n";
                char message[message_s.length() + 1];
                strcpy(message, message_s.c_str());
                serial_comm.write(message, sizeof(message));
            }
        }

        rep_counter.UpdateGyroWindow();
        rep_counter.DriftCounter += 1;
    }
}

/**
* @brief Initial setup.
*/
void initSetup()
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
}


/**
* @brief Get the reference time wrt start of the program
* @return reference time wrt start of the program
*/
float getTime()
{
    float current = 0;
    timer = clock();
    current = (float)timer/CLOCKS_PER_SEC;
    return current;
}
