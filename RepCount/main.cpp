#include "mbed.h"
#include "LSM9DS1.h"
#include "mbed_power_mgmt.h"

#include <cstdint>
#include <math.h>
#include <time.h>
#include <chrono>
#include <deque>
#include <cmath>
#include <string>

#define PI 3.14159265358979323846

LSM9DS1 imu(PF_0,PF_1);
DigitalOut pwron2(PF_4);
DigitalOut pwron1(PG_15);
DigitalOut shutTof(PG_1);
DigitalOut pwronTof(PF_5);

PinName TX = PC_4;
PinName RX = PC_5;

static BufferedSerial serial_comm(TX, RX, 115200);

int32_t AccX, AccY;
float AccZ;
float GyroX, GyroY, GyroZ;
float AccSensitivty, GyroSensitivty;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float elapsedTime, currentTime, previousTime;
float baseValue;
float peakTimeCounter, troughTimeCounter;
int repsCount;

bool peak, trough, update;

//messages
const char reset_message[] = "Resetting reps count\n";
const char normal_rep_count_message[] = "Normal rep count mode\n";
const char vert_rep_count_message[] = "Vertical rep count mode\n";
//

//vert~~~~~~~~~~~~
std::deque<int32_t> accx_reference_window;
std::deque<int32_t> accy_reference_window;
int accFinal = 0; //chosen final acc value
int accRefFinal = 0; //chosen final acc reference value
bool firstPeakFlag;
bool secondPeakFlag;
int firstPeakDirection;
bool vertTroughFlag;
float firstPeakTimer;
float secondPeakTimer = 0;
int vertRepsCount;
bool changeFlag;
bool vertRepTimeout;
int repCountMode;
//~~~~~~~~~~~~~~~~

clock_t timer;

std::deque<int32_t> gyroz_window;

float getAverage();
void repsCounter();
float getTime();
bool checkStationary(bool isAngular);
int getDirection();
float getAverageAcc(deque<int32_t> acc_window);
void vertRepCounter();

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
    repCountMode = 0; //default normal rep count mode

    //Prev X,Y~~~~~~~Testing
    std::deque<int32_t> accx_rn_window; //x-axis reduce noise
    std::deque<int32_t> accy_rn_window; //y-axis reduce noise
    int accx_ref = 0; //reference point
    int accy_ref = 0;
    int accx_rn = 0; //reduce noise point
    int accy_rn = 0;
    int acc_flag = 0; //choose acc from x and y
    float acc_timer = 0;
    //~~~~~~~~~~~~~~~

    int value = 0;

    

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
        AccX = imu.ax_raw * AccSensitivty * 1000; //scale by 1000
        AccY = imu.ay_raw * AccSensitivty * 1000;
        AccZ = imu.az_raw * AccSensitivty;

        //*****************************************************************Testing~
        //reduce noise
        accx_rn_window.push_front(AccX);
        if(accx_rn_window.size() > 20)
        {
            accx_rn = getAverageAcc(accx_rn_window);
            accx_rn_window.pop_back();
        }

        accy_rn_window.push_front(AccY);
        if(accy_rn_window.size() > 20)
        {
            accy_rn = getAverageAcc(accy_rn_window);
            accy_rn_window.pop_back();
        }

        //get reference point
        //get another timer for this?
        accx_reference_window.push_front(AccX);
        if(accx_reference_window.size() > 200)
        {
            accx_reference_window.pop_back();
        }

        accy_reference_window.push_front(AccY);
        if(accy_reference_window.size() > 200)
        {
            accy_reference_window.pop_back();
        }

        if(accx_reference_window.size() > 20 && accy_reference_window.size() > 20 && getTime() - acc_timer > 1) //update every 1s
        {
            accx_ref = getAverageAcc(accx_reference_window);
            accy_ref = getAverageAcc(accy_reference_window);
            acc_timer = getTime();
        }

        if((abs(accx_ref) > abs(accy_ref)) && acc_flag == 0) //update only if a change is requested
        {
            acc_flag = 1; //use x-axis accel value
        }
        else if ((abs(accy_ref) > abs(accx_ref)) && acc_flag == 0)
        {
            acc_flag = 2; //use y-axis accel value
        }

        //always use the most significant axis
        if (acc_flag == 1) //current choice 
        {
            accFinal = accx_rn;
            accRefFinal = accx_ref;

            if(abs(accx_ref) < 300 && abs(accy_ref) > 400) //update only if current axis choice is less significant
            {
                acc_flag = 0;
                changeFlag = true;
            }
        }
        else
        {
            accFinal = accy_rn;
            accRefFinal = accy_ref;
            if(abs(accy_ref) < 300 && abs(accx_ref) > 400)
            {
                acc_flag = 0;
                changeFlag = true;
            }
        }
        //TODO: make sure that it is not in normal rep count mode
        //*******************************************************************
        //printf("%d %d %d %d\n",accx_rn, accy_rn, accx_ref, accy_ref);
        //printf("%d %d\n",accFinal, accRefFinal);
        //printf("%d %d\n",avgX, avgY);
        //*****************************************************************

        //printf("%f\n",AccZ);
        // prevX = AccX;
        // prevY = AccY;
        //printf("%d %d %d\n",imu.ax_raw, imu.ay_raw, imu.az_raw);
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
        //printf("%f %f %f\n", gyroAngleZ, baseValue, avg_value);
        //printf("%f\n", (float)((float)timer/CLOCKS_PER_SEC));
        //printf("%d %d %d %d %d %d\n", imu.gx_raw, imu.gy_raw, imu.gz_raw,imu.ax_raw,imu.ay_raw, imu.az_raw);
        stationary_flag = checkStationary(true);

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
        //TODO: also consider acceleration
        if(getTime() - stationary_timer > 5 && reset_flag == false)
        {
            if(checkStationary(false))
            {
                //printf("Resetting reps count\n");
                serial_comm.write(reset_message, sizeof(reset_message));
                repsCount = 0;
                vertRepsCount = 0;
                gyroAngleZ = 0;
                baseValue = 0;

                acc_flag = 0;

                repCountMode = 0;
                gyroz_window.clear();
                accx_rn_window.clear();
                accy_rn_window.clear();
                accx_reference_window.clear();
                accy_reference_window.clear();
                stationary_timer = getTime();
                reset_flag = true;
            }
        }

        if(repCountMode == 0 || repCountMode == 1)
        {
        repsCounter();
        }
        
        if(repCountMode == 0 || repCountMode == 2)
        {
        vertRepCounter();
        }

        //TODO: error correction(rebase to 0?) if drift too much
        gyroz_window.push_front(gyroAngleZ);

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
    float sum = 0;
    float average;
    for(int i = 0; i < gyroz_window.size(); i++)
    {
        sum += gyroz_window[i];
    }

    average = sum / gyroz_window.size();
    //printf("Average = %f, Size = %d\n", average, gyroz_window.size());
    return average;
}

float getAverageAcc(deque<int32_t> acc_window) //TODO: maybe merge with getAverage
{
    float sum = 0;
    int32_t average;
    for(int i = 0; i < acc_window.size(); i++)
    {
        sum += acc_window[i];
    }

    average = sum / acc_window.size();
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
        if(repCountMode == 0)
        {
            repCountMode = 1;
            //printf("setting normal rep count\n");
            //serial_comm.write(normal_rep_count_message, sizeof(normal_rep_count_message)+1);
        }

        peak = false;
        trough = false;
        repsCount += 1;
        direction = getDirection();
        if(direction == 1)
        {
            //horizontal
            string message_LR = "- " + to_string(repsCount) + "\n";
            char message[message_LR.length() + 1];
            strcpy(message, message_LR.c_str());
            serial_comm.write(message, sizeof(message));
            //printf("%c %d\n",'-', repsCount);
        }
        else if(direction == 2)
        {
            string message_UD = "| " + to_string(repsCount) + "\n";
            char message[message_UD.length() + 1];
            strcpy(message, message_UD.c_str());
            serial_comm.write(message, sizeof(message));
            //printf("%c %d\n",'|', repsCount);
        }
        //printf("%d\n", repsCount);
    }
}

void vertRepCounter()
{
    float currTime = getTime();
    int currSign = 0;

    if(accFinal > accRefFinal)
    {
        currSign = 1;
    }
    else
    {
        currSign = 2;
    }

    //peak1
    if(abs(accFinal - accRefFinal) > 120 && firstPeakFlag == false)
    {
        //wait for 0.5 second before checking for next reps
        if(currTime - secondPeakTimer > 0.5 && currTime - firstPeakFlag > 0.8) //reduce spamming
        {
            firstPeakFlag = true;
            vertRepTimeout = false;
            if(accFinal > accRefFinal)
            {
                firstPeakDirection = 1;
            }
            else
            {
                firstPeakDirection = 2;
            }
            firstPeakTimer = getTime();
            //printf("peak1\n");
        }
    }
    //peak2
    if(abs(accFinal - accRefFinal) > 100 && secondPeakFlag == false && vertTroughFlag == true && firstPeakFlag == true)
    {
        if(currTime - firstPeakTimer < 2 && currTime - firstPeakTimer > 0.5)
        {
            if(currSign == firstPeakDirection)
            {
                secondPeakFlag = true;
                secondPeakTimer = getTime();
                //printf("peak2\n");
            }
        }
    }

    //trough
    if(abs(accFinal - accRefFinal) > 150 && firstPeakFlag == true && vertTroughFlag == false)
    {
        //check for trough within 2 second
        if(currTime - firstPeakTimer < 2 && currTime - firstPeakTimer > 0.3)
        {
            if(currSign != firstPeakDirection && firstPeakDirection != 0)
            {
                vertTroughFlag = true;
                //printf("trough\n");
            }
        }
    }

    if(firstPeakFlag == true && secondPeakFlag == true && vertTroughFlag == true)
    {
        if(repCountMode == 0)
        {
            repCountMode = 2;
            //serial_comm.write(vert_rep_count_message, sizeof(vert_rep_count_message)+1);
            //printf("setting vertical rep count\n");
        }
        firstPeakFlag = false;
        secondPeakFlag = false;
        vertTroughFlag = false;
        firstPeakDirection = 0;
        vertRepsCount += 1;
        
        string message_s = "|| " + to_string(vertRepsCount) + "\n";
        char message[message_s.length() + 1];
        strcpy(message, message_s.c_str());
        serial_comm.write(message, sizeof(message));
        //printf("|| %d \n", vertRepsCount);
    }
    else if(currTime - firstPeakTimer > 2 && vertRepTimeout == false)
    {
        firstPeakFlag = false;
        secondPeakFlag = false;
        vertTroughFlag = false;
        vertRepTimeout = true;
    }
    else if(changeFlag == true)
    {
        firstPeakFlag = false;
        secondPeakFlag = false;
        vertTroughFlag = false;
        changeFlag = false;
    }
    //printf("change flag :%d \n", changeFlag);
    //printf("peak1 flag value outside:%d\n", firstPeakFlag);

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
bool checkStationary(bool isAngular)
{   
    float max = -100000; 
    float min = 100000;
    float max2 = -100000; 
    float min2 = 100000;
    float diff = 0;
    float diff2 = 0;

    if(isAngular == true)
    {
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
        if(diff < 30)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        for(int i = 0; i < accx_reference_window.size(); i++)
        {
            if(accx_reference_window[i] > max)
            {
                max = accx_reference_window[i];
            }
            if(accx_reference_window[i] < min)
            {
                min = accx_reference_window[i];
            }
        }

        for(int i = 0; i < accy_reference_window.size(); i++)
        {
            if(accy_reference_window[i] > max2)
            {
                max2 = accy_reference_window[i];
            }
            if(accy_reference_window[i] < min2)
            {
                min2 = accy_reference_window[i];
            }
        }

        diff = abs(max - min);
        diff2 = abs(max2 - min2);
        if(diff < 150 && diff2 < 150)
        {
            return true;
        }
        else
        {
            return false;
        }
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