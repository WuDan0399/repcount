/**
* Copyright (C) 2021 National University of Singapore & GovTech
* @author Ye Tong
*/

#include "rep_counter_helper.h"
#include <cstdint>
#include <cstdio>
#include <math.h>

/**
* @brief Preprossess the data with the chosen ssensitivity.
* @param ax_raw raw x-axis acceleration IMU reading
* @param ay_raw raw y-axis acceleration IMU reading
* @param az_raw raw z-axis acceleration IMU reading
* @param gx_raw raw x-axis gyroscope IMU reading
* @param gy_raw raw y-axis gyroscope IMU reading
* @param gz_raw raw z-axis gyroscope IMU reading
*/
void rep_counter_helper::Prepossessing(int16_t ax_raw, int16_t ay_raw, int16_t az_raw, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw)
{
    AccX = ax_raw * AccSensitivity * 1000; //scale by 1000
    AccY = ay_raw * AccSensitivity * 1000;
    AccZ = az_raw * AccSensitivity;
    GyroX = gx_raw * GyroSensitivity;
    GyroY = gy_raw * GyroSensitivity;
    GyroZ = gz_raw * GyroSensitivity;

    AccelerationRN();
    UpdateAccWindow();
}

/**
* @brief Taking average of small number of points to reduce noise.
*/
void rep_counter_helper::AccelerationRN()
{
    AccxWindow.push_front(AccX);
    if(AccxWindow.size() > 20)
    {
        AccxRn = GetAverage(AccxWindow);
        AccxWindow.pop_back();
    }

    AccyWindow.push_front(AccY);
    if(AccyWindow.size() > 20)
    {
        AccyRn = GetAverage(AccyWindow);
        AccyWindow.pop_back();
    }
}

/**
* @brief Update the x-axis and y-axis acceleration window
*/
void rep_counter_helper::UpdateAccWindow()
{
    AccxReferenceWindow.push_front(AccX);
    if(AccxReferenceWindow.size() > 200)
    {
        AccxReferenceWindow.pop_back();
    }

    AccyReferenceWindow.push_front(AccY);
    if(AccyReferenceWindow.size() > 200)
    {
        AccyReferenceWindow.pop_back();
    }

}

/**
* @brief Choosse the more domimant axis and update the chosen 
* acc value and acc reference value for comparison.
* @param curr_time current timestamp.
*/
void rep_counter_helper::UpdateAccReference(float curr_time) //get time input
{
    if(AccxReferenceWindow.size() > 20 && AccyReferenceWindow.size() > 20 && curr_time - AccUpdateTimer > 1) //update every 1s
    {
        AccxRef = GetAverage(AccxReferenceWindow);
        AccyRef = GetAverage(AccyReferenceWindow);
        AccUpdateTimer = curr_time;
    }

    //update only if a change is requested (i.e. AccAxis = 0)
    if((abs(AccxRef) > abs(AccyRef)) && AccAxis == 0)
    {
        AccAxis = 1; //use x-axis accel value
    }
    else if ((abs(AccyRef) > abs(AccxRef)) && AccAxis == 0)
    {
        AccAxis = 2; //use y-axis accel value
    }

    //always use the most dominant axis
    if (AccAxis == 1) //x-axis more dominant
    {
        AccFinal = AccxRn;
        AccRefFinal = AccxRef;

        //check if still more dominant, else update
        if(abs(AccxRef) < 300 && abs(AccyRef) > 400)
        {
            AccAxis = 0;
            VertDataChangeFlag = true;
        }
    }
    else //y-axis more dominant
    {
        AccFinal = AccyRn;
        AccRefFinal = AccyRef;

        //check if still more dominant, else update
        if(abs(AccyRef) < 300 && abs(AccyRef) > 400)
        {
            AccAxis = 0;
            VertDataChangeFlag = true;
        }
    }
}

/**
* @brief Update the gyro orientation.
* @param elapsed_time time elapsed after one loop of the program.
*/
void rep_counter_helper::UpdateOrientation(float elapsed_time)
{
        //obtain the orientation
        GyroAngleX = GyroAngleX + GyroX * elapsed_time;
        GyroAngleY = GyroAngleY + GyroY * elapsed_time;
        GyroAngleZ = GyroAngleZ + GyroZ * elapsed_time;

        //minimmise drift
        if (DriftCounter >= 25)
        {
            GyroAngleZ = floor(GyroAngleZ);
            DriftCounter = 0;
        }
}

/**
* @brief Update the gyro reference value.
* @param curr_time current timestamp.
*/
void rep_counter_helper::UpdateGyroReference(float curr_time)
{
    float GyrozAvgValue = 0;
    if (curr_time - GyroUpdateTimer > 1 && GyrozWindow.size() >= 80) //update every 1s
    {
        GyrozAvgValue = GetAverage(GyrozWindow);
        if(StationaryFlag == true)
        {
            GyrozRefValue = GyrozAvgValue;
        }
        else 
        {
            if(GyrozAvgValue <= GyrozRefValue)
            {
                //waveform below reference line
                if(abs(GyrozRefValue - GyrozAvgValue) >= 40)
                {
                    //avg value too far from reference line, move reference line closer to avg value
                    GyrozRefValue = GyrozRefValue - (abs(GyrozRefValue - GyrozAvgValue) - 40);
                }
                else if(abs(GyrozRefValue - GyrozAvgValue) < 30)
                {
                    //avg value too close to reference line, move referencee line further away from avg value
                    GyrozRefValue = GyrozRefValue + 32 - abs(GyrozRefValue - GyrozAvgValue);
                }
            }
            else
            {
                //waveform above reference line
                if(abs(GyrozRefValue - GyrozAvgValue) >= 40)
                {
                    //avg value too far from reference line, move reference line closer to avg value
                    GyrozRefValue = GyrozRefValue + (abs(GyrozRefValue - GyrozAvgValue) - 40);
                }
                else if(abs(GyrozRefValue - GyrozAvgValue) < 30)
                {
                    //avg value too close to reference line, move referencee line further away from avg value
                    GyrozRefValue = GyrozRefValue - (32 - abs(GyrozRefValue - GyrozAvgValue));
                }
            }
        }
        GyroUpdateTimer = curr_time;
    }
}

/**
* @brief Update the gyrowindow.
*/
void rep_counter_helper::UpdateGyroWindow() //get time input
{
    //TODO: error correction(rebase to 0?) if drift too much
    GyrozWindow.push_front(GyroAngleZ);

    if(GyrozWindow.size() > 100)
    {
        while(GyrozWindow.size() > 80)
        {
            GyrozWindow.pop_back();
        }
    }
}

/**
* @brief Reset the program is the criteria have met.
* @param curr_time current timestamp.
* @return true if data are being reset, false otherwise.
*/
bool rep_counter_helper::ResetCheck(float curr_time)
{
    if(curr_time - StationaryTimer > 5 && ResetFlag == false)
    {
        if(CheckStationary(false)) //TODO: why false?
        {
            AngularRepsCount = 0;
            VertRepsCount = 0;
            GyroAngleZ = 0;
            GyrozRefValue = 0;

            AccAxis = 0;

            RepCountMode = 0;
            GyrozWindow.clear();
            AccxWindow.clear();
            AccyWindow.clear();
            AccxReferenceWindow.clear();
            AccyReferenceWindow.clear();
            StationaryTimer = curr_time;
            ResetFlag = true;
            return true;
        }
    }
    return false;
}

/**
* @brief Get the average value of the data in the deque.
* @param window The window to get the average value.
* @return The average value.
*/
float rep_counter_helper::GetAverage(std::deque<int32_t> window)
{
    float sum = 0;
    float average;
    for(int i = 0; i < window.size(); i++)
    {
        sum += window[i];
    }
    average = sum / window.size();
    return average;
}

/**
* @brief Check and update angular rep counts.
* @param curr_time current timestamp.
* @return The direction of the angular motion if checks passsed, 0 if the checks failed.
*/
int rep_counter_helper::AngularMotion(float curr_time)
{
    int direction;

    //peak range
    if(abs(GyroAngleZ - GyrozRefValue) >= 70 && abs(GyroAngleZ - GyrozRefValue) <= 200 && AngularPeakFlag == false)
    {
        //wait for 0.5 second before checking for next reps
        if(curr_time - AngularTroughTime > 0.5)
        {
            AngularPeakFlag = true;
            AngularPeakTime = curr_time;
        }
    }

    //trough range
    if(abs(GyroAngleZ - GyrozRefValue) < 30 && AngularPeakFlag == true)
    {
        //check for trough within 2 second
        if(curr_time - AngularPeakTime <= 2 && curr_time - AngularPeakTime >= 0.4)
        {
        AngularTroughFlag = true;
        AngularTroughTime = curr_time;
        }
        else if(curr_time - AngularPeakTime > 2)
        {
            AngularPeakFlag = false;
        }
    }

    if(AngularPeakFlag == true && AngularTroughFlag == true)
    {
        if(RepCountMode == 0)
        {
            RepCountMode = 1;
        }

        AngularPeakFlag = false;
        AngularTroughFlag = false;
        AngularRepsCount += 1;
        direction = GetDirection();
        return direction;
    }
    return 0;
}

/**
* @brief Check and update vertical reps count.
* @param curr_time current timestamp.
* @return true if the checks passes, false if otherwise.
*/
bool rep_counter_helper::VerticalMotion(float curr_time)
{
    int CurrSign = 0;

    //check acc value is above or below reference value
    if(AccFinal > AccRefFinal)
    {
        CurrSign = 1;
    }
    else
    {
        CurrSign = 2;
    }

    //peak1
    if(abs(AccFinal - AccRefFinal) > 120 && VertFirstPeakFlag == false)
    {
        //wait for 0.5 second before checking for next reps
        if(curr_time - VertSecondPeakTime > 0.5 && curr_time - VertFirstPeakFlag > 0.8) //reduce spamming
        {
            VertFirstPeakFlag = true;
            VertRepTimeout = false;

            //check direction of first peak
            if(AccFinal > AccRefFinal)
            {
                VertFirstPeakDirection = 1;
            }
            else
            {
                VertFirstPeakDirection = 2;
            }
            VertFirstPeakTime = curr_time;
        }
    }
    //peak2
    if(abs(AccFinal - AccRefFinal) > 100 && VertSecondPeakFlag == false && VertTroughFlag == true && VertFirstPeakFlag == true)
    {
        if(curr_time - VertFirstPeakTime < 2 && curr_time - VertFirstPeakTime > 0.5)
        {
            //check if direction of second peak is the same as first peak
            if(CurrSign == VertFirstPeakDirection)
            {
                VertSecondPeakFlag = true;
                VertSecondPeakTime = curr_time;
            }
        }
    }

    //trough
    if(abs(AccFinal - AccRefFinal) > 150 && VertFirstPeakFlag == true && VertTroughFlag == false)
    {
        //check for trough within 2 second
        if(curr_time - VertFirstPeakTime < 2 && curr_time - VertFirstPeakTime > 0.3)
        {
            if(CurrSign != VertFirstPeakDirection && VertFirstPeakDirection != 0)
            {
                VertTroughFlag = true;
            }
        }
    }

    if(VertFirstPeakFlag == true && VertSecondPeakFlag == true && VertTroughFlag == true)
    {
        if(RepCountMode == 0)
        {
            RepCountMode = 2;
        }
        VertFirstPeakFlag = false;
        VertSecondPeakFlag = false;
        VertTroughFlag = false;
        VertFirstPeakDirection = 0;
        VertRepsCount += 1;
        
        return true;
    }
    else if(curr_time - VertFirstPeakTime > 2 && VertRepTimeout == false)
    {
        VertFirstPeakFlag = false;
        VertSecondPeakFlag = false;
        VertTroughFlag = false;
        VertRepTimeout = true;
    }
    else if(VertDataChangeFlag == true)
    {
        VertFirstPeakFlag = false;
        VertSecondPeakFlag = false;
        VertTroughFlag = false;
        VertDataChangeFlag = false;
    }
    return false;
}

/**
* @brief Check if the device is stationary.
* @param isAngular Determine if the check is angular or vertical.
* @return true of the device is stationary, false if otherwise.
*/
bool rep_counter_helper::CheckStationary(bool isAngular)
{
    float max = -100000; 
    float min = 100000;
    float max2 = -100000; 
    float min2 = 100000;
    float diff = 0;
    float diff2 = 0;

    //check if stationary with angular values
    if(isAngular == true)
    {
        for(int i = 0; i < GyrozWindow.size(); i++)
        {
            if(GyrozWindow[i] > max)
            {
                max = GyrozWindow[i];
            }
            if(GyrozWindow[i] < min)
            {
                min = GyrozWindow[i];
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
    else //check if stationary with acc values
    {
        for(int i = 0; i < AccxReferenceWindow.size(); i++)
        {
            if(AccxReferenceWindow[i] > max)
            {
                max = AccxReferenceWindow[i];
            }
            if(AccxReferenceWindow[i] < min)
            {
                min = AccxReferenceWindow[i];
            }
        }

        for(int i = 0; i < AccyReferenceWindow.size(); i++)
        {
            if(AccyReferenceWindow[i] > max2)
            {
                max2 = AccyReferenceWindow[i];
            }
            if(AccyReferenceWindow[i] < min2)
            {
                min2 = AccyReferenceWindow[i];
            }
        }

        diff = abs(max - min);
        diff2 = abs(max2 - min2);

        //stationary for both x and y axis
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

/**
* @brief Get the direction that the device is in.
* @return direction = 1 if it is horizontal, direction = 2 if it is vertical.
*/
int rep_counter_helper::GetDirection()
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
