/**
* Copyright (C) 2021 National University of Singapore & GovTech
* @author Ye Tong
*/

#ifndef REP_COUNTER_HELPER_H_
#define REP_COUNTER_HELPER_H_

#include <deque>

class rep_counter_helper
{

public:

    // variables
    float AccSensitivity, GyroSensitivity; //set sensitivity
    int DriftCounter = 0;
    bool ResetFlag = false;
    
    bool StationaryFlag;
    float StationaryTimer;

    int AngularRepsCount; //angular motion rep counter
    int VertRepsCount; //vertical motion rep counter
    int RepCountMode = 0; //angular motion mode = 1, vertical motion mode = 2

    rep_counter_helper(){};
    void Prepossessing(int16_t ax_raw, int16_t ay_raw, int16_t az_raw, int16_t gx_raw, int16_t gy_raw, int16_t gz_raw); //gx,gy = 0
    void UpdateAccReference(float curr_time); //select acc reference
    void UpdateOrientation(float elapsedTime);
    void UpdateGyroReference(float curr_time);
    void UpdateGyroWindow();
    bool ResetCheck(float curr_time);
    int AngularMotion(float curr_time);
    bool VerticalMotion(float curr_time);
    bool CheckStationary(bool isAngular);

private:

    //window
    std::deque<int32_t> AccxWindow; //x-axis reduce noise
    std::deque<int32_t> AccyWindow; //y-axis reduce noise
    std::deque<int32_t> AccxReferenceWindow; //x-axis reference point window
    std::deque<int32_t> AccyReferenceWindow; //y-axis reference point window
    std::deque<int32_t> GyrozWindow; //z direction gyro window

    //data variable
    int32_t AccX, AccY; //scaled x-axis acceleration and y-axis acceleration
    float AccZ;
    float GyroX, GyroY, GyroZ;

    //vertical motion private variable
    int AccxRef = 0; //x-axis reference point
    int AccyRef = 0; //y-axis reference point
    int AccxRn = 0; //x-axis reduce noise point
    int AccyRn = 0; ///y-axis reduce noise point
    int AccFinal = 0; //final chosen acc value between AccxRn and AccyRn
    int AccRefFinal = 0; //final chosen value between AccxRef and AccyRef
    int AccAxis = 0; //choose the more dominant axis between x and y
    float AccUpdateTimer = 0; //timer for reference line update

    int VertFirstPeakDirection = 0; //up = 1, down = 2
    float VertFirstPeakTime = 0; //time of the first peak
    float VertSecondPeakTime = 0; //time of the second peak
    bool VertFirstPeakFlag; //first peak checker flag
    bool VertSecondPeakFlag; //second peak checker flag
    bool VertTroughFlag; //trough checker flag
    bool VertDataChangeFlag; //signal for change ref data
    bool VertRepTimeout; //timeout to reset checker flag


    //angular motion private variable
    float GyroAngleX; //x-axis orientation
    float GyroAngleY;  //y-axis orientation
    float GyroAngleZ; //z-axis orientation
    float GyrozRefValue = 0; //z-axis gyro reference value
    float AngularPeakTime, AngularTroughTime; //peak and trough time
    float GyroUpdateTimer = 0; //time for gyro reference line update
    bool AngularPeakFlag, AngularTroughFlag;

    //private functions
    void AccelerationRN();
    void UpdateAccWindow();
    float GetAverage(std::deque<int32_t> window);
    int GetDirection();


};
#endif //REP_COUNTER_HELPER_H_