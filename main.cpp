#include "mbed.h"
#include "LSM9DS1.h"
#include "mbed_power_mgmt.h"
#include "mbed_wait_api.h"
#include "medianfilter.h"
#include "ns_timer.h"
#include <algorithm>

#include <cstdio>
#include <math.h>
#include <time.h>
#include <vector>
#include <string>
//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	512	// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain
#define shift_bits 9
#define sft 15

#define mul_fr32_c(inArg0, inArg1)\
({\
int32_t A = (inArg0 >> 16); \
int32_t C = (inArg1 >> 16); \
uint32_t B = (inArg0 & 0xFFFF); \
uint32_t D = (inArg1 & 0xFFFF); \
int32_t AC = A*C; \
int32_t AD_CB = A*D + C*B; \
uint32_t BD = B*D; \
int32_t product_hi = AC + (AD_CB >> 16); \
uint32_t ad_cb_temp = AD_CB << 16; \
uint32_t product_lo = BD + ad_cb_temp; \
if (product_lo < BD) \
		product_hi++; \
int32_t result = (product_hi << (32-sft)) | (product_lo >> sft); \
result; \
})

#define mul_fr32(inArg0, inArg1)\
({\
int32_t result = (inArg0 * inArg1) >> sft; \
result; \
})

#define rshift_fr32(inArg)\
({\
int32_t result = inArg >> sft; \
result; \
})

//---------------------------------------------------------------------------------------------------
// Variable definitions
volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
volatile float q_0 = 1.0f, q_1 = 0.0f, q_2 = 0.0f, q_3 = 0.0f;
volatile float m_q0 = 1.0f, m_q1 = 0.0f, m_q2 = 0.0f, m_q3 = 0.0f;
volatile int length = 1;
// Add Madgwick function prototype
float invSqrt(float x);
void computeAngles();
void computeAngles_mod();
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
// int MadgwickAHRSupdateIMU_mod(float gx, float gy, float gz, float ax, float ay, float az);
int MadgwickAHRSupdateIMU_mod(float *gxf, float *gyf, float *gzf, float *axf, float *ayf, float *azf, int length);


//====================================================================================================
// Functions

int32_t float_to_fr32(float real){
	return ((int32_t)((1U << sft) * (real)));
}

float fr32_to_float(int32_t fr32){
	return ((float)fr32 * 1.0F / (1U << sft));
}


LSM9DS1 imu(PF_0,PF_1);
DigitalOut pwron2(PF_4);
DigitalOut pwron1(PG_15);
DigitalOut shutTof(PG_1);
DigitalOut pwronTof(PF_5);

int max();

int16_t gyroz_window[200];

float roll;
float pitch;
float yaw;
char anglesComputed;
void computeAngles();   

bool peak = false;
bool trough = false;
bool update = false; 
int8_t reps = 0;

int max()
{
    int temp_max = 0;
    for (int i =0 ; i <= 200; i++ )
    {
        if (gyroz_window[i] > temp_max)
            temp_max =  gyroz_window[i];
    }
    return temp_max;
}

int main(){
    pwron2 = 1;
    pwron1 = 1;
    shutTof = 1;

    pwronTof = 1;
    int8_t aO;
    int8_t samples2 = 0;
    int8_t count = 0;
    int8_t reps = 0; //repeated?
    int16_t gFinal[3], aFinal[3];
    element yfilteredinput[10], yfilteredoutput[10] = {0};
    element xfilteredinput[10], xfilteredoutput[10] = {0};
    element zfilteredinput[10], zfilteredoutput[10] = {0};
    int16_t gyroz_window[200];
    bool peak = false;
    bool trough = false;
    int16_t gyro_z = 0;
    bool update = false;
    
    printf("pwron1 is:%d\r\n", pwron1.is_connected());
    printf("pwron1 read:%d\r\n", pwron1.read());
    printf("pwron2 is :%d\r\n", pwron2.is_connected());
    printf("pwron2 read:%d\r\n", pwron2.read());
    printf("stis:%d\r\n", shutTof.is_connected());
    printf("str:%d\r\n", shutTof.read());
    printf("ptis:%d\r\n", pwronTof.is_connected());
    printf("ptr:%d\r\n", pwronTof.read());

    imu.begin();

    if (!imu.begin()) {
        printf("Failed to communicate with LSM9DS1.\n");

    }
    else{
        printf("Communicating\n");
    }
    imu.calibration();
    // printf("Calibration Done!");
    count = 0;
    int gz_count = 0;   

    float gxf[length], gyf[length],gzf[length],axf[length], ayf[length], azf[length];
    
    while(1){
        //while(samples2 < 10){        
        imu.readAccel();
        imu.readGyro();
        // imu.readMag(); 
        // wait_ns(5511000);
        // printf("%d %d %d %d %d %d\n", imu.gx_raw, imu.gy_raw, imu.gz_raw,imu.ax_raw,imu.ay_raw, imu.az_raw);
        // printf("%d %d %d %d\n", imu.gz_raw,imu.ax_raw,imu.ay_raw, imu.az_raw);
        // Timer t;
        // t.start();
        // for (int j = 1; j<10000; j++){
            // MadgwickAHRSupdateIMU(imu.gx_raw, imu.gy_raw, imu.gz_raw,imu.ax_raw,imu.ay_raw, imu.az_raw);
        // }
        // t.stop();
        // printf("Ori: 10000 iteration is: %llu\n", t.read_high_resolution_us());
    
        gxf[count] = (float)imu.gx_raw;
        gyf[count] = (float)imu.gy_raw;
        gzf[count] = (float)imu.gz_raw;
        axf[count] = (float)imu.ax_raw;
        ayf[count] = (float)imu.ay_raw;
        azf[count] = (float)imu.az_raw;
        if (count == length-1){
            // printf("Algorithm Start\n");
            count = 0;
            // Timer t1;
            // t1.start();
            // for (int j = 1; j<10000; j++){
            MadgwickAHRSupdateIMU_mod(gxf, gyf, gzf, axf, ayf, azf, length);
            // }
            // t1.stop();
            // printf("Mod 1000 iteration is: %llu\n", t1.read_high_resolution_us());
        }
        else{
            count ++;
            //printf("%d\n", count);
        }
        
        /* Working */
        if (gz_count < 200){
            gyroz_window[gz_count++] = abs(imu.gz_raw); //???What is gyroz window for?

        }
        else{
            gz_count=0;
        }
       
        /* Working */
        aFinal[0] = imu.ax_raw / 8;
        aFinal[1] = imu.ay_raw / 8;
        aFinal[2] = imu.az_raw / 8;
        xfilteredinput[samples2++] = aFinal[0];
        yfilteredinput[samples2++] = aFinal[1];
        zfilteredinput[samples2++] = aFinal[2];
        //printf("%d %d %d\n", aFinal[0],aFinal[1],aFinal[2]);            
        
        samples2 = 0;

        /* Working */
        gyro_z = abs(imu.gz_raw); 
    
        //printf("Bench Press!");
        // // /* Calculate Peak and troughs */
        // Bench press exercise
        // if(aFinal[1] >= 800  && aFinal[2]<1500 && gyro_z <5000){
        //     peak = true;
        //     update = true; 
        //     // printf("Get Peak\n");
        // }
        // else if(aFinal[1] <= -800 && aFinal[2]>-1500 && gyro_z <5000){
        //     trough = true;
        //     update = false;
        //     // printf("Get Trough\n");
        // }

        // if(peak && trough && update){
        //     peak = false;
        //     trough = false;
        //     update = false;
        //     reps += 1;
        //     printf("Reps: %d\n", reps);
        //     // printf("Reps: %d %d\n", reps, gyro_z);
        // }
            
        //imu.readGyro();
        /* // Testing: Test to see if Returns ACK
        i2c.start();
        int result1 = i2c.write(0x6B<<1, cmd, 1);
        printf("Result from i2c_write : %d\r\n", result1);
        i2c.stop();
        wait_us(1000);
        i2c.start();
        int result2 = i2c.read(0x6B<<1, cmd+1, 1);
        printf("Result from i2c_Read : %d\r\n", result2);
        i2c.stop();
        */ 
    // }
    }
}

// //---------------------------------------------------------------------------------------------------
// // IMU algorithm update
// // Original Algorithm

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
     Timer t2;
    t2.start();
    for (int j = 1; j<10000; j++){
    // Timer t0;
    // t0.start();
    // for (int j = 1; j<10000; j++){
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   
    // }
    // t0.stop();
    // printf("Ori Before Accelerable Part: 10000 iteration is: %llu\n", t0.read_high_resolution_us());

    // Timer t;
    // t.start();
    // for (int j = 1; j<10000; j++){

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	// if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		// recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		// ax *= recipNorm;
		// ay *= recipNorm;
		// az *= recipNorm;   
		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;
    
		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	// }

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);
    // }
    // t.stop();
    // printf("Ori Accelerable Part: 10000 iteration is: %llu\n", t.read_high_resolution_us());

    // Timer t2;
    // t2.start();
    // for (int j = 1; j<10000; j++){
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
    anglesComputed = 0;
    }
    t2.stop();
    // printf("Ori After Accelerable Part: 10000 iteration is: %llu\n", t2.read_high_resolution_us());
    
    printf("Ori w\\o function call: 10000 iteration is: %llu\n", t2.read_high_resolution_us());
    // computeAngles();
    // char sing0 = q0>0?' ':'-';
    // char sing1 = q1>0?' ':'-';
    // char sing2 = q2>0?' ':'-';
    // char sing3 = q3>0?' ':'-';
    // printf("Ori: %c%f %c%f %c%f %c%f\n", sing0, abs(q0), sing1, abs(q1), sing2, abs(q2), sing3, abs(q3)); 
    // printf("ori: %f %f %f %f\n", q0,q1,q2,q3);
}

int MadgwickAHRSupdateIMU_mod(float *gxf, float *gyf, float *gzf, float *axf, float *ayf, float *azf, int length ) {
	float recipNorm;
	int _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _4q3, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3; 
	int qDot1, qDot2, qDot3, qDot4;
	int s0, s1, s2, s3;
	int gx[length], gy[length], gz[length], ax[length], ay[length], az[length];
	int reverse_1 = 0x55555555;
	int reverse_2 = 0x33333333; 
	int reverse_3 = 0x0F0F0F0F;
	int reverse_4 = 0x00FF00FF;
	int mediate = 10;
	// unsigned int logical_number = 0x077CB531U;
	// static const int MultiplyDeBruijnBitPosition[32] = {0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17, 4, 8, 31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9};
	unsigned int logical_number = 0x07C4ACDDU;
	// static int MultiplyDeBruijnBitPosition[32] = {0, 9, 1, 10, 13, 21, 2, 29, 11, 14, 16, 18, 22, 25, 3, 30, 8, 12, 20, 28, 15, 17, 24, 7, 19, 27, 23, 6, 26, 5, 4, 31};
	static int MultiplyDeBruijnBitPosition[32] = {-15, -6, -14, -5, -2, 6, -13, 14, -4, -1, 1, 3, 7, 10, -12, 15, -7, -3, 5, 13, 0, 2, 9, -8, 4, 12, 8, -9, 11, -10, -11, 16};
	// for(int i=0; i<32; i++){
	// 	MultiplyDeBruijnBitPosition[i] = MultiplyDeBruijnBitPosition[i] - sft;
	// }

	int q0_int ;
	int q1_int;
	int q2_int;
	int q3_int;
    
    float tax = ax[0];
    float tay = ay[0];
    float taz = az[0];
    float tgx = gx[0], tgy=gy[0], tgz =gz[0];

	for(int iter=0; iter<length; iter++){
        q0_int = float_to_fr32(m_q0);
        q1_int = float_to_fr32(m_q1);
        q2_int = float_to_fr32(m_q2);
        q3_int = float_to_fr32(m_q3);
		recipNorm = invSqrt(axf[iter] * axf[iter] + ayf[iter] * ayf[iter] + azf[iter] * azf[iter]);
		axf[iter] *= recipNorm;
		ayf[iter] *= recipNorm;
		azf[iter] *= recipNorm;   
		ax[iter] = float_to_fr32(axf[iter]);
		ay[iter] = float_to_fr32(ayf[iter]);
		az[iter] = float_to_fr32(azf[iter]);

        gxf[iter] *= 0.0174533f * 5;
        gyf[iter] *= 0.0174533f * 5;
        gzf[iter] *= 0.0174533f * 5;

		gx[iter] = (int)gxf[iter];
		gy[iter] = (int)gyf[iter];
		gz[iter] = (int)gzf[iter];

    }

       for (int i=0; i<length; i++){
		int recipNorm_second;
		// Rate of change of quaternion from gyroscope
		qDot1 = (-q1_int * gx[i] - q2_int * gy[i] - q3_int * gz[i]);
		qDot2 = (q0_int * gx[i] + q2_int * gz[i] - q3_int * gy[i]);
		qDot3 = (q0_int * gy[i] - q1_int * gz[i] + q3_int * gx[i]);
		qDot4 = (q0_int * gz[i] + q1_int * gy[i] - q2_int * gx[i]);

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2 * q0_int;
		_2q1 = 2 * q1_int;
		_2q2 = 2 * q2_int;
		_2q3 = 2 * q3_int;
		_4q0 = 4 * q0_int;
		_4q1 = 4 * q1_int;
		_4q2 = 4 * q2_int;
		_4q3 = 4 * q3_int;
		_8q1 = 8 * q1_int;
		_8q2 = 8 * q2_int;
		q0q0 = mul_fr32(q0_int, q0_int);
		q1q1 = mul_fr32(q1_int, q1_int);
		q2q2 = mul_fr32(q2_int, q2_int);
		q3q3 = mul_fr32(q3_int, q3_int);

	    // Gradient decent algorithm corrective step
		// s0 = mul_fr32(_4q0, q2q2)  + mul_fr32(_2q2, ax[i]) + rshift_fr32(_4q0 * q1q1 - _2q1 * ay[i]);
		// s1 = rshift_fr32(_4q1 * q3q3 - _2q3 * ax[i] ) + rshift_fr32(q0q0 * _4q1 - _2q0 * ay[i]) - _4q1 + mul_fr32(_8q1, q1q1) + mul_fr32(_8q1, q2q2) + mul_fr32(_4q1, az[i]);
		// s2 = mul_fr32(q0q0, _4q2) + mul_fr32(_2q0, ax[i]) + rshift_fr32(_4q2 * q3q3 - _2q3 * ay[i])  - _4q2 + mul_fr32(_8q2, q1q1) + mul_fr32(_8q2, q2q2) + mul_fr32(_4q2, az[i]);
		// s3 = rshift_fr32(q1q1 * _4q3 - _2q1 * ax[i]) + rshift_fr32(q2q2 * _4q3 - _2q2 * ay[i]);
        s0 = _4q0 * q2q2 + _2q2 * ax[i] + _4q0 * q1q1 - _2q1 * ay[i];
        s1 = _4q1 * q3q3 - _2q3 * ax[i] + q0q0 * _4q1 - _2q0 * ay[i] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az[i];
        s2 = q0q0 * _4q2 + _2q0 * ax[i] + _4q2 * q3q3 - _2q3 * ay[i] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az[i];
        s3 = q1q1 * _4q3 - _2q1 * ax[i] + q2q2 * _4q3 - _2q2 * ay[i];
        
        s0 >>= sft;
        s1 >>= sft;
        s2 >>= sft;
        s3 >>= sft;
		// normalise step magnitude
		// recipNorm_second = FastDistance2D(FastDistance2D(s0, s1), FastDistance2D(s2, s3));
		int abs_s0 = ((s0^(s0>>31))-(s0>>31));//opt
		int abs_s1 = ((s1^(s1>>31))-(s1>>31)); //opt
		int min_s = abs_s0+(((abs_s1-abs_s0)>>(31))&(abs_s1-abs_s0)); 
		int abs_ss0 = (abs_s0+abs_s1-(min_s>>1)-(min_s>>2)+(min_s>>4));

		int abs_s2 = ((s2^(s2>>31))-(s2>>31));
		int abs_s3 = ((s3^(s3>>31))-(s3>>31));	
		int min_ss = abs_s2+(((abs_s3-abs_s2)>>(31))&(abs_s3-abs_s2)); 
		int abs_ss1 = (abs_s2+abs_s3-(min_ss>>1)-(min_ss>>2)+(min_ss>>4));

		int min_sss = abs_ss0+(((abs_ss1-abs_ss0)>>(31))&(abs_ss1-abs_ss0)); 
		recipNorm_second = (abs_ss0+abs_ss1-(min_sss>>1)-(min_sss>>2)+(min_sss>>4));

		// //Binary reverse
		// recipNorm_second =((recipNorm_second >>1)&reverse_1)|((recipNorm_second&reverse_1)<<1);
		// recipNorm_second =((recipNorm_second >>2)&reverse_2)|((recipNorm_second&reverse_2)<<2);
		// recipNorm_second =((recipNorm_second >>4)&reverse_3)|((recipNorm_second&reverse_3)<<4);
		// recipNorm_second =((recipNorm_second >>8)&reverse_4)|((recipNorm_second&reverse_4)<<8);
		// recipNorm_second =(recipNorm_second >>16)|(recipNorm_second <<16);
		// //Count the consecutive zero bits (trailing) on the right with multiply and lookup
		// recipNorm_second = 17 - MultiplyDeBruijnBitPosition[(((recipNorm_second & -recipNorm_second) * logical_number)) >> 27];
		recipNorm_second |= (recipNorm_second >> 1); 
		recipNorm_second |=  (recipNorm_second >> 2);
		recipNorm_second |=  (recipNorm_second >> 4);
		recipNorm_second |=  (recipNorm_second >> 8);
		recipNorm_second |=  (recipNorm_second >> 16); 
		recipNorm_second = MultiplyDeBruijnBitPosition[(recipNorm_second * logical_number) >> 27];
		//Adjust value; avoid overflow
		// sft: the number of bits to represent the decimal part
		// recipNorm_second -= sft;
		s0 >>= recipNorm_second;
		s1 >>= recipNorm_second;
		s2 >>= recipNorm_second;
		s3 >>= recipNorm_second;	

		// Apply feedback step
		qDot1 -= s0;
		qDot2 -= s1;
		qDot3 -= s2;
		qDot4 -= s3;
		// Integrate rate of change of quaternion to yield quaternion
		q0_int =  q0_int * mediate + (qDot1 >> shift_bits);
		q1_int =  q1_int * mediate + (qDot2 >> shift_bits);
		q2_int =  q2_int * mediate + (qDot3 >> shift_bits);
		q3_int =  q3_int * mediate + (qDot4 >> shift_bits);

	    //	recipNorm_second = FastDistance2D(FastDistance2D(q0_int, q1_int), FastDistance2D(q2_int, q3_int));	
		abs_s0 = ((q0_int^(q0_int>>31))-(q0_int>>31));
		abs_s1 = ((q1_int^(q1_int>>31))-(q1_int>>31));
		min_s = abs_s0+(((abs_s1-abs_s0)>>(31))&(abs_s1-abs_s0)); 
		abs_ss0 = (abs_s0+abs_s1-(min_s>>1)-(min_s>>2)+(min_s>>4));

		abs_s2 = ((q2_int^(q2_int>>31))-(q2_int>>31));
		abs_s3 = ((q3_int^(q3_int>>31))-(q3_int>>31));
		min_ss = abs_s2+(((abs_s3-abs_s2)>>(31))&(abs_s3-abs_s2)); 
		abs_ss1 = (abs_s2+abs_s3-(min_ss>>1)-(min_ss>>2)+(min_ss>>4));

		min_sss = abs_ss0+(((abs_ss1-abs_ss0)>>(31))&(abs_ss1-abs_ss0)); 
		recipNorm_second = (abs_ss0+abs_ss1-(min_sss>>1)-(min_sss>>2)+(min_sss>>4));	
		// Normalise quaternion
		// //Binary reverse
		// recipNorm_second =((recipNorm_second >>1)&reverse_1)|((recipNorm_second&reverse_1)<<1);
		// recipNorm_second =((recipNorm_second >>2)&reverse_2)|((recipNorm_second&reverse_2)<<2);
		// recipNorm_second =((recipNorm_second >>4)&reverse_3)|((recipNorm_second&reverse_3)<<4);
		// recipNorm_second =((recipNorm_second >>8)&reverse_4)|((recipNorm_second&reverse_4)<<8);
		// recipNorm_second =(recipNorm_second >>16)|(recipNorm_second <<16);
		// //Count the consecutive zero bits (trailing) on the right with multiply and lookup
		// recipNorm_second = 17 - MultiplyDeBruijnBitPosition[(((recipNorm_second & -recipNorm_second) * logical_number)) >> 27];
		recipNorm_second |= (recipNorm_second >> 1); 
		recipNorm_second |=  (recipNorm_second >> 2);
		recipNorm_second |=  (recipNorm_second >> 4);
		recipNorm_second |=  (recipNorm_second >> 8);
		recipNorm_second |=  (recipNorm_second >> 16); 
		recipNorm_second = MultiplyDeBruijnBitPosition[(recipNorm_second * logical_number) >> 27];
		
		//Adjust the value of quaternion; avoid overflow
		// sft: the number of bits to represent the decimal part
		// recipNorm_second -= sft;
		q0_int >>= recipNorm_second;
		q1_int >>= recipNorm_second;
		q2_int >>= recipNorm_second;
		q3_int >>= recipNorm_second;	

		// q0 = fr32_to_float(q0_int);
		// q1 = fr32_to_float(q1_int);
		// q2 = fr32_to_float(q2_int);
		// q3 = fr32_to_float(q3_int);	
		// recipNorm = invSqrt((q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3));
		// printf("q0: %f; q1: %f; q2: %f; q3: %f\n", q0*recipNorm, q1*recipNorm, q2*recipNorm, q3*recipNorm);

	}
    
    

	// for (int i=0; i<length; i++){
	// 	int recipNorm_second;
	// 	// Rate of change of quaternion from gyroscope
	// 	qDot1 = (-q1_int * gx[i] - q2_int * gy[i] - q3_int * gz[i]);
	// 	qDot2 = (q0_int * gx[i] + q2_int * gz[i] - q3_int * gy[i]);
	// 	qDot3 = (q0_int * gy[i] - q1_int * gz[i] + q3_int * gx[i]);
	// 	qDot4 = (q0_int * gz[i] + q1_int * gy[i] - q2_int * gx[i]);

	// 	// Auxiliary variables to avoid repeated arithmetic
	// 	_2q0 = 2 * q0_int;
	// 	_2q1 = 2 * q1_int;
	// 	_2q2 = 2 * q2_int;
	// 	_2q3 = 2 * q3_int;
	// 	_4q0 = 4 * q0_int;
	// 	_4q1 = 4 * q1_int;
	// 	_4q2 = 4 * q2_int;
	// 	_4q3 = 4 * q3_int;
	// 	_8q1 = 8 * q1_int;
	// 	_8q2 = 8 * q2_int;
	// 	q0q0 = mul_fr32(q0_int, q0_int);
	// 	q1q1 = mul_fr32(q1_int, q1_int);
	// 	q2q2 = mul_fr32(q2_int, q2_int);
	// 	q3q3 = mul_fr32(q3_int, q3_int);

	// // Gradient decent algorithm corrective step
	// 	// s0 = mul_fr32(_4q0, q2q2)  + mul_fr32(_2q2, ax[i]) + rshift_fr32(_4q0 * q1q1 - _2q1 * ay[i]);
	// 	// s1 = rshift_fr32(_4q1 * q3q3 - _2q3 * ax[i] ) + rshift_fr32(q0q0 * _4q1 - _2q0 * ay[i]) - _4q1 + mul_fr32(_8q1, q1q1) + mul_fr32(_8q1, q2q2) + mul_fr32(_4q1, az[i]);
	// 	// s2 = mul_fr32(q0q0, _4q2) + mul_fr32(_2q0, ax[i]) + rshift_fr32(_4q2 * q3q3 - _2q3 * ay[i])  - _4q2 + mul_fr32(_8q2, q1q1) + mul_fr32(_8q2, q2q2) + mul_fr32(_4q2, az[i]);
	// 	// s3 = rshift_fr32(q1q1 * _4q3 - _2q1 * ax[i]) + rshift_fr32(q2q2 * _4q3 - _2q2 * ay[i]);
    //     s0 = _4q0 * q2q2 + _2q2 * ax[i] + _4q0 * q1q1 - _2q1 * ay[i];
    //     s1 = _4q1 * q3q3 - _2q3 * ax[i] + q0q0 * _4q1 - _2q0 * ay[i] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az[i];
    //     s2 = q0q0 * _4q2 + _2q0 * ax[i] + _4q2 * q3q3 - _2q3 * ay[i] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az[i];
    //     s3 = q1q1 * _4q3 - _2q1 * ax[i] + q2q2 * _4q3 - _2q2 * ay[i];
        
    //     s0 >>= sft;
    //     s1 >>= sft;
    //     s2 >>= sft;
    //     s3 >>= sft;
	// 	// normalise step magnitude
	// 	// recipNorm_second = FastDistance2D(FastDistance2D(s0, s1), FastDistance2D(s2, s3));
	// 	int abs_s0 = ((s0^(s0>>31))-(s0>>31));//opt
	// 	int abs_s1 = ((s1^(s1>>31))-(s1>>31)); //opt
	// 	int min_s = abs_s0+(((abs_s1-abs_s0)>>(31))&(abs_s1-abs_s0)); 
	// 	int abs_ss0 = (abs_s0+abs_s1-(min_s>>1)-(min_s>>2)+(min_s>>4));

	// 	int abs_s2 = ((s2^(s2>>31))-(s2>>31));
	// 	int abs_s3 = ((s3^(s3>>31))-(s3>>31));	
	// 	int min_ss = abs_s2+(((abs_s3-abs_s2)>>(31))&(abs_s3-abs_s2)); 
	// 	int abs_ss1 = (abs_s2+abs_s3-(min_ss>>1)-(min_ss>>2)+(min_ss>>4));

	// 	int min_sss = abs_ss0+(((abs_ss1-abs_ss0)>>(31))&(abs_ss1-abs_ss0)); 
	// 	recipNorm_second = (abs_ss0+abs_ss1-(min_sss>>1)-(min_sss>>2)+(min_sss>>4));

	// 	// //Binary reverse
	// 	// recipNorm_second =((recipNorm_second >>1)&reverse_1)|((recipNorm_second&reverse_1)<<1);
	// 	// recipNorm_second =((recipNorm_second >>2)&reverse_2)|((recipNorm_second&reverse_2)<<2);
	// 	// recipNorm_second =((recipNorm_second >>4)&reverse_3)|((recipNorm_second&reverse_3)<<4);
	// 	// recipNorm_second =((recipNorm_second >>8)&reverse_4)|((recipNorm_second&reverse_4)<<8);
	// 	// recipNorm_second =(recipNorm_second >>16)|(recipNorm_second <<16);
	// 	// //Count the consecutive zero bits (trailing) on the right with multiply and lookup
	// 	// recipNorm_second = 17 - MultiplyDeBruijnBitPosition[(((recipNorm_second & -recipNorm_second) * logical_number)) >> 27];
	// 	recipNorm_second |= (recipNorm_second >> 1); 
	// 	recipNorm_second |=  (recipNorm_second >> 2);
	// 	recipNorm_second |=  (recipNorm_second >> 4);
	// 	recipNorm_second |=  (recipNorm_second >> 8);
	// 	recipNorm_second |=  (recipNorm_second >> 16); 
	// 	recipNorm_second = MultiplyDeBruijnBitPosition[(recipNorm_second * logical_number) >> 27];
	// 	//Adjust value; avoid overflow
	// 	// sft: the number of bits to represent the decimal part
	// 	// recipNorm_second -= sft;
	// 	s0 >>= recipNorm_second;
	// 	s1 >>= recipNorm_second;
	// 	s2 >>= recipNorm_second;
	// 	s3 >>= recipNorm_second;	

	// 	// Apply feedback step
	// 	qDot1 -= s0;
	// 	qDot2 -= s1;
	// 	qDot3 -= s2;
	// 	qDot4 -= s3;
	// 	// Integrate rate of change of quaternion to yield quaternion
	// 	q0_int =  q0_int * mediate + (qDot1 >> shift_bits);
	// 	q1_int =  q1_int * mediate + (qDot2 >> shift_bits);
	// 	q2_int =  q2_int * mediate + (qDot3 >> shift_bits);
	// 	q3_int =  q3_int * mediate + (qDot4 >> shift_bits);

	// //	recipNorm_second = FastDistance2D(FastDistance2D(q0_int, q1_int), FastDistance2D(q2_int, q3_int));	
	// 	abs_s0 = ((q0_int^(q0_int>>31))-(q0_int>>31));
	// 	abs_s1 = ((q1_int^(q1_int>>31))-(q1_int>>31));
	// 	min_s = abs_s0+(((abs_s1-abs_s0)>>(31))&(abs_s1-abs_s0)); 
	// 	abs_ss0 = (abs_s0+abs_s1-(min_s>>1)-(min_s>>2)+(min_s>>4));

	// 	abs_s2 = ((q2_int^(q2_int>>31))-(q2_int>>31));
	// 	abs_s3 = ((q3_int^(q3_int>>31))-(q3_int>>31));
	// 	min_ss = abs_s2+(((abs_s3-abs_s2)>>(31))&(abs_s3-abs_s2)); 
	// 	abs_ss1 = (abs_s2+abs_s3-(min_ss>>1)-(min_ss>>2)+(min_ss>>4));

	// 	min_sss = abs_ss0+(((abs_ss1-abs_ss0)>>(31))&(abs_ss1-abs_ss0)); 
	// 	recipNorm_second = (abs_ss0+abs_ss1-(min_sss>>1)-(min_sss>>2)+(min_sss>>4));	
	// 	// Normalise quaternion
	// 	// //Binary reverse
	// 	// recipNorm_second =((recipNorm_second >>1)&reverse_1)|((recipNorm_second&reverse_1)<<1);
	// 	// recipNorm_second =((recipNorm_second >>2)&reverse_2)|((recipNorm_second&reverse_2)<<2);
	// 	// recipNorm_second =((recipNorm_second >>4)&reverse_3)|((recipNorm_second&reverse_3)<<4);
	// 	// recipNorm_second =((recipNorm_second >>8)&reverse_4)|((recipNorm_second&reverse_4)<<8);
	// 	// recipNorm_second =(recipNorm_second >>16)|(recipNorm_second <<16);
	// 	// //Count the consecutive zero bits (trailing) on the right with multiply and lookup
	// 	// recipNorm_second = 17 - MultiplyDeBruijnBitPosition[(((recipNorm_second & -recipNorm_second) * logical_number)) >> 27];
	// 	recipNorm_second |= (recipNorm_second >> 1); 
	// 	recipNorm_second |=  (recipNorm_second >> 2);
	// 	recipNorm_second |=  (recipNorm_second >> 4);
	// 	recipNorm_second |=  (recipNorm_second >> 8);
	// 	recipNorm_second |=  (recipNorm_second >> 16); 
	// 	recipNorm_second = MultiplyDeBruijnBitPosition[(recipNorm_second * logical_number) >> 27];
		
	// 	//Adjust the value of quaternion; avoid overflow
	// 	// sft: the number of bits to represent the decimal part
	// 	// recipNorm_second -= sft;
	// 	q0_int >>= recipNorm_second;
	// 	q1_int >>= recipNorm_second;
	// 	q2_int >>= recipNorm_second;
	// 	q3_int >>= recipNorm_second;	

	// 	// q0 = fr32_to_float(q0_int);
	// 	// q1 = fr32_to_float(q1_int);
	// 	// q2 = fr32_to_float(q2_int);
	// 	// q3 = fr32_to_float(q3_int);	
	// 	// recipNorm = invSqrt((q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3));
	// 	// printf("q0: %f; q1: %f; q2: %f; q3: %f\n", q0*recipNorm, q1*recipNorm, q2*recipNorm, q3*recipNorm);

	// }

	//Convert the quaternion into floating number and then normalise it
	m_q0 = fr32_to_float(q0_int);
	m_q1 = fr32_to_float(q1_int);
	m_q2 = fr32_to_float(q2_int);
	m_q3 = fr32_to_float(q3_int);		
	recipNorm = invSqrt((m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3));
	m_q0 *= recipNorm;
	m_q1 *= recipNorm;
	m_q2 *= recipNorm;
	m_q3 *= recipNorm;

    // char sing0 = m_q0>0?' ':'-';
    // char sing1 = m_q1>0?' ':'-';
    // char sing2 = m_q2>0?' ':'-';
    // char sing3 = m_q3>0?' ':'-';
    // printf("%c%f %c%f %c%f %c%f\n", sing0, abs(m_q0), sing1, abs(m_q1), sing2, abs(m_q2), sing3, abs(m_q3));
    // printf("%f, %f, %f, %f\n", m_q0, m_q1, m_q2, m_q3);
    computeAngles_mod();
	return 0;
}

void computeAngles()
{
	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	anglesComputed = 1;
    // printf("ori: %f %f %f \r\n",roll, pitch, yaw);
    // char singr = roll>0?' ':'-';
    // char singp = pitch>0?' ':'-';
    // char singy = yaw>0?' ':'-';
    // printf("Ori: %c%f %c%f %c%f\n", singr, abs(roll), singp, abs(pitch), singy, abs(yaw));
}
void computeAngles_mod()
{
	roll = atan2f(m_q0*m_q1 + m_q2*m_q3, 0.5f - m_q1*m_q1 - m_q2*m_q2);
	pitch = asinf(-2.0f * (m_q1*m_q3 - m_q0*m_q2));
	yaw = atan2f(m_q1*m_q2 + m_q0*m_q3, 0.5f - m_q2*m_q2 - m_q3*m_q3);
	anglesComputed = 1;

    char singr = roll>0?' ':'-';
    char singp = pitch>0?' ':'-';
    char singy = yaw>0?' ':'-';
    printf("Roll: %c%f Pitch: %c%f Yaw: %c%f\n", singr, abs(roll), singp, abs(pitch), singy, abs(yaw));

    // if ((abs(pitch-o_pitch)>1.5 & abs(pitch-o_pitch)<3) ^ (abs(yaw-o_yaw)>1.5 & abs(yaw-o_yaw)<3)){
    //     printf("New montion detected, Recount.\n");
    //     reps=0;
    // }

    if(roll< -1.3f && roll> -2.0f){
        peak = true;
        update = true; 
    }
    else if(roll > 0.5f && roll < 1.5f){
        trough = true;
        update = false;
    }
    if(peak && trough && update){
        peak = false;
        trough = false;
        update = false;
        reps += 1;
        std::string dir("");
        if (abs(pitch-yaw) <1){
            dir = "|";
        }
        else{
            dir = "--";
        }
        //printf("Reps: %d %s\n", reps, dir.c_str());
        // printf("Reps: %d\n", reps);
    }
    wait_us(10000);
}
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
    float halfx = 0.5f * x;  
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
