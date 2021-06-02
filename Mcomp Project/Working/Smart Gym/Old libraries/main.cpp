#include "mbed.h"
#include "LSM9DS1.h"

LSM9DS1 imu(PF_0,PF_1);

DigitalOut pwron2(PF_4);
DigitalOut pwron1(PG_15);
DigitalOut shutTof(PG_1);
DigitalOut pwronTof(PF_5);

int main(){
    pwron2 = 1;
    pwron1 = 1;
    shutTof = 1;
    pwronTof = 1;
    // printf("pwron1 is:%d\r\n", pwron1.is_connected());
    // printf("pwron1 read:%d\r\n", pwron1.read());
    // printf("pwron2 is :%d\r\n", pwron2.is_connected());
    // printf("pwron2 read:%d\r\n", pwron2.read());
    // // printf("stis:%d\r\n", shutTof.is_connected());
    // printf("str:%d\r\n", shutTof.read());
    // printf("ptis:%d\r\n", pwronTof.is_connected());
    // printf("ptr:%d\r\n", pwronTof.read());

    int status = imu.begin();
    printf("Status: %d", status);
    //imu.calibrate();
    // int ready = imu.accelAvailable();
    // printf(" Ready : %d\n", ready);
    // if (!imu.begin()) {
        printf("Failed to communicate with LSM9DS1.\n");
    //}
    while(1){
        imu.readAccel();
        wait_us(1000);
        //imu.readMag();
        //imu.readGyro();      
        //pc.printf("%d %d %d %d %d %d %d %d %d\n\r", lol.calcGyro(lol.gx), lol.calcGyro(lol.gy), lol.calcGyro(lol.gz), lol.ax, lol.ay, lol.az, lol.mx, lol.my, lol.mz);
        //pc.printf("%d %d %d\n\r", lol.calcGyro(lol.gx), lol.calcGyro(lol.gy), lol.calcGyro(lol.gz));
        //printf("gyro: %d %d %d\n\r", int(imu.gx), int(imu.gy), int(imu.gz));
        //printf("accel: %d %d %d\n\r", int(imu.ax*10000), int(imu.ay*10000), int(imu.az*10000));
        //printf("mag: %d %d %d\n\n\r", int(imu.mx), int(imu.my), int(imu.mz));
        //wait_us(1000000);
        //printf("%d %d %d\n", imu.ax_raw, imu.ay_raw, imu.az_raw);
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
        
    }
}