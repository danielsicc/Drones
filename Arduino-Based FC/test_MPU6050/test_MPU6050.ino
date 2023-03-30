#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;
int x, y, z;
int a, b, c;
float axx, ayy, azz;
float gxx, gyy, gzz;
int ax, ay, az;
int gx, gy, gz;

void setup() {
    Serial.begin(38400);
    mpu.initialize();
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
}

void loop() {
    //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    axx=ax;
    ayy=ay;
    azz=az;
    gxx=gx;
    gyy=gy;
    gzz=gz;
    x=(axx/16384)*90;
    y=(ayy/16384)*90;
    z=(azz/16384)*90;
    a=(gxx/500);
    b=(gyy/500);
    c=(gzz/500);

        Serial.print(x); Serial.print("\t");
        Serial.print(y); Serial.print("\t");
        Serial.print(z); Serial.print("\t");
        Serial.print(a); Serial.print("\t");
        Serial.print(b); Serial.print("\t");
        Serial.println(c);

        delay(300);
}