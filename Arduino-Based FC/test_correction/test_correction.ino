#include <MPU6050.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <LowPower.h>

MPU6050 mpu;
Servo A,B,C,D;

const int Min=1000, Max=2000, ch1=8, ch2=9, ch3=10, ch4=11, ch5=12, ch6=13;
float axx, ayy, azz, ang, x, y;
int channel1, channel2, channel3, channel4, channel5, channel6, revs, trim, pitch, roll, yaw, mag, ax, ay, az;

void setup() {
  Serial.begin(9600);
  pinMode(ch1,INPUT);
  pinMode(ch2,INPUT);
  pinMode(ch3,INPUT);
  pinMode(ch4,INPUT);
  pinMode(ch5,INPUT);
  pinMode(ch6,INPUT);
  A.attach(2, Min, Max);
  B.attach(3, Min, Max);
  C.attach(4, Min, Max);
  D.attach(5, Min, Max);

  A.writeMicroseconds(Min);
  B.writeMicroseconds(Min);
  C.writeMicroseconds(Min);
  D.writeMicroseconds(Min);

  mpu.initialize();
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
}

void loop() {
  channel5=pulseIn(ch5,HIGH); //Serial.println(channel5);
  channel6=pulseIn(ch6,HIGH); //Serial.println(channel6); Serial.println();
  if (channel5>1900&&channel6<1900){
    fly();
  }
  else if (channel5>1900&&channel6>1900) {
    A.writeMicroseconds(1300);
    B.writeMicroseconds(1300);
    C.writeMicroseconds(1300);
    D.writeMicroseconds(1300);
  }
  else {
    A.writeMicroseconds(Min);
    B.writeMicroseconds(Min);
    C.writeMicroseconds(Min);
    D.writeMicroseconds(Min);
    delay(50);
    LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
    }
}

void fly() {
  int revsA, revsB, revsC, revsD;
  
  //Serial.print(pitch); Serial.print("\t");
  //Serial.print(roll); Serial.print("\t");
  //Serial.print(yaw); Serial.print("\t"); Serial.println();

  channel1=pulseIn(ch1,HIGH); //Serial.println(channel1);
  channel2=pulseIn(ch2,HIGH); //Serial.println(channel2);
  channel3=pulseIn(ch3,HIGH); //Serial.println(channel3);
  channel4=pulseIn(ch4,HIGH); //Serial.println(channel4); Serial.println();

  if(channel1>1550||channel1<1450||channel2>1550||channel2<1450||channel6<1100){
    revsA=revos(1);
    revsB=revos(2)-10;
    revsC=revos(3);
    revsD=revos(4)+10;
    //Serial.println("No Stab");
  }
  else{
    revsA=correction(1);
    revsB=correction(2)-10;
    revsC=correction(3);
    revsD=correction(4)+10;
    //Serial.println("Stab");
  }

  A.writeMicroseconds(revsA); //Serial.println(revos(1));
  B.writeMicroseconds(revsB); //Serial.println(revos(2));
  C.writeMicroseconds(revsC); //Serial.println(revos(3));
  D.writeMicroseconds(revsD); //Serial.println(revos(4));
  //delay(100);
}

int revos(int motor) {
  switch(motor){
    case 1:
      revs=channel3+map(channel2,1000,2000,200,-200)+map(channel1,1000,2000,-200,200);
      return revs;
    case 2:
      revs=channel3+map(channel2,1000,2000,200,-200)+map(channel1,1000,2000,200,-200);
      return revs;
    case 3:
      revs=channel3+map(channel2,1000,2000,-200,200)+map(channel1,1000,2000,200,-200);
      return revs;
    case 4:
      revs=channel3+map(channel2,1000,2000,-200,200)+map(channel1,1000,2000,-200,200);
      return revs;
  }
  
}

int correction(int motor) {
  int avrgx, avrgy, avrgz, smooth=10;
  mpu.getAcceleration(&ax, &ay, &az);
  //mpu.getRotation(&gx, &gy, &gz);

  for (int i=1; i<=smooth; i++) {
    avrgx+=ax; avrgy+=ay; avrgz+=az;
  }

  axx=avrgx/smooth; ayy=avrgy/smooth; azz=avrgz/smooth;
  pitch=(axx/16384)*90; roll=(ayy/16384)*90; yaw=(azz/16384)*90;

  ang=atan2(roll,pitch);
  mag=sqrt(sq(pitch)+sq(roll));
  //Serial.println(mag);
  y=mag*cos(ang);
  x=mag*sin(ang);
  //Serial.println(ang); Serial.println(x); Serial.println(y); Serial.println();
  switch(motor){
    case 1:
      revs=channel3-y-x;
      return revs;
    case 2:
      revs=channel3-y+x;
      return revs;
    case 3:
      revs=channel3+y+x;
      return revs;
    case 4:
      revs=channel3+y-x;
      return revs;
  }
}