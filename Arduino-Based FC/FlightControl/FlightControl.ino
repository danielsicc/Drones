//#include <LowPower.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Servo.h>


MPU6050 mpu;
Servo A,B,C,D;

const int Min=1000, Max=2000, ch1=8, ch2=9, ch3=10, ch4=11, ch5=12, ch6=13, rateX=100, rateY=100,
     rateZ=50, CorrectP=2, smooth=15;
float axx, ayy, azz, ang, x, y, avrgx, avrgy, avrgz;
int channel1, channel2, channel3, channel4, channel5, channel6, revs, revsA, revsB, revsC, revsD,
     pitch, roll, yaw, mag, CorrectCh1, CorrectCh2, CorrectCh4;
int16_t	ax, ay, az;

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

  channel4=pulseIn(ch4,HIGH);
  channel2=pulseIn(ch2,HIGH);
  channel1=pulseIn(ch1,HIGH);
  if (channel1!=0&&channel2!=0&&channel4!=0){
    for(int i=1; i<smooth; i++){
      channel4+=pulseIn(ch4,HIGH);
      channel2+=pulseIn(ch2,HIGH);
      channel1+=pulseIn(ch1,HIGH);
    }
  channel4=channel4/smooth;
  channel2=channel2/smooth;
  channel1=channel1/smooth;
  CorrectCh4=1500-channel4; //Serial.println(); Serial.println(CorrectCh4);
  CorrectCh2=1500-channel2; //Serial.println(CorrectCh2);
  CorrectCh1=1500-channel1; //Serial.println(CorrectCh1);
  }
}

void loop() {
  channel5=pulseIn(ch5,HIGH); //Serial.println(channel5);
  if(channel5>1900){
  fly();
  }
  else {
    A.writeMicroseconds(Min);
    B.writeMicroseconds(Min);
    C.writeMicroseconds(Min);
    D.writeMicroseconds(Min);
    delay(50);
    //LowPower.idle(SLEEP_4S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
  }
}

void fly() {
  channel1=pulseIn(ch1,HIGH)+CorrectCh1; Serial.println(channel1);
  channel2=pulseIn(ch2,HIGH)+CorrectCh2; Serial.println(channel2);
  channel3=pulseIn(ch3,HIGH); Serial.println(channel3);
  channel4=pulseIn(ch4,HIGH)+CorrectCh4; Serial.println(channel4);
  channel6=pulseIn(ch6,HIGH); Serial.println(channel6); Serial.println();

  if(channel1>1550||channel1<1450||channel2>1550||channel2<1450||channel4>1550||channel4<1450){
    if(channel1<1400){
      channel1+=50;
    }
    else{
      channel1-=50;
    }
    if(channel2<1400){
      channel2+=50;
    }
    else{
      channel2-=50;
    }
    if(channel4<1400){
      channel4+=50;
    }
    else{
      channel4-=50;
    }
    revsA=revos(1);
    revsB=revos(2);
    revsC=revos(3);
    revsD=revos(4);
    //Serial.println("No Stab");
  }
  else if (channel6>1100&&channel6<1900){
    revsA=correction(1);
    revsB=correction(2);
    revsC=correction(3);
    revsD=correction(4);
    //Serial.println("Stab");
  }
  else if (channel6>1900) {
    channel3=1300;
    revsA=correction(1);
    revsB=correction(2);
    revsC=correction(3);
    revsD=correction(4);
  }
  else{
    revsA=0;
    revsB=0;
    revsC=0;
    revsD=0;
  }

  A.writeMicroseconds(channel3+revsA); //Serial.println(revsA);
  B.writeMicroseconds(channel3+revsB); //Serial.println(revsB);
  C.writeMicroseconds(channel3+revsC); //Serial.println(revsC);
  D.writeMicroseconds(channel3+revsD); //Serial.println(revsD); Serial.println();
  //delay(100);
}

int revos(int motor) {
  switch(motor){
    case 1:
      revs=map(channel2,1000,2000,rateX,-rateX)+map(channel1,1000,2000,-rateY,rateY)+map(channel4,1000,2000,rateZ,-rateZ);
      return revs;
    case 2:
      revs=map(channel2,1000,2000,rateX,-rateX)+map(channel1,1000,2000,rateY,-rateY)+map(channel4,1000,2000,-rateZ,rateZ);
      return revs;
    case 3:
      revs=map(channel2,1000,2000,-rateX,rateX)+map(channel1,1000,2000,rateY,-rateY)+map(channel4,1000,2000,rateZ,-rateZ);
      return revs;
    case 4:
      revs=map(channel2,1000,2000,-rateX,rateX)+map(channel1,1000,2000,-rateY,rateY)+map(channel4,1000,2000,-rateZ,rateZ);
      return revs;
  }
  
}

int correction(int motor) {
  avrgx=0, avrgy=0, avrgz=0;

  for (int i=1; i<=smooth; i++) {
    mpu.getAcceleration(&ax,&ay,&az);
    //mpu.getRotation(&gx, &gy, &gz);
    avrgx+=ax; avrgy+=ay; avrgz+=az;
  }

  axx=avrgx/smooth; ayy=avrgy/smooth; //azz=avrgz/smooth;
  pitch=(axx/8192)*45; roll=(ayy/8192)*45; //yaw=(azz/16384)*90;
  //Serial.println(pitch); Serial.println(roll); //Serial.println(yaw); 
  //Serial.println();

  ang=atan2(roll,pitch);
  mag=CorrectP*sqrt(sq(pitch)+sq(roll));
  //Serial.println(mag);
  y=mag*cos(ang);
  x=mag*sin(ang);
  //Serial.println(ang); Serial.println(x); Serial.println(y); Serial.println();
  switch(motor){
    case 1:
      revs=-y-x;
      return revs;
    case 2:
      revs=-y+x;
      return revs;
    case 3:
      revs=y+x;
      return revs;
    case 4:
      revs=y-x;
      return revs;
  }
}
