#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include "RF24.h"

#define  CE_PIN  7
#define  CSN_PIN 8
#define bt 3

MPU6050 mpu;

RF24 radio(CE_PIN, CSN_PIN);

int val = -1, preval;
byte addresses[][6] = {"1Node", "2Node"};
float ctx,cty,ctz;
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;    
uint16_t packetSize;   
uint16_t fifoCount;     
uint8_t fifoBuffer[64];

Quaternion q;           
VectorInt16 aa;       
VectorInt16 aaReal;     
VectorInt16 aaWorld;  
VectorFloat gravity;    
float ypr[3];          

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

class A{
  public:
  float x,y,z;  
};
A angle;

int Get(){
  float X,Y,Z;
  bool modified=1;
  while(modified){ 
    while (!mpuInterrupt && fifoCount < packetSize) {}
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount(); 
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            X=ypr[0]*180/M_PI;
            Y=ypr[1]*180/M_PI;
            Z=ypr[2]*180/M_PI;
            modified=0;
    }
  }
  X+=ctx;
  if(X>360)
  X-=360;
  if(X<0)
  X=X+360;
  
  Y+=cty;
  if(Y>360)
  Y-=360;
  if(Y<0)
  Y=Y+360;
  
  Z+=ctz;
  if(Z>360)
  Z-=360;
  if(Z<0)
  Z=Z+360;
  
  angle.x=X;
  angle.y=Y;
  angle.z=Z;
  return 1;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(bt, INPT_PULLUP);
  Wire.begin();
  mpu.initialize();
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);
  radio.begin();
  radio.setChannel(85);  
  radio.setDataRate(RF24_250KBPS); 
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.stopListening(); 
  mpu.initialize();
  pinMode(2, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
  for(int i = 0;i<600;i++, delay(100),Serial.print('.'))
    Get();
  Get();
  ctx=90-angle.x;
  cty=90-angle.y;
  ctz=90-angle.z;
}

void loop() {
  // put your main code here, to run repeatedly:
  preval = val;
  val = !digitalRead(bt);
  if(val == 1 and preval == 0){
    Get();
    ctx=90-angle.x;
    cty=90-angle.y;
    ctz=90-angle.z;
  }
  Get();
  String S;
  S=angle.x;
  S+="    ";
  S+=angle.y;
  S+="    ";
  S+=angle.z;
  radio.write(&angle, sizeof(angle));
  Serial.println(S);
  delay(50);
}
