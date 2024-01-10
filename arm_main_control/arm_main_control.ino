#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include "HCPCA9685.h"

/********** serialport def and var **********/
#define BUFFER_LENGTH 20
#define FRAME_LENGTH 16
#define DATA_LENGTH 12 // data has 10 bytes
#define CRC_LOW_IDX 13
#define CRC_HIGH_IDX 14
int revBuffer[BUFFER_LENGTH] = {0}; // buffer for receiving frame
int dataBuffer[DATA_LENGTH] = {0}; // buffer storing valid data
int revCount = 0;
bool revStart = false;
SoftwareSerial mySerial(10,11); // rx, tx

/********** motor data **********/
unsigned int jointAngles[6] = {0}; //angles of 6 joints
unsigned int jointAnglesLast[6] = {0};
float jointAnglesNow[6] = {0};
unsigned int servoPos[6] = {0}; // pos cmd sent to servo
unsigned int servoPosLast[6] = {0}; // last joint angles

/********** servo driver module PCA9685 **********/
/*
PCA9685...........Uno/Nano
GND...............GND
OE................N/A
SCL...............A5
SDA...............A4
VCC...............5V
*/
// I2C slave address for the device/module. For the HCMODU0097 the default I2C addressis 0x40 
#define  I2C_ADD 0x40
#define RESPONSE_TIME 5 //microseconds
// motors' idx on PCA9685
#define SHOULDER_IDX 12
#define ELBOW_IDX    11
#define WRIST1_IDX   10
#define WRIST2_IDX   9
#define GRIPPER_IDX  8
HCPCA9685 hcpca9685(I2C_ADD); // Create an instance of the library

float tf = 1;
float timestep = 0.1;


void setup() {
  // put your setup code here, to run once:

  /********** serialport init **********/
  Serial.begin(115200); // USB serial
  mySerial.begin(115200); // extra serial
  mySerial.println("SerialPort Init Complete.");
  delay(3000);

  /********** servo driver module init **********/
  hcpca9685.Init(SERVO_MODE);
  hcpca9685.Sleep(false);// Wake the device up
  mySerial.println("Servo Driver Init Complete.");
  delay(3000);

  /********** servo init **********/
  mySerial.println("Servo back to 0 pos......");
  initServo();
  mySerial.println("Servo Init Complete.");
  delay(2000);


  mySerial.println("Init All Clear");

}

void loop() {
  // put your main code here, to run repeatedly:

  /********** serialport receive **********/
  if(Serial.available() > 0)
  {
    // inByte用int接收.如果用char需要将read返回的值转换为char型,否则会出问题,不是正确值
    int inByte = Serial.read();
    if(inByte == 0xA8) // frame header detected
    {
      revStart = true;
      //mySerial.print("header verified\r\n");
    }
    if(revStart)
    {
      revBuffer[revCount] = inByte;
      revCount += 1;
    }
    if(revCount == FRAME_LENGTH) // receive finish
    {
      //mySerial.print("rev finish\r\n");
      if(revBuffer[FRAME_LENGTH -1] == 0xFE) // 
      {
          //mySerial.println("ender verify success");
          unsigned short revCrc = revBuffer[CRC_LOW_IDX] + (revBuffer[CRC_HIGH_IDX] << 8); // crc directly from received data
          // dataArray用char才能crc校验成功
          char dataArray[DATA_LENGTH] = {0}; // temporarily store data
          for(int i = 0; i < DATA_LENGTH; i++) // extract data from revBuffer
          {
            dataArray[i] = revBuffer[1+i];
          }
          unsigned short crc = calCrc(dataArray); // crc calced using received data
          if(crc == revCrc) // verify success
          {
            //mySerial.println("crc verify success");
            for(int i = 0; i < DATA_LENGTH; i++)// move data to dataBuffer
            {
              dataBuffer[i] = dataArray[i]; 
            }
          }
          else // crc verify failure
          {
            //mySerial.println("crc verify failure");
            Serial.flush(); // refresh serial port data
          }
      }
      else
      {
        //mySerial.println("ender verify failure");
        Serial.flush(); // refresh serial port data
      }
      revStart = false; //
      revCount = 0; // back to initial value
    }  
  }

  /********** data process **********/
  decodeJointAngles();
  jointAnglesToServoPos();


  /********** drive the motor **********/
  //printJointAngles();
  //printServoPos();
  cmdServoInterpolateToTarget(); // interpolate to target pos
  


}

// calc crc value
unsigned short calCrc(char* dataArray)
{
    int k = 0;
    int len = DATA_LENGTH;
    unsigned short crc = 0xFFFF;  // initial value of crc's 16 bit register

    while(len--)
    {
        crc ^= dataArray[k++];
        for (int i = 0; i < 8; i++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001; //多项式 POLY（0x8005)的高低位交换值，这是由于其模型的一些参数决定的
            else
                crc = (crc >> 1);
        }
    }

    return crc;
}

void decodeJointAngles()
{
  for(int i = 0; i < 6; i++)
  {
    //jointAnglesLast[i] = jointAngles[i];
    unsigned int angle = dataBuffer[i*2] + (dataBuffer[i*2+1] << 8);
    jointAngles[i] = angle;

  }
}

void printJointAngles()
{
  mySerial.print("joint angles:");
  mySerial.print(jointAngles[0]);
  mySerial.print(",");
  mySerial.print(jointAngles[1]);
  mySerial.print(",");
  mySerial.print(jointAngles[2]);
  mySerial.print(",");
  mySerial.print(jointAngles[3]);
  mySerial.print(",");
  mySerial.print(jointAngles[4]);
  mySerial.print(",");
  mySerial.println(jointAngles[5]);

}

void printServoPos()
{
  mySerial.print("Servo Pos:");
  mySerial.print(servoPos[0]);
  mySerial.print(",");
  mySerial.print(servoPos[1]);
  mySerial.print(",");
  mySerial.print(servoPos[2]);
  mySerial.print(",");
  mySerial.print(servoPos[3]);
  mySerial.print(",");
  mySerial.print(servoPos[4]);
  mySerial.print(",");
  mySerial.println(servoPos[5]);
}

void jointAnglesToServoPos()
{
  for(int i = 0; i < 6; i++) // 90 in jointAngle corresponds to 160 in pos
  {
    servoPosLast[i] = servoPos[i]; // update last pos
    servoPos[i] = (unsigned int)((float)jointAngles[i] / 90.0 * 160.0); // update target pos
  }
}

// 感觉插值会改善一点,但也没有很多
// 内层循环在servoPos上变化
void cmdServoInterpolateToTarget()
{
  // joint 2: shoudler
  for(int i = servoPosLast[1]; i <= servoPos[1]; i++)
  {
    hcpca9685.Servo(SHOULDER_IDX, i);
    delayMicroseconds(RESPONSE_TIME);
  }
  // joint 3: elbow
  for(int i = servoPosLast[2]; i <= servoPos[2]; i++)
  {
    hcpca9685.Servo(ELBOW_IDX, i);
    delayMicroseconds(RESPONSE_TIME);
  }
  // joint 4: wrist1
  for(int i = servoPosLast[3]; i <= servoPos[3]; i++)
  {
    hcpca9685.Servo(WRIST1_IDX, i);
    delayMicroseconds(RESPONSE_TIME);
  }
  // joint 5: wrist2
  for(int i = servoPosLast[4]; i <= servoPos[4]; i++)
  {
    hcpca9685.Servo(WRIST2_IDX, i);
    delayMicroseconds(RESPONSE_TIME);
  }
  //joint 6: gripper
  for(int i = servoPosLast[5]; i <= servoPos[5]; i++)
  {
    hcpca9685.Servo(GRIPPER_IDX, i);
    delayMicroseconds(RESPONSE_TIME);
  }
}

void cmdServo()
{
  hcpca9685.Servo(SHOULDER_IDX, servoPos[1]); // second element of jointAngles is shoulder
  //delay(RESPONSE_TIME); // wait for motors to respond
  hcpca9685.Servo(ELBOW_IDX, servoPos[2]);
  //delay(RESPONSE_TIME); // wait for motors to respond
  hcpca9685.Servo(WRIST1_IDX, servoPos[3]);
  //delay(RESPONSE_TIME); // wait for motors to respond
  hcpca9685.Servo(WRIST2_IDX, servoPos[4]);
  //delay(RESPONSE_TIME); // wait for motors to respond
}

void initServo()
{
  hcpca9685.Servo(SHOULDER_IDX, 0); // second element of jointAngles is shoulder
  delay(10); // wait for motors to respond
  hcpca9685.Servo(ELBOW_IDX, 0);
  delay(10); // wait for motors to respond
  hcpca9685.Servo(WRIST1_IDX, 0);
  delay(10); // wait for motors to respond
  hcpca9685.Servo(WRIST2_IDX, 0);
  delay(10); // wait for motors to respond
  hcpca9685.Servo(GRIPPER_IDX, 0);
  delay(10); // wait for motors to respond
}

// 内层循环在jointAngles上变化
float cubicSpline(float _tf, float t)
{
  for(int i = 0; i < 6; i++)
  {
    float a0 = (float)jointAnglesLast[i]; //初始角度
    mySerial.print(a0);
    float a1 = 0;
    float a2 = 3/(pow(_tf,2))*((float)jointAngles[i] - (float)jointAnglesLast[i]);
    float a3 = -2/(pow(_tf,3))*((float)jointAngles[i] - (float)jointAnglesLast[i]);
    jointAnglesNow[i] = a0 + a1*(t) +a2*(pow(t,2)) +a3*(pow(t,3));
  }

  mySerial.print("joint angles Now:");
  mySerial.print(jointAnglesNow[0]);
  mySerial.print(",");
  mySerial.print(jointAnglesNow[1]);
  mySerial.print(",");
  mySerial.print(jointAnglesNow[2]);
  mySerial.print(",");
  mySerial.print(jointAnglesNow[3]);
  mySerial.print(",");
  mySerial.print(jointAnglesNow[4]);
  mySerial.print(",");
  mySerial.println(jointAnglesNow[5]);

  // a0 = _thetai;
  // a1 = 0;
  // a2 = 3/(pow(_tf,2))*(_thetaf - _thetai);
  // a3 = -2/(pow(_tf,3))*(_thetaf - _thetai);
  // _theta0 = a0 + a1*(t) +a2*(pow(t,2)) +a3*(pow(t,3));
  // return _theta0; 
}

void cmdServoCubicSpline()
{
  for(float t = 0; t <= tf; t += timestep)
  {
    cubicSpline(tf, t); //得到当前舵机目标角度
    // 将当前舵机目标角度转换为servoPos
    for(int i = 0; i < 6; i++) // 90 in jointAngle corresponds to 160 in pos
    {
      servoPos[i] = (unsigned int)(jointAnglesNow[i] / 90.0 * 160.0); // update target pos
    }
    // 驱动舵机
    hcpca9685.Servo(SHOULDER_IDX, servoPos[1]); // second element of jointAngles is shoulder
    delayMicroseconds(RESPONSE_TIME);
    hcpca9685.Servo(ELBOW_IDX, servoPos[2]);
    delayMicroseconds(RESPONSE_TIME);
    hcpca9685.Servo(WRIST1_IDX, servoPos[3]);
    delayMicroseconds(RESPONSE_TIME);
    hcpca9685.Servo(WRIST2_IDX, servoPos[4]);
    delayMicroseconds(RESPONSE_TIME);
    hcpca9685.Servo(GRIPPER_IDX, servoPos[5]);
    delayMicroseconds(RESPONSE_TIME);
  }
}
