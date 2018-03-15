

//  To detect the type of junction in a grid based maze.
//  Types of junction: [ T ] or [ + ]  [ L ]


// CODE TO INITIALIZE QTR ---------------------------------------------------------------------------------------------------------

#include <QTRSensors.h>

#define rightMotorF 9
#define rightMotorB 10
#define rightMotorPWM 11
#define leftMotorF 6
#define leftMotorB 7
#define leftMotorPWM 5
#define stby 8


QTRSensorsRC qtr((unsigned char[]) {12,A0, A1, A2, A3, 4}, 6, 2500);


// QTR INITIALISATION ENDS----------------------------------------------------------------------------------------------------------


// CODE TO INITIALIZE IMU ---------------------------------------------------------------------------------------------------------

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_EULER


//Defined for IMU

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
float init_imu;

int base;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() 
{

  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  pinMode(13,OUTPUT);

 //QTR calibration

    for (int i = 0; i < 100; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }


  // BEGIN IMU INIT
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);

  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

/*
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
 

  // END IMU INIT
    
  while(millis() < 20000) {
    init_imu = imu_read();
    Serial.println(millis());
  }
  base = imu_read();

}

int lastError = 0;
float kp = 0.1;
float kd = 0;
float ki = 0;
int integral = 0;
int derivative = 0;
int Time, last_time=0;
int delta_time;
unsigned int sensors[6];
int count_b = 0;

//LOOP----------------------------------------------------------------------------------------------------------------------------------------------------------

void loop()
{
  
  //Serial.println (imu_read_off());
  if(check_junction())
  {
    count_b = 1;
  }
  if(count_b > 0)
  {
    digitalWrite(13,HIGH);
  }
  else
    digitalWrite(13,HIGH);
  
  
  
  int position = qtr.readLine(sensors,QTR_EMITTERS_ON, 1);
   
    //Serial.println(position);
    int error = int(position) - 2500;        //for white line on black surface
    //Serial.println(error);
    integral += error;
    derivative = error - lastError;
    Time=millis();
    delta_time=Time-last_time;
    int power_difference = kp * error + ki * integral + kd * derivative/delta_time;
    //Serial.println(power_difference);
    lastError = error;
    last_time=Time;
    
    
    const int maximum = 100;
    
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;
      
    if (power_difference < 0) {
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM, maximum);
      digitalWrite(leftMotorF, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorPWM, maximum + power_difference);
      digitalWrite(stby,HIGH);
    } 
    else { 
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM, maximum - power_difference);
      digitalWrite(leftMotorF, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorPWM, maximum);
      digitalWrite(stby,HIGH);
    }
}

//LOOP end ----------------------------------------------------------------------------------------------------------------------------------------------------

float imu_read()
{
 if (!dmpReady) return 0;
    while (!mpuInterrupt && fifoCount < packetSize) {
    }


    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();


    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));


    } else if (mpuIntStatus & 0x02) {

        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();


        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

       #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            if (euler[0]*180/M_PI <0){
              float neg_deg = euler[0]*180/M_PI +360;
              Serial.print("euler\t");
              Serial.println(neg_deg);
              return neg_deg;
            }
            else{
              Serial.print("euler\t");
              Serial.println(euler[0] * 180/M_PI);
              return  euler[0]*180/M_PI;
            }
         #endif
    }
}


float imu_read_off()
{
  float check = imu_read();
  check = check - base;
  
  if (check < 0)
  {
    check += 360;
  }
 Serial.println(check);
  return check;
}

//IMU INITIALISATION ENDS----------------------------------------------------------------------------------------------------------


//Variable declaration-------------------------------------------------------------------------------------------------------------

char ch_L = 'n';        //for 90 turn left condition
char ch_R = 'n';        //for 90 turn right condition
char ch_T = 'n';        //for T-junc condition
char ch_135 = 'n';      //for 135 junc condition
float turn_i = 0, turn_f = 0;      // value of angles before and after turns
float turn = 0;         //change in angle after turn
int count = 0;          //to hold sensor check for 10 iterations and to read imu after 10 iteration
int const iterations = 10;


//Variable declaration ends-------------------------------------------------------------------------------------------------------------


float angle_diff(float current, float previous)
{ 
  float total = 0;
  if (current <15 && previous > 345)
  {
    if (abs(current + 360 - previous) < 15)
    {
          total = total + abs(current + 360 - previous);
    }
  }
  else if (current > 345 && previous < 15)
  {
    if (abs(previous + 360 - current) < 15)
    {
          total = total + abs(previous + 360 - current);
    }
  }
  else
  {
    if (abs(current - previous) < 15)
    {
          total = total + abs(current - previous);
    }
  }
  return total;
}


// JUNCTION DETECTION BEGINS-------------------------------------------------------------------------------------------------

int check_junction()
{
  //for 90L and T(1,1) and Y(1,1)

  if((ch_L == 'n')&&(sensors[0]>800 && sensors[5]<200))
  {
    ch_L = 'y';
    turn_i = imu_read_off();
    count = 0;
  }

  if (count < iterations && ch_L == 'y' )
  {
      turn_f = imu_read_off();
      turn+= angle_diff(turn_f,turn_i);
  }

  // CONSIDERING RANGE OF TURN AS (+-10)

  if(count == iterations && ch_L=='y')
    {
      if(abs(turn)>0 && abs(turn)<10)
      {
        /*RETURN T(1,1) i.e. 1*/
        return 1;
      }
      if(abs(turn)>35 && abs(turn)<55)                //NOT SURE IF +45 OR -45 SO INCLUDING ABS() AND SIMILARLY IN FUTURE
      {
        /*RETURN Y(1,1) i.e. 4*/
        return 4;
      }
      if(abs(turn)>80 && abs(turn)<100)
      {
        /*RETURN 90L i.e. 90*/
        return 90;
      }
      count = 0;
      ch_L = 'n';
    }

  //for 90R and T(1,1) and Y(1,1)

  if((ch_R == 'n')&&(sensors[0]>800 && sensors[1]>800 && sensors[2]<200 && sensors[3]<200 && sensors[4]<200 && sensors[5]<200))
  {
    ch_R = 'y';
    turn_i = imu_read_off();
    count = 0;
  }

  if (count < iterations && ch_L == 'y' )
  {
      turn_f = imu_read_off();
      turn+= angle_diff(turn_f,turn_i);
  }

  if(count==iterations && ch_R=='y')
  {
    if(abs(turn)>0 && abs(turn)<10)               //for turn=0
      {
        /*RETURN T(1,1) i.e. 1*/
        return 2;
      }
    if(abs(turn)>35 && abs(turn)<55)
      {
        /*RETURN Y(1,1) i.e. 4*/
        return 5;
      }
    if(abs(turn)>80 && abs(turn)<100)
      {
        /*RETURN 90R i.e. 90*/
        return 90;
      }
    count = 0;
    ch_R = 'n';
  }

  //for T(2) i.e. 3 and Y(2) i.e. 6
  //for y(2) sensor[0] and [7] might not read white always
  if((ch_T == 'n')&&(sensors[0]<800 && sensors[1]<200 && sensors[2]<200 && sensors[3]<200 && sensors[4]<200 && sensors[5]<800))
  {
    ch_T = 'y';
    turn_i = imu_read_off();
    count = 0;
  }

  if (count < iterations && ch_T == 'y' )
  {
      turn_f = imu_read_off();
      turn+= angle_diff(turn_f,turn_i);
  }

  if(count==iterations && ch_T=='y')
  {
    if (abs(turn)>80 && abs(turn)<100)
    {
      /* return T(2) */
      return 3;
    }
    if (abs(turn)>35 && abs(turn)<55)
    {
      /* return Y(2) */
      return 6;
    }
  }

  //for 135 turn i.e. 135
  if((ch_135=='n')&&((sensors[0]>700 && sensors[1]<500 && sensors[2]<200 && sensors[3]<200 && sensors[4]>800 && sensors[5]>800)||(sensors[0]>800 && sensors[1]>800 && sensors[2]<200 && sensors[3]<200 && sensors[4]<200 && sensors[5]<500)))
  {
    ch_135 = 'y';
    turn_i = imu_read_off();
    count = 0;
  }

  if (count < iterations && ch_135 == 'y' )
  {
      turn_f = imu_read_off();
      turn+= angle_diff(turn_f,turn_i);
  }  

  if(count==iterations && ch_135=='y')
  {
    if(abs(turn)>35 && abs(turn)<55)
      /* return turn 135*/
      return 135;
  }
  
  //for END i.e. -1
  //for dead END
  count++;
  return 0;
}




//  JUNCTION DETECTION ENDS ---------------------------------------------------------------------------------------------------------------
