#include <QTRSensors.h>

#define RightMotorF 6
#define RightMotorB 7
#define LeftMotorF 10
#define LeftMotorB 11


QTRSensorsRC qtr((unsigned char[]) {A0, 12, A1, A2, A3, A4, 11, A5}, 8, 2500);
 
void setup()
{
  pinMode(RightMotorF, OUTPUT);
  pinMode(RightMotorB, OUTPUT);
  pinMode(LeftMotorF, OUTPUT);
  pinMode(LeftMotorB, OUTPUT);

  Serial.begin(9600);
  for (int i = 0; i < 100; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  } 
  
}

int R = 4.2;             //Radius of wheels
int L = 8.5;             //Distance between wheels
float alpha = L/R;      //defined so that while calculating v_r, R/L do not have dimention
float omega = 0;
int v_l = 0;       //Velocity of left wheel
int v_r = 0;       //Velocity of right wheel

void loop()
{
    unsigned int sensors[8];
  
    int position = qtr.readLine(sensors, QTR_EMITTERS_ON, 1);        //for white line on black surface
   
    int error_pos = int(position) - 3500;
    float error_phi = error_pos * 3.14 / 35;

    omega = PID(error_phi);
    
    const int v = 400;

    /*              v_l = (2*v + omega*L)/(2*R)               */
    v_l = (2*v/R + omega*alpha)/2;
    v_r = (2*v/R - omega*alpha)/2;

    if(v_l>0)
    {
    analogWrite(LeftMotorF,v_l);
    analogWrite(LeftMotorB,0);
    }
    else
    {
    analogWrite(LeftMotorF,0);
    analogWrite(LeftMotorB,-v_l);
    }
    if(v_r>0)
    {
    analogWrite(RightMotorF,v_r);
    analogWrite(RightMotorB,0);
    }
    else
    {
    analogWrite(RightMotorF,0);
    analogWrite(RightMotorB,-v_r);
    }    


    // printing commands for testing-----------------------------------------------------------------------

    for(int i =0; i<8; i++)
    {
      Serial.print(sensors[i]);
      Serial.print('\t');
    }
    Serial.println();
    Serial.println(error_phi);
    Serial.println(omega);
    Serial.println(v_l);
    Serial.println(v_r);
    Serial.println();
   //------------------------------------------------------------------------------------------------------

}


float kp = 0.5;  // 0.08 // for small = 0.1
float kd = 0 ; // 1.0   // for small = 1.7
float ki = 0;
int lastError = 0;
int integral = 0;
int derivative = 0;
int Time, last_time=0;
int delta_time;



int PID(int err_phi)
{
  
    integral += err_phi;
    derivative = err_phi - lastError;
    Time=millis();
    delta_time = Time - last_time;

    //PID
    int omega = kp * err_phi + ki * integral + kd * derivative/delta_time;
    
    lastError = err_phi;
    last_time=Time;

    return omega;
}
