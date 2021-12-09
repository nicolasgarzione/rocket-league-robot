/* REV 1 */

/* motControl *****************************************************************
 * Code to allow Arduino Mega to control motors and send agyroscope data to RPi
 ******************************************************************************/
 
/* Include the Wire library */
#include<Wire.h>

/*************************************************
 * Defining motor pins used for the motor drivers
 * on the 4-wheeled robot
 *************************************************/

/* front left motor */
const int MOT_0_1 = 24;
const int MOT_0_2 = 25;
const int MOT_0_PWM = 2;

/* Rear left motor */
const int MOT_1_1 = 26;
const int MOT_1_2 = 27;
const int MOT_1_PWM = 3;

/* Front right motor */
const int MOT_2_1 = 10;
const int MOT_2_2 = 11;
const int MOT_2_PWM = 4;

/* Rear right motor */
const int MOT_3_1 = 12;
const int MOT_3_2 = 13;
const int MOT_3_PWM = 5;

/* "Kicker" motor (unused) */
const int MOT_4_1 = 8;
const int MOT_4_2 = 9;
const int MOT_4_PWM = 6;

/* Standby pin for all motor drivers */
const int STANDBY = 22;

/* 2-speed shifting capability */
const int motSpeed1 = 255;
const int motSpeed2 = 155;
int motSpeed = 255;
const int kickSpeed = 100;

/* Address and variables for GY-521 gyroscope sensor */
const int MPU=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;


void setup()
{
    /* Initialization of gyroscope sensor */
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    /* Opens serial channel to RPi */
    Serial.begin(115200);

    /* Pin modes set for all motor control pins */
    pinMode(MOT_0_1, OUTPUT);
    pinMode(MOT_0_2, OUTPUT);
    pinMode(MOT_0_PWM, OUTPUT);

    pinMode(MOT_1_1, OUTPUT);
    pinMode(MOT_1_2, OUTPUT);
    pinMode(MOT_1_PWM, OUTPUT);

    pinMode(MOT_2_1, OUTPUT);
    pinMode(MOT_2_2, OUTPUT);
    pinMode(MOT_2_PWM, OUTPUT);

    pinMode(MOT_3_1, OUTPUT);
    pinMode(MOT_3_2, OUTPUT);
    pinMode(MOT_3_PWM, OUTPUT);

    pinMode(MOT_4_1, OUTPUT);
    pinMode(MOT_4_2, OUTPUT);
    pinMode(MOT_4_PWM, OUTPUT);

    pinMode(STANDBY, OUTPUT);
    digitalWrite(STANDBY, HIGH);
}

void loop()
{
    /* If data (motor control) is being recieved */
    if (Serial.available() > 0)
    {   

      /* Pulls data from gyroscope */
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU,14,true);
      AcX=Wire.read()<<8|Wire.read();
      AcY=Wire.read()<<8|Wire.read();
      AcZ=Wire.read()<<8|Wire.read();
      Tmp=Wire.read()<<8|Wire.read();
      GyX=Wire.read()<<8|Wire.read();
      GyY=Wire.read()<<8|Wire.read();
      GyZ=Wire.read()<<8|Wire.read();

      /* Only gyroscope data in the z axis is sent to RPi representing angle of robot */
      Serial.println(GyZ);
      //delay(50); 

      /*Reads serial data from RPi */
      int serialByte = Serial.readStringUntil('\n').toInt();
 
      /*******************
       *0 stop motors
       *1 forward
       *2 backwards
       *3 strafe left
       *4 strafe right
       *5 forward left
       *6 forward right
       *7 backward left
       *8 backward right
       *9 turn left
       *10 turn right
       *11 speed 1
       *12 speed 2
       *13 spinup
       *14 spindown
       *******************/
        

      if (serialByte == 11)
      {
        /* "Shift up" */
        motSpeed = motSpeed1;
      }
      else if (serialByte == 12)
      {
        /* "Shift down" */
        motSpeed = motSpeed2;
      }
      else if (serialByte == 13)
      {
        /* Spin up kicking motor */
        kickControl(true);
      }
      else if (serialByte == 14)
      {
        /* Spin down kicking motor */
        kickControl(false); 
      }
      else
      {
        /* Change drive direction */
        driveControl(serialByte);
      }
        
    }   // end if (Serial.available() > 0)
    
    
} // end of loop

/* kickControl(...) ***************************************************************
 *     This function is called to either spin up or down the kicking motor.
 **********************************************************************************/

void kickControl(bool mode)
{
  if(mode)
  {
    motorControl(4,-kickSpeed);
  }
  else
  {
    motorControl(4,0);
  }
}

/* driveControl(...) **************************************************************
 *     This function is called to change the driving direction of the robot. Each 
 *     motor direction configuration is stored in this function to be utilized in
 *     directing the robot in 10 distinct directions.
 **********************************************************************************/

void driveControl(int mode)
{
  /* Stops robot */
  if (mode == 0)
  {
    motorControl(0,0);
    motorControl(1,0);
    motorControl(2,0);
    motorControl(3,0);
  }
  /* Forward */
  else if (mode == 1)
  {
    motorControl(0,motSpeed);
    motorControl(1,motSpeed);
    motorControl(2,motSpeed);
    motorControl(3,motSpeed);
  }
  /* Backwards */
  else if (mode == 2)
  {
    motorControl(0,-motSpeed);
    motorControl(1,-motSpeed);
    motorControl(2,-motSpeed);
    motorControl(3,-motSpeed);
  }
  /* Strafe left */
  else if (mode == 3)
  {
    motorControl(0,-motSpeed);
    motorControl(1,motSpeed);
    motorControl(2,motSpeed);
    motorControl(3,-motSpeed);
  }
  /* Strafe right */
  else if (mode == 4)
  {
    motorControl(0,motSpeed);
    motorControl(1,-motSpeed);
    motorControl(2,-motSpeed);
    motorControl(3,motSpeed);
  }
  /* Forward left */
  else if (mode == 5)
  {
    motorControl(0,0);
    motorControl(1,motSpeed);
    motorControl(2,motSpeed);
    motorControl(3,0);
  }
  /* Forward right */
  else if (mode == 6)
  {
    motorControl(0,motSpeed);
    motorControl(1,0);
    motorControl(2,0);
    motorControl(3,motSpeed);
  }
  /* Backwards left */
  else if (mode == 7)
  {
    motorControl(0,-motSpeed);
    motorControl(1,0);
    motorControl(2,0);
    motorControl(3,-motSpeed);
  }
  /* Backwards right */
  else if (mode == 8)
  {
    motorControl(0,0);
    motorControl(1,-motSpeed);
    motorControl(2,-motSpeed);
    motorControl(3,0);
  }
  /* Turn left*/
  else if (mode == 9)
  {
    motorControl(0,-motSpeed);
    motorControl(1,-motSpeed);
    motorControl(2,motSpeed);
    motorControl(3,motSpeed);
  }
  /* Turn right */
  else if (mode == 10)
  {
    motorControl(0,motSpeed);
    motorControl(1,motSpeed);
    motorControl(2,-motSpeed);
    motorControl(3,-motSpeed);
  }
}

/* motorControl(...) **************************************************************
 *     This function is called for each individual motor and is preprogrammed to 
 *     ensure each motor is spining in the right direction and coordinates with the 
 *     desired driving characteristics defined in driveControl(...)
 **********************************************************************************/

void motorControl(int motnum, int input)
{
    if (motnum == 0)
    {
      //Serial.print("0");
      analogWrite(MOT_0_PWM, abs(input));
      if (input >= 0)
      {
        digitalWrite(MOT_0_1, HIGH);
        digitalWrite(MOT_0_2, LOW);
      }
      else
      {
        digitalWrite(MOT_0_1, LOW);
        digitalWrite(MOT_0_2, HIGH);
      }
    }
    else if (motnum == 1)
    {
      //Serial.print("1");
      analogWrite(MOT_1_PWM, abs(input));
      if (input >= 0)
      {
        digitalWrite(MOT_1_1, HIGH);
        digitalWrite(MOT_1_2, LOW);
      }
      else
      {
        digitalWrite(MOT_1_1, LOW);
        digitalWrite(MOT_1_2, HIGH);
      }
    }
    else if (motnum == 2)
    {
      //Serial.print("2");
      analogWrite(MOT_2_PWM, abs(input));
      if (input >= 0)
      {
        digitalWrite(MOT_2_1, LOW);
        digitalWrite(MOT_2_2, HIGH);
      }
      else
      {
        digitalWrite(MOT_2_1, HIGH);
        digitalWrite(MOT_2_2, LOW);
      }
    }
    else if (motnum == 3)
    {
      //Serial.print("3");
      analogWrite(MOT_3_PWM, abs(input));
      if (input >= 0)
      {
        digitalWrite(MOT_3_1, LOW);
        digitalWrite(MOT_3_2, HIGH);
      }
      else
      {
        digitalWrite(MOT_3_1, HIGH);
        digitalWrite(MOT_3_2, LOW);
      }
    }
    else if (motnum == 4)
    {
      //Serial.print("3");
      analogWrite(MOT_4_PWM, abs(input));
      if (input >= 0)
      {
        digitalWrite(MOT_4_1, LOW);
        digitalWrite(MOT_4_2, HIGH);
      }
      else
      {
        digitalWrite(MOT_4_1, HIGH);
        digitalWrite(MOT_4_2, LOW);
      }
    }
    else
    {
      //do nothing
    }
}
