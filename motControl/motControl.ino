
#include<Wire.h>
//0 front left, 1 rear left, 2 front right, 3 rear right
const int MOT_0_1 = 24;
const int MOT_0_2 = 25;
const int MOT_0_PWM = 2;

const int MOT_1_1 = 26;
const int MOT_1_2 = 27;
const int MOT_1_PWM = 3;

const int MOT_2_1 = 10;
const int MOT_2_2 = 11;
const int MOT_2_PWM = 4;

const int MOT_3_1 = 12;
const int MOT_3_2 = 13;
const int MOT_3_PWM = 5;
 
const int STANDBY = 22;

const int motSpeed1 = 255;
const int motSpeed2 = 155;
int motSpeed = 255;
const int kickSpeed = 100;

const int MPU=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

const int MOT_4_1 = 8;
const int MOT_4_2 = 9;
const int MOT_4_PWM = 6;

void setup()
{
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
  
    Serial.begin(115200);

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

    pinMode(STANDBY, OUTPUT);
    digitalWrite(STANDBY, HIGH);

    pinMode(MOT_4_1, OUTPUT);
    pinMode(MOT_4_2, OUTPUT);
    pinMode(MOT_4_PWM, OUTPUT);
}

void loop()
{
    if (Serial.available() > 0)
    {   
      
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

    Serial.println(GyZ);
    delay(50);
      
      int serialByte = Serial.readStringUntil('\n').toInt();

      //Serial.println(String(serialByte));

      

      //0 stop motors
      //1 forward
      //2 backwards
      //3 strafe left
      //4 strafe right
      //5 forward left
      //6 forward right
      //7 backward left
      //8 backward right
      //9 turn left
      //10 turn right
      //11 speed 1
      //12 speed 2
      //13 spinup
      //14 spindown

      if (serialByte == 11)
      {
        motSpeed = motSpeed1;
      }
      else if (serialByte == 12)
      {
        motSpeed = motSpeed2;
      }
      else if (serialByte == 13)
      {
        kickControl(true);
      }
      else if (serialByte == 14)
      {
        kickControl(false); 
      }
      else
      {
        driveControl(serialByte);
      }
        
    }               // end if (Serial.available() > 0)
    
    
} // end of loop

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

void driveControl(int mode)
{
  if (mode == 0)
  {
    motorControl(0,0);
    motorControl(1,0);
    motorControl(2,0);
    motorControl(3,0);
  }
  else if (mode == 1)
  {
    motorControl(0,motSpeed);
    motorControl(1,motSpeed);
    motorControl(2,motSpeed);
    motorControl(3,motSpeed);
  }
  else if (mode == 2)
  {
    motorControl(0,-motSpeed);
    motorControl(1,-motSpeed);
    motorControl(2,-motSpeed);
    motorControl(3,-motSpeed);
  }
  else if (mode == 3)
  {
    motorControl(0,-motSpeed);
    motorControl(1,motSpeed);
    motorControl(2,motSpeed);
    motorControl(3,-motSpeed);
  }
  else if (mode == 4)
  {
    motorControl(0,motSpeed);
    motorControl(1,-motSpeed);
    motorControl(2,-motSpeed);
    motorControl(3,motSpeed);
  }
  else if (mode == 5)
  {
    motorControl(0,0);
    motorControl(1,motSpeed);
    motorControl(2,motSpeed);
    motorControl(3,0);
  }
  else if (mode == 6)
  {
    motorControl(0,motSpeed);
    motorControl(1,0);
    motorControl(2,0);
    motorControl(3,motSpeed);
  }
  else if (mode == 7)
  {
    motorControl(0,-motSpeed);
    motorControl(1,0);
    motorControl(2,0);
    motorControl(3,-motSpeed);
  }
  else if (mode == 8)
  {
    motorControl(0,0);
    motorControl(1,-motSpeed);
    motorControl(2,-motSpeed);
    motorControl(3,0);
  }
  else if (mode == 9)
  {
    motorControl(0,-motSpeed);
    motorControl(1,-motSpeed);
    motorControl(2,motSpeed);
    motorControl(3,motSpeed);
  }
  else if (mode == 10)
  {
    motorControl(0,motSpeed);
    motorControl(1,motSpeed);
    motorControl(2,-motSpeed);
    motorControl(3,-motSpeed);
  }
}

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
      
    }

    //Serial.println("Recieved");
    
    //Serial.print("Motor ");
    //Serial.print(motnum);
    //Serial.print(" at power: ");
    //Serial.println(abs(input));
}
