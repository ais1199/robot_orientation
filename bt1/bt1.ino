#include <Wire.h>

/*
 * Progect:
 *  Program for Omni-wheel robot, which is going to the North, overcoming simple obstacles .
 *  2-channel motor driver Pololu on MC33926 <3А
 *  Chip ATmega328
 *  \author ais
 *  \version 1.0
 *  \date 02.10.2016
 *  \date Last Change: 21.03.2017
 */
#include <Arduino.h>

#include <LSM303.h>

LSM303 compass;

//the connection of the engines
#define out1_1      2  // in1
#define PWM_1       3  // in2 PWM
#define out1_2      4  // in3

#define PWM_2       5  // in5 PWM
#define out2_1      6  // in6
#define out2_2      7  // in7

#define out3_1      8  // in8 
#define PWM_3       9  // in9 PWM
#define out3_2      10 // in10

#define RSPEED 75//---------------------------------------speed

#define nn 20//-----------------------------------------sensor base size
#define da 60//----------------------------------------------range for sensors, placed over the wheels(1,2,3)
#define da2 80//--------------------------------------------range for all others sensors(4,5,6)
#define crit 200//-------------------------------------critical range for sensors
#define lim 30//--------------------------------------------//limits to stop overcoming for sensors 1,2,3
#define lim2 45//-----------------------------------------//limits to stop overcoming for sensors 4,5,6

#define S 1//-------------------------------------------compass base size (can be changed)
#define mist 20//---------------------------------------------compass inaccuracy

//----------------------------------------------------------
// --------------------------------------------------------------functions to rule the motors
//----------------------------------------------------------

void Go1(int speed)//for motor 1
{
  if(speed==0)
  {
    digitalWrite(out1_1, HIGH);
    digitalWrite(out1_2, HIGH);
    analogWrite(PWM_1, 255);
  }
  else
  if(speed>0)
  {
    digitalWrite(out1_1, HIGH);
    digitalWrite(out1_2, LOW);
    analogWrite(PWM_1, speed);
  }
  else
  {
    digitalWrite(out1_1, LOW);
    digitalWrite(out1_2, HIGH);
    analogWrite(PWM_1, -speed);
  }
}

void Go2(int speed)//for motor 2
{
  if(speed==0)
  {
    digitalWrite(out2_1, HIGH);
    digitalWrite(out2_2, HIGH);
    analogWrite(PWM_2, 255);
  }
  else
  if(speed>0)
  {
    digitalWrite(out2_1, HIGH);
    digitalWrite(out2_2, LOW);
    analogWrite(PWM_2, speed);
  }
  else
  {
    digitalWrite(out2_1, LOW);
    digitalWrite(out2_2, HIGH);
    analogWrite(PWM_2, -speed);
  }
  
}

void Go3(int speed)//for motor 3
{
  if(speed==0)
  {
    digitalWrite(out3_1, HIGH);
    digitalWrite(out3_2, HIGH);
    analogWrite(PWM_3, 255);
  }
  else
  if(speed>0)
  {
    digitalWrite(out3_1, HIGH);
    digitalWrite(out3_2, LOW);
    analogWrite(PWM_3, speed);
  }
  else
  {
    digitalWrite(out3_1, LOW);
    digitalWrite(out3_2, HIGH);
    analogWrite(PWM_3, -speed);
  }
}
//--------------------------------------------
//--------------------------------------------------functions to move in some direction
//--------------------------------------------
void goFwd(int speed)//forward
{
  Go1(speed);
  Go2(speed);
  Go3(0);
}

void goBack(int speed)//back
{
  Go1(-speed);
  Go2(-speed);
  Go3(0);
}

void goLeft(int speed)//turn to the left
{
  Go1(-speed);
  Go2(speed);
  Go3(speed);
}

void goRight(int speed)//turn to the right
{
  Go1(speed);
  Go2(-speed);
  Go3(-speed);
}

void goFwdLeft(int speed)//go forward and to the left (under 30 degree)
{
  Go1(0);
  Go2(speed);
  Go3(-speed);
}

void goFwdRight(int speed)//go forward and to the right (under 30 degree)
{
  Go1(speed);
  Go2(0);
  Go3(speed);
}

void goBackLeft(int speed)//go backand to the left (under 30 degree)
{
  Go1(-speed);
  Go2(0);
  Go3(-speed);
}

void goBackRight(int speed)//go back and to the right (under 30 degree)
{
  Go1(0);
  Go2(-speed);
  Go3(speed);
}

void goFR(int speed)//go forward, turning to the right, fixed the 2-nd wheel
{
  Go1(speed);
  Go2(0);
  Go3(-speed);
}  

void goFL(int speed)//go forward, turning to the left, fixed the 1-st wheel
{
  Go1(0);
  Go2(speed);
  Go3(speed);
}

void robotStop()//stop the robot
{
  Go1(0);
  Go2(0);
  Go3(0);
}

//--------------------------------------------------------------------------------------sensors part
// if you had a compass or other thing, connected to I2C, don't use A4 and A5 ports
const int analogInPin2 = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue2 = 0;        // value read from the pot
int outputValue2 = 0;        // value output to the PWM (analog out)

const int analogInPin3 = A2;  // Analog input pin that the potentiometer is attached to

int sensorValue3 = 0;        // value read from the pot
int outputValue3 = 0;        // value output to the PWM (analog out)

const int analogInPin1 = A7;  // Analog input pin that the potentiometer is attached to

int sensorValue1 = 0;        // value read from the pot
int outputValue1 = 0;        // value output to the PWM (analog out)

const int analogInPin4 = A1;  // Analog input pin that the potentiometer is attached to

int sensorValue4 = 0;        // value read from the pot
int outputValue4 = 0;        // value output to the PWM (analog out)

const int analogInPin5 = A3;  // Analog input pin that the potentiometer is attached to

int sensorValue5 = 0;        // value read from the pot
int outputValue5 = 0;        // value output to the PWM (analog out)

//const int analogInPin6 = A5;  see that? don't do like this, using I2C
int outputValue6=0;
const int analogInPin7 = A6;  // Analog input pin that the potentiometer is attached to

int sensorValue6 = 0;        // value read from the pot
int outputValue7 = 0;        // value output to the PWM (analog out)

int b=0;
//bases for sensors filters
int a1[nn];
int a2[nn];
int a3[nn];
int a4[nn];
int a5[nn];
int a6[nn];
void zerobase(int*a,int n)//nulls base
{
 int c;
 for(c=0;c<n;c++)
 {
   *(a+c)=0;
 }
}
void writedown()//put data in bases and calculates correct value for sensors data
{
  if(b>=nn)b=0;
  a1[b]=outputValue1;
  a2[b]=outputValue2;
  a3[b]=outputValue3;
  a4[b]=outputValue4;
  a5[b]=outputValue5;
  a6[b]=outputValue7;
  
  b++;
  
  int c;
  outputValue1=0;
  outputValue2=0;
  outputValue3=0;
  outputValue4=0;
  outputValue5=0;
  outputValue7=0;

  for(c=0;c<nn;c++)
  {
    outputValue1+=a1[c];
    outputValue2+=a2[c];
    outputValue3+=a3[c];
    outputValue4+=a4[c];
    outputValue5+=a5[c];
    
    outputValue7+=a6[c];
    
    
  }
  c=outputValue1/nn;
  outputValue1=c;
  c=outputValue2/nn;
  outputValue2=c;
  c=outputValue3/nn;
  outputValue3=c;
  c=outputValue4/nn;
  outputValue4=c;
  c=outputValue5/nn;
  outputValue5=c;
  c=outputValue7/nn;
  outputValue7=c;
  
}
//-----------------------------------------------------------------pointer and others
char mt='w';//set move tipe 
char moving;//where robot should go
float pointer;//point on heading, which robot is to follow
int ni =0;//for keeping nomber of series, which have been done
//----------------------------------------------------------------------------------setup function
void setup()
{
  Serial.begin(9600);
  //-------------------------------------------------------------------compass settings
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
  pointer=0;
  
  // -----------------------------------------------------------------setting output ports for motors
  pinMode(PWM_1, OUTPUT); 
  pinMode(PWM_2, OUTPUT);
  pinMode(PWM_3, OUTPUT);
  pinMode(out1_1, OUTPUT);
  pinMode(out1_2, OUTPUT);
  pinMode(out2_1, OUTPUT);
  pinMode(out2_2, OUTPUT);
  pinMode(out3_1, OUTPUT);
  pinMode(out3_2, OUTPUT);
  //-------------------------------------------------------------setting start position

  ni=0;
  zerobase(a1,nn);
  zerobase(a2,nn);
  zerobase(a3,nn);
  zerobase(a4,nn);
  zerobase(a5,nn);
  zerobase(a6,nn);
  
  moving='w';
  robotStop();
  mt='w';
}
//--------------------------------------------function to get difference in pointer and current heading
float orientation()
{
   float head;
   compass.read();
   head = compass.heading();
    
   float dif=pointer-head;
   if(dif<=-180)dif=360+dif;
   if(dif>=180)dif=-(360-dif);
   return dif;
}
//------------------------------------------------------------------------loop

void loop()
{ 
 int dop; 
 float dif;
    //getting sensor data. 7 and 6 is not a mistake. it is a problems of output
  sensorValue6 = analogRead(analogInPin7);            
  outputValue7 = map(sensorValue6, 0, 1023, 0, 255);
  
  analogWrite(analogOutPin, outputValue7);
  
  sensorValue5 = analogRead(analogInPin5);            
  outputValue5 = map(sensorValue5, 0, 1023, 0, 255);  
  analogWrite(analogOutPin, outputValue5); 
  sensorValue4 = analogRead(analogInPin4);            
  outputValue4 = map(sensorValue4, 0, 1023, 0, 255);  
  analogWrite(analogOutPin, outputValue4); 
  sensorValue3 = analogRead(analogInPin3);            
  outputValue3 = map(sensorValue3, 0, 1023, 0, 255);  
  analogWrite(analogOutPin, outputValue3);   
  sensorValue2 = analogRead(analogInPin2);            
  outputValue2 = map(sensorValue2, 0, 1023, 0, 255);  
  analogWrite(analogOutPin, outputValue2); 
  sensorValue1 = analogRead(analogInPin1);            
  outputValue1 = map(sensorValue1, 0, 1023, 0, 255);  
  analogWrite(analogOutPin, outputValue1);
  writedown();//correcting with filter
  //move correcting, based on compass data 
  if(ni%2==0)
  {
    dif=orientation(); //reading difference 
    if(((outputValue7<lim2)&&(outputValue2<lim)&&(outputValue1<lim)))
    {
      if((mt=='o')&&(dif<=30&&dif>-30))mt='w';//correcting move tipe
    }

    if((dif>mist||dif<-mist)&&mt=='w')//correcting heading, if move tipe is forward
    {
      
      if(dif<0)
      {
        moving='q';        
      }
      else
      {
        moving='e';        
      }
    }
    else
    {
      moving='w';
    }
  }
  
 
  if(((outputValue7>da2)||(outputValue2>da)||(outputValue1>da)))//corrections for move tipe and heading, if robot sow the wall infront of it
  {
    
    if(mt=='w')mt='o'; 
    if(mt=='o')moving='q';
  }
  else
  {
    if(mt=='o') //anuther case, when robot overgoing the wall
    {
      moving='w';
      if(outputValue2<da-15)moving='r';
    }
    
  }
  //reserve wall detectors, which shuldn't be used 
  if(((outputValue1>da)||(outputValue7>da2)||(outputValue5>da2)) && moving=='a')
  {
    robotStop();
  }
  
  if(((outputValue5>da2)||(outputValue3>da)||(outputValue1>da)) && moving=='z')
  {
    robotStop();
    
  }
  if(((outputValue3>da)||(outputValue4>da2)||(outputValue5>da2)) && moving=='x')
  {
    robotStop();
    
  }
  if(((outputValue4>da2)||(outputValue2>da)||(outputValue3>da)) && moving=='c')
  {
    robotStop();
    
  }
  if(((outputValue2>da)||(outputValue7>da2)||(outputValue4>da2)) && moving=='d')
  {
    robotStop();
    
  }
//reading moving, and taking action, based on it. all movings are protected from crashing
  if(((outputValue7<da2)&&(outputValue2<da)&&(outputValue1<da)))
  {
    if(moving=='w')
    {
      goFwd(RSPEED);
    }
    if(moving=='r')
    {
      goFR(RSPEED);
    }
    if(moving=='l')
    {
      goFL(RSPEED);
    }
  }
  
  if(((outputValue5<da2)&&(outputValue4<da2)&&(outputValue3<da)))
  {
    if(moving=='x')goBack(RSPEED);
  }
  
  if(((outputValue7<da2)&&(outputValue5<da2)&&(outputValue1<da)))
  {
    if(moving=='a')goFwdLeft(RSPEED);
  }
  
  if(((outputValue7<da2)&&(outputValue2<da)&&(outputValue4<da2)))
  {
    if(moving=='d')goFwdRight(RSPEED);
  }

  if(((outputValue5<da2)&&(outputValue3<da)&&(outputValue1<da)))
  {
    if(moving=='z')goBackLeft(RSPEED);
  }
  
  if(((outputValue4<da2)&&(outputValue2<da)&&(outputValue3<da)))
  {
    if(moving=='c')goBackRight(RSPEED);
  }
  
  if((outputValue4<crit)&&(outputValue2<crit)&&(outputValue3<crit)&&(outputValue5<crit)&&(outputValue7<crit)&&(outputValue1<crit))
  {
    if(moving=='q')
  {
    if(mt=='o')goLeft(RSPEED-15);
    else
    {
      goLeft(RSPEED-5);
    }
  }
  if(moving=='e')
  {
    goRight(RSPEED-5);
  }
  }
  else
  {
    robotStop();
  }
  //sleep 5
  ni++;
  delay(5);
  

}  
