\\******** RBE 2001 Final Project **********\\
\\******** March 17th 2021 ***************\\
\\ Developed by Megan Aloise, Roopsa Ghoosh, and Cameron Huneke \\


#include <Arduino.h>
#include <Romi32U4.h>
#include "Chassis.h"
#include "IRdecoder.h"
#include "RemoteConstants.h"
#include "BlueMotor.h"
#include "Romi32U4Buttons.h"
#include "Timer.h"
#include "servo32u4.h"
#include "Rangefinder.h"

Romi32U4Motors motors;
Chassis chassis;
Rangefinder range;

long temp = 4.1;
long desiredDist = 12; // in CM

enum States
{
  STARTING,
  DRIVINGTO45,
  ADJUSTINGGRIPPER,
  GRIPPING45,
  DRIVINGTO90TURN,
  DRIVINGTOSUPPLY,
  PLACINGONSUPPLY,
  AWAITINGNEWPANEL,
  GRABBINGFROMSUPPLY,
  DRIVINGTONEG90TURN,
  DRIVINGTO45PLACING,
  PLACING45,
  FINDINGOTHERSIDE,
  DRIVINGTO25,
  GRIPPING25,
  DRIVETO25PLACING,
  PLACING25,
  PAUSED
} state;

int stage = 1;

IRDecoder decoder(14);

bool paused = false;
bool newReady = false;
bool restart = false;

States stateBeforePaused = STARTING; // hopefully it's fine

void checkRemote()
{
  int16_t code = decoder.getKeyCode();
  switch (code)
  {
  case remoteVolMinus:
    paused = true;
    stateBeforePaused = state;
    state = PAUSED;
    break;
  case remoteVolPlus:
    paused = false;
    state = stateBeforePaused;
    break;
  case remote2:
    newReady = true;
    // state = ?
    break;
  case remote3:
    newReady = false;
    // state = ?
    break;
  case remote5:
    restart = true;
    //state = ?
    break;
  case remote6:
    restart = false;
    //state = ?
    break;
  }
}

BlueMotor bm;
Servo32U4 servo;

void turnAndInchToRoof()
{
  chassis.turnPID(180);
  servo.Write(300);             // open gripper
  chassis.driveDistance(-temp); // desired dist NEGATIVE
}

int old_effort = 0;

const char X = 5;
char encoderArray[4][4] = {
    {0, -1, 1, X},
    {1, 0, X, -1},
    {-1, X, 0, 1},
    {X, 1, -1, 0}};

Romi32U4ButtonB pb;

void isr()
{
  bm.newValue = (digitalRead(1) << 1) | digitalRead(0);
  char value = encoderArray[bm.oldValue][bm.newValue];
  if (value == X)
  {
    bm.errorCount++;
  }
  else
  {
    bm.count -= value;
  }
  bm.oldValue = bm.newValue;
}

//generating the PWM signals

void sweep()
{
  digitalWrite(bm.AIN1, HIGH);
  digitalWrite(bm.AIN2, LOW);
  for (int i = 0; i < 400; i++)
  {
    OCR1C = i;
    delay(10);
  }
}

void pwmSetup()
{
  TCCR1A = 0xA8;
  TCCR1B = 0x11;
  ICR1 = 400;
  OCR1C = 0;
}

void setup()
{
  state = STARTING;

  range.setup();
  Serial.begin(9600);
  decoder.init();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(bm.PWMOutPin, OUTPUT);
  pinMode(bm.AIN2, OUTPUT);
  pinMode(bm.AIN1, OUTPUT);
  pwmSetup();
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  attachInterrupt(digitalPinToInterrupt(0), isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(1), isr, CHANGE);

  servo.Init();
  servo.Attach();
  servo.SetMinMaxUS(900, 2100);
  pinMode(18, INPUT);

  // put your setup code here, to run once:
}

const int targetBMPos = 140;
const float KpBM = 6;
const float KiBM = 0.1;
const float KdBM = 0;

long timeToPrint = 0;
float sumOfErrorsBM = 0;

long romiLen = 17.5; // in CM

void loop()
{

  //Lab Part 1, when doing the other direction just change the loop to
  //for(int i=0;i>-400;i--)
  //for(int i=0;i<400;i++)
  //check for the effort values and the position
  //check if the motr turns consistently starts to turn

  long temporary = 5.5;
  long encoderVarTemp = 3.2;

  switch (state)
  {
  case STARTING:
    // make sure lifitng after in start position
    motors.setEfforts(150, 150); //choose efforts
    state = DRIVINGTO45;

  case DRIVINGTO45:
  {
    long currentDist = range.getDistanceCM();
    while (currentDist < desiredDist)
    {                                                      // temp is dist desired to proper placement
      long error = range.getDistanceCM() - encoderVarTemp; // change to however encoder calculated
      if (error * error < temporary)
      { // some reasonable value
        motors.setEfforts(0, 0);
        turnAndInchToRoof();
        state = ADJUSTINGGRIPPER;
      }
      else
      {
        while (encoderVarTemp < temporary)
        {                          // temp is dist desired
          motors.setEfforts(0, 0); // may need to separate from while statement
          turnAndInchToRoof();
          servo.Write(0); // close gripper
          state = ADJUSTINGGRIPPER;
        };
      }
    };
  }
  break;
  case ADJUSTINGGRIPPER:
  {
    long errorG = encoderVarTemp - temp; //servo read value vs desired
    while (errorG * errorG > 0.01)
    {                      // or some other val
      servo.Write(errorG); // alter later; to compensate for error
    };
    if (stage == 1 && encoderVarTemp == temp) // not exactly but whatever
    {
      // third is while adjust gripper is finished
      state = GRIPPING45;
    }
    else if (stage == 2 && encoderVarTemp == temp)
    {
      state = GRABBINGFROMSUPPLY;
    }
    else if (stage == 3 && encoderVarTemp == temp)
    {
      state = GRIPPING25;
    }
    else if (stage == 4 && encoderVarTemp == temp)
    {
      state = GRABBINGFROMSUPPLY;
    }
    else
    {
      Serial.println("Oh no! 1");
      state = PAUSED;
    }
  }
  break;
  case GRIPPING45:
  {
    checkRemote();
    if (restart)
    {
      state = STARTING;
    }
    else
    {
      motors.setEfforts(150, 150); // desired effort
      stage++;
      state = DRIVINGTO90TURN;
    }
  }
  break;
  case DRIVINGTO90TURN:
  {
    // line sensor to stop at corner
    if (temp > temporary)
    { // voltage values
      motors.setEfforts(0, 0);
      chassis.driveDistance(0.5 * romiLen); // where temp is length of Romi
      chassis.turnPID(90);
      motors.setEfforts(150, 150);
    }
    if (stage <= 2)
    {
      state = DRIVINGTOSUPPLY;
    }
    else if (stage >= 3)
    {
      state = DRIVETO25PLACING;
    }
    else
    {
      Serial.println("Oh no! 2");
      state = PAUSED;
    }
  }
  break;
  case DRIVINGTOSUPPLY:
  {
    while (range.getDistanceCM() < desiredDist)
    { // where temp is desired dist
      motors.setEfforts(0, 0);
      chassis.turnPID(180);
      bm.moveTo(temp);              // desired height
      chassis.driveDistance(-temp); // drive backwards small amount to line w supply
      state = PLACINGONSUPPLY;
    };
  }
  break;
  case PLACINGONSUPPLY:
  {
    checkRemote();
    if (restart)
    {
      state = STARTING;
    }
    else
    {
      servo.Write(300);            // open gripper
      chassis.driveDistance(temp); // a few forward
      state = AWAITINGNEWPANEL;
    }
  }
  break;
  case AWAITINGNEWPANEL:
  {
    checkRemote();
    if (newReady)
    {
      chassis.driveDistance(-temp); // some small dist back
      servo.Write(0);               // close gripper
      state = ADJUSTINGGRIPPER;
    }
    else
    {
      Serial.println("Oh No! 3");
      state = PAUSED;
    }
  }
  break;
  case GRABBINGFROMSUPPLY:
  {
    checkRemote(); // might not need all these but for double measure
    if (restart)
    {
      state = STARTING;
    }
    else
    {
      bm.moveTo(temp); // desired pos for 45
      motors.setEfforts(150, 150);
      if (stage <= 2)
      {
        state = DRIVINGTONEG90TURN;
      }
      else
      {
        state = DRIVINGTO90TURN;
      }
    }
  }
  break;
  case DRIVINGTONEG90TURN:
  {
    // use line sensor to detect corner/stop
    if (temp < temporary)
    { //line sensor reading a stop w voltages
      motors.setEfforts(0, 0);
      chassis.driveDistance(0.5 * romiLen); // where temp is length of Romi
      chassis.turnPID(90);
      motors.setEfforts(150, 150);
    }
    if (stage <= 2)
    {
      state = DRIVINGTO45PLACING;
    }
    else if (stage >= 3)
    {
      state = DRIVINGTOSUPPLY;
    }
    else
    {
      Serial.println("Oh no! 4");
      state = PAUSED;
    }
  }
  break;
  case DRIVINGTO45PLACING:
  {
    while (range.getDistanceCM() < desiredDist)
    { //where temp is desired dist; could be issues w conditions changing
      motors.setEfforts(0, 0);
      chassis.turnPID(180);
      bm.moveTo(temp);              // temp is desired pos
      chassis.driveDistance(-temp); // where temp is small NEG dist
      state = PLACING45;
    };
  }
  break;
  case PLACING45:
  {
    checkRemote();
    if (restart)
    {
      state = STARTING;
    }
    else
    {
      servo.Write(300); // open gripper
      chassis.findOtherSide();
      motors.setEfforts(150, 150);
      state = FINDINGOTHERSIDE;
    }
  }
  break;
  case FINDINGOTHERSIDE:
  {
    // use line sensor to detect path
    if (temp < temporary)
    { //line sensor condition of detecting path
      motors.setEfforts(0, 0);
      chassis.driveDistance(0.5 * romiLen); // where temp is romi len
      chassis.turnPID(90);
      motors.setEfforts(150, 150);
      state = DRIVINGTO25;
    }
  }
  break;
  case DRIVINGTO25:
  {
    stage++;
    while (range.getDistanceCM() < desiredDist)
    {
      chassis.turnPID(180);
      chassis.driveDistance(-temp); // drive backward some small dist
      servo.Write(0);               // close gripper
      state = ADJUSTINGGRIPPER;
    };
  }
  break;
  case GRIPPING25:
  {
    checkRemote();
    if (restart)
    {
      state = STARTING;
    }
    else
    {
      stage++;
      motors.setEfforts(150, 150);
      state = DRIVINGTONEG90TURN;
    }
  }
  break;
  case DRIVETO25PLACING:
  {
    while (range.getDistanceCM() < desiredDist)
    { // where temp is desired dist from platform
      motors.setEfforts(0, 0);
      chassis.turnPID(180);
      chassis.driveDistance(temp); // some small dist
      bm.moveTo(temp);             // desired height
      state = PLACING25;
    }
  }
  break;
  case PLACING25:
  {
    checkRemote();
    if (restart)
    {
      state = STARTING;
    }
    else
    {
      servo.Write(300);            // open gripper
      chassis.driveDistance(temp); // some small dist to back up
      state = PAUSED;
    }
  }
  break;
  case PAUSED:
  {
    motors.setEfforts(0, 0);
  }
  break;
  }

  checkRemote();
}
