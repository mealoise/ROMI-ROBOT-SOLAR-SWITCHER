#include <Arduino.h>
#include <Romi32U4.h>
#include "Chassis.h"
#include "IRdecoder.h"
#include "RemoteConstants.h"
#include "BlueMotor.h"
#include "Romi32U4Buttons.h"
#include "Timer.h"
#include "servo32u4.h"
#include "LineSensor.h"
#include "Rangefinder.h"

Romi32U4Motors motors;
Romi32U4Encoders encoders;
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
  LIFTING45PLATE,
  DRIVINGTO90TURN,
  CROSSTURN,
  DRIVINGTOSUPPLY,
  ROTATEATSUPPLY,
  PLACINGONSUPPLY,
  AWAITINGNEWPANEL,
  GRABBINGFROMSUPPLY,
  DRIVINGTONEG90TURN,
  CROSSTURNTWO,
  ROTATEAT45,
  BACKINGUP45,
  DRIVINGTO45PLACING,
  PLACING45,
  FINDINGOTHERSIDE,
  DRIVINGTO25,
  GRIPPING25,
  DRIVETO25PLACING,
  ROTATEAT25,
  BACKINGUP25,
  LIFTING25PLATE,
  PLACING25,
  PAUSED
} state;

enum traverse_progress
{
  TURN1,
  FD1,
  TURN2,
  FD2,
  TURN3,
  FD3,
  LOCATEHOUSE
} progress;

//States current_state = GRIPPING45;

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
  case remote2:
  {
    newReady = true;
    break;
  case remote3:
    newReady = false;
    break;
  case remote5:
    paused = true;
    stateBeforePaused = state;
    state = PAUSED;
    break;
  case remote6:
    paused = false;
    state = STARTING;
    break;
  }
  }
}

BlueMotor bm;
Servo32U4 servo;
LineSensor ls;

bool turnAndInch_finished = false;

void turnAndInchToRoof()
{
  if (chassis.getDoneTurning())
  {
    servo.Write(1100); // open gripper

    if (analogRead(A0) < 240) // open gripper analog read
    {
      if (chassis.getDDBDone())
      {
        turnAndInch_finished = true;
      }
      else
      {
        chassis.driveDistanceBackward(5.3);
      }
    }
  }
  else
  {
    chassis.turnPID(175);
  }
}

int old_effort = 0;

const char X = 5;
char encoderArray[4][4] = {
    {0, -1, 1, X},
    {1, 0, X, -1},
    {-1, X, 0, 1},
    {X, 1, -1, 0}};

Romi32U4ButtonB pb;

// effort variables used in driveToObject()
int16_t old_efforts = 0;
int16_t new_efforts = 0;

float dto_error = 0;
bool dto_done = false;
float dto_int_error = 0;

void driveToObject(double desiredDist)
{

  const float Kp = 8;
  const float Ki = .2;
  dto_error = range.getDistanceCM() - desiredDist;
  dto_int_error += dto_error;
  float a = (Kp * dto_error) + (Ki * dto_int_error);

  new_efforts = constrain(a, -50, 60);

  if (range.getDistanceCM() > desiredDist + 40)
  {
    // motors.setEfforts(1.1 * old_efforts, old_efforts);
    ls.lineFollow();
  }
  else
  {
    ls.lineFollow();
  }
  dto_done = dto_error < .25;
}

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
  //state = PAUSED;
  state = STARTING;
  progress = TURN1;
  range.setup();

  Serial.begin(9600);
  decoder.init();

  pinMode(bm.PWMOutPin, OUTPUT);
  pinMode(bm.AIN2, OUTPUT);
  pinMode(bm.AIN1, OUTPUT);
  pwmSetup();
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  attachInterrupt(digitalPinToInterrupt(0), isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(1), isr, CHANGE);

  bm.setup();

  chassis.resetPID();

  servo.SetMinMaxUS(900, 2500);
  servo.Attach();
  pinMode(18, INPUT); // IR receiver

  // put your setup code here, to run once:
}

const int targetBMPos = 140;
const float KpBM = 6;
const float KiBM = 0.1;
const float KdBM = 0;

long timeToPrint = 0;
float sumOfErrorsBM = 0;

long romiLen = 16; // in CM

Timer PIDTimer(10);

void loop()
{
  checkRemote();
  range.loop();

  switch (state)
  {
  case STARTING:
  {
    bm.moveTo(-3282); // to be ready to grab 45; 45 pickup initial position
    if (bm.wasArmMoved())
    {
      state = DRIVINGTO45;
      chassis.resetPID();
    }
  }
  break;

  case DRIVINGTO45:
  {
    if (PIDTimer.isExpired() && !dto_done)
    {
      driveToObject(22);
      PIDTimer.reset();
    }
    if (!turnAndInch_finished && ((range.getDistanceCM()) < 22.5 || dto_done))
    {
      turnAndInchToRoof();
    }
    else if (turnAndInch_finished)
    {
      state = GRIPPING45;
      chassis.resetPID();
    }
  }
  break;

  case ADJUSTINGGRIPPER: //TODO: not being used currently
  {
    //Serial.println("ADJUSTINGGRIPPER state");
    long errorG = analogRead(A0) - 400; //servo read value vs desired
    Serial.println("analogread success");
    servo.Write(2500 + errorG * 2500 / 400);
    if (errorG * errorG < 0.01) // alter later; to compensate for error
    {                           // or some other val
      if (stage == 1)           // not exactly but whatever
      {
        // third is while adjust gripper is finished
        state = GRIPPING45;
      }
      else if (stage == 2)
      {
        state = GRABBINGFROMSUPPLY;
      }
      else if (stage == 3)
      {
        state = GRIPPING25;
      }
      else if (stage == 4)
      {
        state = GRABBINGFROMSUPPLY;
      }
      else
      {
        //Serial.println("Oh no! 1");
        state = PAUSED;
      }
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
      servo.Write(2250);
      if (analogRead(A0) > 395)
      {
        stage++;
        state = LIFTING45PLATE;
        bm.resetPID();
      }
    }
  }
  break;

  case LIFTING45PLATE:
  {
    bm.moveTo(-7100);
    if (bm.wasArmMoved())
    {
      bm.resetPID();
      bm.setEffort(0);
      chassis.setDDDone(false);
      chassis.setDoneTurning(false);
      chassis.resetPID();
      ls.setDoneLineFollow(false);
      //encoders.getCountsAndResetRight();
      //encoders.getCountsAndResetLeft();
      state = DRIVINGTO90TURN;
    }
  }
  break;
  case DRIVINGTO90TURN:
  {
    if (ls.getDoneLineFollow())
    // line sensor to stop at corner
    {
      chassis.driveDistance(0.5 * romiLen);

      if (chassis.getDDDone())
      {
        chassis.resetPID();
        chassis.setDoneTurning(false);
        encoders.getCountsAndResetRight();
        encoders.getCountsAndResetLeft();
        state = CROSSTURN;
      }
    }
    else
    {
      ls.lineFollow();
    }
  }
  break;

  case CROSSTURN:
  {

    chassis.turnPID(90);
    if (chassis.getDoneTurning())
    {
      chassis.resetPID();
      //bm.resetPID();
      chassis.setDoneTurning(false);
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      dto_done = false;
      state = DRIVINGTOSUPPLY;
    }
  }
  break;

  case DRIVINGTOSUPPLY:
  {
    if (dto_done)
    {
      state = ROTATEATSUPPLY;
      chassis.resetPID();
      chassis.setDoneTurning(false);
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
    }
    else
    {
      driveToObject(15);
    }
  }
  break;

  case ROTATEATSUPPLY:
  {
    chassis.turnPID(175);
    if (chassis.getDoneTurning())
    {
      state = PLACINGONSUPPLY;
      bm.resetPID();
      chassis.setDDBDone(false);
      chassis.checkloop = 0;
    }
  }
  break;

  case PLACINGONSUPPLY:
  {
    checkRemote();
    if (restart)
    {
      //state = STARTING;
    }
    else
    {

      chassis.driveDistanceBackward(5); // drive backwards small amount to line w supply
      if (chassis.getDDBDone())
      {
        bm.moveTo(-20); // desired height for supply dropoff
        if (bm.wasArmMoved())
        {
          bm.setEffort(0);
          servo.Write(1100); // open gripper
          if (analogRead(A0) < 240)
          { // to be adjusted if needed
            state = AWAITINGNEWPANEL;
          }
        }
      }
    }
  }
  break;

  case AWAITINGNEWPANEL:
  {
    checkRemote();
    if (newReady)
    {
      servo.Write(2250);        // close gripper
      if (analogRead(A0) > 395) // checks if gripper is sufficiently closed before proceeding
      {
        state = GRABBINGFROMSUPPLY;
        bm.resetPID();
      }
    }
  }
  break;

  case GRABBINGFROMSUPPLY:
  {
    checkRemote(); // might not need all these but for double measure
    if (restart)
    {
      //state = STARTING;
    }
    else
    {
      bm.moveTo(-7100);
      if (bm.wasArmMoved())
      {

        state = DRIVINGTONEG90TURN;
        bm.resetPID();
        bm.setEffort(0);
        ls.setDoneLineFollow(false);
        chassis.setDDDone(false);
        chassis.setDoneTurning(false);
        chassis.resetPID();
      }
    }
  }
  break;

  case DRIVINGTONEG90TURN:
  {
    if (ls.getDoneLineFollow())
    // line sensor to stop at corner
    {
      chassis.driveDistance(0.5 * romiLen + 0.3); // to position Romi at center of T for proper point turn

      if (chassis.getDDDone())
      {
        chassis.resetPID();
        chassis.setDoneTurning(false);
        encoders.getCountsAndResetRight();
        encoders.getCountsAndResetLeft();
        state = CROSSTURNTWO;
      }
    }
    else
    {
      ls.lineFollow();
    }
  }
  break;

  case CROSSTURNTWO:
  {
    chassis.turnPID(-90);
    if (chassis.getDoneTurning())
    {
      chassis.resetPID();
      bm.resetPID();
      chassis.setDoneTurning(false);
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      dto_done = false;
      state = DRIVINGTO45PLACING;
    }
  }
  break;

  case DRIVINGTO45PLACING:
  {
    if (dto_done)
    {
      state = ROTATEAT45;
      chassis.resetPID();
      chassis.setDoneTurning(false);
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
    }
    else
    {
      driveToObject(21.2);
    }
  }
  break;

  case ROTATEAT45:
  {
    chassis.turnPID(180);
    if (chassis.getDoneTurning())
    {
      state = BACKINGUP45;
      bm.resetPID();
      chassis.resetPID();
      chassis.setDDBDone(false);
      chassis.checkloop = 0;
    }
  }
  break;
  case BACKINGUP45:
  {
    if (chassis.getDDBDone())
    {
      state = PLACING45;
    }
    else
    {
      chassis.driveDistanceBackward(5.3);
    }
  }
  break;
  case PLACING45:
  {
    checkRemote();
    if (restart)
    {
      // state = STARTING;
    }
    else
    {
      bm.moveTo(-3624);
      if (bm.wasArmMoved())
      {
        servo.Write(1100); // open gripper
        if (analogRead(A0) < 240)
        { // subj to change
          state = FINDINGOTHERSIDE;
          chassis.resetPID();
          chassis.setDoneTurning(false);
          chassis.setDDBDone(false);
          chassis.setDDDone(false);
        }
      }
    }
  }
  break;

  case FINDINGOTHERSIDE:
  {

    switch (progress)
    {
    case TURN1:
    {
      chassis.turnPID(90);
      if (chassis.getDoneTurning())
      {
        progress = FD1;
        chassis.resetPID();
        encoders.getCountsAndResetLeft();
        encoders.getCountsAndResetRight();
      }
    }
    break;

    case FD1:
    {
      chassis.driveDistanceNoLine(30);
      if (chassis.getDDDone())
      {
        progress = TURN2;
        chassis.setDDDone(false); // This is going to be an issue
        chassis.resetPID();
        encoders.getCountsAndResetLeft();
        encoders.getCountsAndResetRight();
      }
    }
    break;

    case TURN2:
    {
      chassis.turnPID(90);
      if (chassis.getDoneTurning())
      {
        chassis.resetPID(); // also going to be in issue etc etc
        progress = FD2;
        encoders.getCountsAndResetLeft();
        encoders.getCountsAndResetRight();
      }
    }
    break;

    case FD2:
    {
      chassis.driveDistanceNoLine(70);
      if (chassis.getDDDone())
      {
        progress = TURN3;
        chassis.setDDDone(false);
        chassis.resetPID();
        encoders.getCountsAndResetLeft();
        encoders.getCountsAndResetRight();
      }
    }
    break;

    case TURN3:
    {
      chassis.turnPID(90);
      if (chassis.getDoneTurning())
      {
        progress = FD3;
        chassis.resetPID();
        chassis.setDDDone(false);
        encoders.getCountsAndResetLeft();
        encoders.getCountsAndResetRight();
      }
    }
    break;

    case FD3:
    {
      if (ls.getDoneLineFollow())
      {
        chassis.driveDistance(.5 * romiLen);
        if (chassis.getDDDone())
        {
          progress = LOCATEHOUSE;
          chassis.resetPID();
          encoders.getCountsAndResetLeft();
          encoders.getCountsAndResetRight();
        }
      }
      else
      {
        ls.lineFollow();
      }
    }
    break;

    case LOCATEHOUSE:
    {
      chassis.turnPID(90);
      if (chassis.getDoneTurning())
      {
        bm.moveTo(-3600); // temporary value, change to desired height for 25 pickup
        state = DRIVINGTO25;
        chassis.resetPID(); // issue w reset while in this condition potentially
        encoders.getCountsAndResetLeft();
        encoders.getCountsAndResetRight();
      }
    }
    break;
    }
  }
  break;

  case DRIVINGTO25:
  {
    stage++;

    if (PIDTimer.isExpired() && !dto_done)
    {
      driveToObject(22);
      //Serial.println(range.getDistanceCM());
      PIDTimer.reset();
    }
    //Serial.println(turnAndInch_finished);
    if (!turnAndInch_finished && ((range.getDistanceCM()) < 22.5 || dto_done))
    {
      //Serial.println("drive initilize success");
      //delay(2500);
      turnAndInchToRoof();
    }
    else if (turnAndInch_finished)
    {
      Serial.println("inch to roof success");

      state = GRIPPING25;
      chassis.resetPID();
    }
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

      servo.Write(2250);
      if (analogRead(A0) > 395)
      {
        stage++;
        state = LIFTING25PLATE;
        bm.resetPID();
      }
    }
  }
  break;
  case LIFTING25PLATE:
  {
    bm.moveTo(-7100);
    if (bm.wasArmMoved())
    {
      bm.resetPID();
      bm.setEffort(0);
      chassis.setDDDone(false);
      chassis.setDoneTurning(false);
      chassis.resetPID();
      ls.setDoneLineFollow(false);
      //encoders.getCountsAndResetRight();
      //encoders.getCountsAndResetLeft();
      state = DRIVINGTONEG90TURN;
    }
  }
  break;
  case DRIVETO25PLACING:
  {
    //where temp is desired dist; could be issues w conditions changing
    if (dto_done)
    {
      state = ROTATEAT25;
      chassis.resetPID();
      chassis.setDoneTurning(false);
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
    }
    else
    {
      driveToObject(21.2);
    }
  }
  break;
  case ROTATEAT25:
  {
    chassis.turnPID(180);
    if (chassis.getDoneTurning())
    {
      state = BACKINGUP25;
      bm.resetPID();
      chassis.resetPID();
      chassis.setDDBDone(false);
      chassis.checkloop = 0;
    }
  }
  break;
  case BACKINGUP25:
  {
    if (chassis.getDDBDone())
    {
      state = PLACING25;
    }
    else
    {
      chassis.driveDistanceBackward(5.3);
    }
  }
  case PLACING25:
  {
    checkRemote();
    if (restart)
    {
      // state = STARTING;
    }
    else
    {

      bm.moveTo(-3624); // new desired 25 location
      if (bm.wasArmMoved())
      {
        servo.Write(1100); // open gripper
        if (analogRead(A0) < 240)
        { // subj to change
          state = FINDINGOTHERSIDE;
          chassis.resetPID();
          chassis.setDoneTurning(false);
          chassis.setDDBDone(false);
          chassis.setDDDone(false);
        }
      }
    }
  }
  break;
  case PAUSED:
  {
    // motors.setEfforts(0, 0);
    // bm.setEffort(0);
    // servo.Detach();
    // checkRemote();
    // if (!paused)
    // {
    //   state = stateBeforePaused;
    // }
  }
  break;
  }
}
