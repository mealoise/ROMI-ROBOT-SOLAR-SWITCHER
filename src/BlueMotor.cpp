#include "BlueMotor.h"
#include "Pushbutton.h"
#include "Romi32U4.h"
#include "Romi32U4Buttons.h"

//Romi32U4Motors motors;
//Romi32U4Encoders encoders;
long value;

void BlueMotor::setup()
{
  pinMode(PWMOutPin, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  // motors.setEfforts(0, 0);
}

void BlueMotor::reset()
{

  digitalWrite(PWMOutPin, LOW);
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
  if (clockwise)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  int value = constrain(effort, 0, 400);
  OCR1C = value;
}

void BlueMotor::setEffort(int effort)
{
  if (effort < 0)
  {
    setEffort(-effort, true); //clockwise
  }
  else
  {
    setEffort(effort, false); //anticlockwise
  }
}

void BlueMotor::setEffortWithoutDB(float effort)
{
  //propotionally over the 0 to 400 range
  // so if the live band value starts from 60, 0 would be equal to 60 and 1 would be equal to 61
  if (effort == 0)
  {
    effort_new = 0;
  }
  else if (effort > 0)
  {
    effort_new = 138 + (.655 * effort);
  }
  else if (effort < 0)
  {
    effort_new = (-140 + (.65 * effort));
  }
  setEffort(effort_new);
  //where a is the starting point of the deadband value
}

int BlueMotor::getNewEffort()
{
  return effort_new;
}

void BlueMotor::moveTo(long position)
{
  // setting values for current PID loop
  pid_current_error = position - getPosition();
  pid_integral += pid_current_error;
  pid_deriv = pid_current_error - pid_last_error;

  // calculating PID output effort
  p_gain = (proportional_k * pid_current_error);
  i_gain = (integral_k * pid_integral);
  d_gain = (deriv_k * pid_deriv);

  effort_out = p_gain + i_gain + d_gain;

  if (effort_out > 400)
  {
    effort_out = 400;
  }
  else if (effort_out < -400)
  {
    effort_out = -400;
  }
  //effort_out=constrain(effort_out,-400,400);
  // remembering variables for next calc cycle
  pid_last_error = pid_current_error;
  setEffortWithoutDB(effort_out);
}

float BlueMotor::getEffortOut()
{
  return effort_out;
}

long BlueMotor::getPosition()
{
  return count;
}


