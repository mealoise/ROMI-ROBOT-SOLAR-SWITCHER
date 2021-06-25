#include "Chassis.h"
#include <Romi32U4.h>

/** * Assume the robot drives about 12 inches / second Take the number of inches, divide by 12 and drive that long */
void Chassis::driveDistance(float cm)
{
    encoders.getCountsAndResetLeft();
    float inches = cm / 2.54;
    float newVal = (inches / 7.8539) * CPR;
    while (encoders.getCountsLeft() < newVal)
    {
        motors.setEfforts(100, 100);
    };
    motors.setEfforts(0, 0);
    delay(1000);

    /**
    motors.setEfforts(100, 100);
    delay(inches/12 * 1000);
    motors.setEfforts(0, 0);
    */
}
/** * Assume the robot turns at about 180 degrees per second */
void Chassis::turnAngle(float degrees)
{
    encoders.getCountsAndResetLeft();
    float newVal = (degrees / 360) * wheelTrack * CPR / 2.5; // 2.5 inches is the diameter measured for my romi wheels
    while (encoders.getCountsLeft() < newVal)
    {
        motors.setEfforts(100, -100);
    }
    motors.setEfforts(0, 0);
    delay(1000);
}

long chassisTemp = 4;

void Chassis::findOtherSide()
{
    for (int i = 0; i < 3; i++)
    {
        driveDistance(chassisTemp); // some small dist
        turnAngle(90);
    }
}

void Chassis::resetPID()
{

    left_current_error = 0;
    right_current_error = 0;

    left_integral = 0;
    right_integral = 0;

    left_last_error = 0;
    right_last_error = 0;
}

void Chassis::turnPID(float degrees)
{
    //establishing errors
    left_current_error = (-1 * degrees / deg_in_one) * CPR - encoders.getCountsLeft();
    right_current_error = (degrees / deg_in_one) * CPR - encoders.getCountsRight();

    left_integral += left_current_error;
    right_integral += right_current_error;

    left_deriv = left_current_error - left_last_error;
    right_deriv = right_current_error - right_last_error;

    //gain calcs
    left_p_gain = (lpk * left_current_error);
    right_p_gain = (rpk * right_current_error);

    left_i_gain = (lik * left_integral);
    right_i_gain = (rik * right_integral);

    left_d_gain = (ldk * left_deriv);
    right_deriv = (rdk * right_deriv);
    // Establish efforts
    left_pid_effort = left_p_gain + left_i_gain + left_d_gain;
    right_pid_effort = right_p_gain + right_i_gain + right_d_gain;

    // Setting Efforts

    if (left_pid_effort > 140)
    {
        left_pid_effort = 140;
    }
    else if (left_pid_effort < -140)
    {
        left_pid_effort = -140;
    }

    if (right_pid_effort > 140)
    {
        right_pid_effort = 140;
    }
    else if (right_pid_effort < -140)
    {
        right_pid_effort = -140;
    }

    // remembering errors for derivative control
    left_last_error = left_current_error;
    right_last_error = right_current_error;

    // setting efforts
    motors.setLeftEffort(left_pid_effort);
    motors.setRightEffort(right_pid_effort);
}
