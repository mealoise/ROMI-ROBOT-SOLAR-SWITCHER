#pragma once

#include <Arduino.h>
#include <Romi32U4.h>
#include <Romi32U4Encoders.h>
#include <Romi32U4Motors.h>

class Chassis
{
    /** * Assume the robot drives about 12 inches / second* Take the number of inches, divide by 12 and drive that long */
public:
    void driveDistance(float inches);
    /** * Assume the robot turns at about 180 degrees per second */
    void turnAngle(float degrees);
    void findOtherSide();
    void resetPID();
    void turnPID(float degrees);

    const float wheelDiameter = 2.8;
    const int CPR = 1440;
    const float wheelTrack = 5.75;

private:
    Romi32U4Motors motors;
    Romi32U4Encoders encoders;

    int left_current_error = 0;
    int right_current_error = 0;

    int left_integral = 0;
    int right_integral = 0;

    int left_deriv = 0;
    int right_deriv = 0;

    int left_last_error = 0;
    int right_last_error = 0;

    float lpk = 1.2;
    float rpk = .8;

    float lik = 0.0005;
    float rik = 0.0005;

    float ldk = 30;
    float rdk = 20;

    float left_p_gain = 0;
    float right_p_gain = 0;

    float left_i_gain = 0;
    float right_i_gain = 0;

    float left_d_gain = 0;
    float right_d_gain = 0;

    float left_pid_effort = 0;
    float right_pid_effort = 0;
};