#pragma once

#include "Romi32U4.h"

class BlueMotor
{
public:
    //BlueMotor();
    const int PWMOutPin = 11;
    const int AIN2 = 4;
    const int AIN1 = 13;
    const int ENCA = 0;
    const int ENCB = 1;
    int count = 0;
    int value = 0;
    int oldValue = 0;
    int newValue = 0;
    int errorCount = 0;
    void setEffort(int effort);
    void setEffortWithoutDB(float effort);
    void moveTo(long position); // for next lab
    long getPosition();
    int getNewEffort();
    void reset();
    void setup();
    float getEffortOut();

    int effort_new = 0;
    float p_gain = 0;
    float i_gain = 0;
    float d_gain = 0;
    float get_effort_new();


private:
    void setEffort(int effort, bool clockwise);
    static void isr();
    const int tolerance = 3;
    //int effort_new=0;
    long pid_current_error = 0;
    long pid_last_error = 0;
    float proportional_k = 5;
    long pid_integral = 0;
    float integral_k = 0;
    long pid_deriv = 0;
    float deriv_k = .3;
    float effort_out = 0;
};
