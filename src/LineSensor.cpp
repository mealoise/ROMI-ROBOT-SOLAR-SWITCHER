#include "LineSensor.h"
#include "Romi32U4Motors.h"

Romi32U4Motors motors1;

LineSensor::LineSensor()
{
}

float LineSensor::getValue()
{
    return getLeft() - getRight();
}

int LineSensor::getLeft()
{
    return analogRead(A2);
}

int LineSensor::getRight()
{
    return analogRead(A3);
}

void LineSensor::lineFollow()
{
    float K = .08;
    if (getRight() > 800 && getLeft() > 800)
    {
        motors1.setEfforts(0, 0);
        doneLineFollow = true;
    }
    else
    {
        motors1.setEfforts(55 + (K * getLeft()), 50 + (K * getRight()));
    }

}
// void LineSensor::lineFollowBackward()
// {
//     float K = -1 * 0.08;

//     if (getRight() > 800 && getLeft() > 800)
//     {
//         motors1.setEfforts(0, 0);
//         //doneLineFollow = true;
//     }
//     else
//     {
//         motors1.setEfforts(-1 * (50 + (K *getLeft())), -1 * (45 + (K * getRight())));
//     }
// }

void LineSensor::setDoneLineFollow(bool val)
{
    doneLineFollow = val;
}

bool LineSensor::getDoneLineFollow()
{
    return doneLineFollow;
}