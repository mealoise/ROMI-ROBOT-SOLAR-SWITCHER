#include "LineSensor.h"

LineSensor::LineSensor()
{
    
}

float LineSensor::getValue()
{
    return getLeft() - getRight();
}

float LineSensor::getLeft()
{
    return analogRead(A2);
}

float LineSensor::getRight()
{
    return analogRead(A3);
}
