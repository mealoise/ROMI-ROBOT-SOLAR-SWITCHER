
#pragma once
#include "Arduino.h"

class LineSensor
{
public:
    LineSensor();
    float getValue();
    int getLeft();
    int getRight();
    void lineFollow();
    //void lineFollowBackward();


    bool al_plate=false;
    void setDoneLineFollow(bool val);
    bool getDoneLineFollow();


    //const
    private:
    bool doneLineFollow = false;
};
