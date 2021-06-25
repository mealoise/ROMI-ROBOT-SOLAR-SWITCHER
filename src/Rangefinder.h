#pragma once

#include <Arduino.h>

#include "Romi32U4.h"


class Rangefinder{
    public:
        void setup();
        void loop();
        float getDistanceCM();
    private:
};