#ifndef SERVO_SIMPLEFOCDXLCORE_H
#define SERVO_SIMPLEFOCDXLCORE_H

#include "Arduino.h"
#include <BLDCMotor.h>
#include <dxlCom.h>

class simplefocDxlCore
{
public:
    // Functions
    simplefocDxlCore();
    uint8_t update();

    // Variables

private:
    // Functions

    // Variables
    BLDCMotor *motor;
    dxlCom *com;

    /*
        bool newparameter;
      bool parameter_available();
      */
};

#endif