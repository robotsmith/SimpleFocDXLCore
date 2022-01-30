#ifndef SERVO_SIMPLEFOCDXLCORE_H
#define SERVO_SIMPLEFOCDXLCORE_H

#include "Arduino.h"
#include <BLDCMotor.h>
#include <dxlCom.h>
#include <dxlMemory.h>

class simplefocDxlCore
{
public:
    // *** Functions

    // Constructor
    simplefocDxlCore(BLDCMotor *_motor);
    /*
    Update the dxl core
    */
    void update();
    /*
    attach the serial port to the dxlCore
    @param  &serial Serial port
    */
    void attach(HardwareSerial &serial);

    // Factory reset memory
    void factoryResetMem();

    // load default parameters in memory
    void loadDefaultMem();

    // Execute the command in the DXL packet
    void executePacketCommand();
    // *** Variables

private:
    // *** Functions

    // Refresh data from motor and update DXL memory
    void refreshMotorData();
    // Update parameters from com to memory
    void update_parameters();

    // *** Variables

    // Simplefoc motor
    BLDCMotor *motor;

    // DXL memory
    dxlMemory dxlmem;
    // DXL com handler
    dxlCom dxlcom;

    // Parameter not yet inserted in memory
    bool pending_parameter = false;
};

#endif