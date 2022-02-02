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
    //INIT
    void init();
    /*
    Update the dxl core
    */
    void update();
    /*
    attach the serial port to the dxlCore
    @param  &serial Serial port
    */
    void attachSerial(HardwareSerial &serial);
    /*
    attach other Hardware
    @param1 LED_PIN
    @param2 TEMPERATURE_PIN
    @param3 INPUT_VOLTAGE_PIN
    */
    void attachHarware(byte led_pin, byte temperature_pin, byte input_voltage_pin)
    {
        _led_pin = led_pin;
        _temp_pin = temperature_pin;
        _in_voltage = input_voltage_pin;

        // Init outputs
        pinMode(_led_pin,OUTPUT);
        digitalWrite(_led_pin,LOW);
        pinMode(_temp_pin,INPUT);
        pinMode(_in_voltage,INPUT);
        
    }

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

    // *** PIN Storage
    // LED PIN
    byte _led_pin;
    // TEMPERATURE PIN
    byte _temp_pin;
    // INPUT VOLTAGE
    byte _in_voltage;


};

#endif