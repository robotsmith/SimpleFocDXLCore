#ifndef SERVO_SIMPLEFOCDXLCORE_H
#define SERVO_SIMPLEFOCDXLCORE_H

#include "Arduino.h"
#include <BLDCMotor.h>
#include <dxlCom.h>
#include <dxlMemory.h>
#define EEPROM_ENABLED


// LED BLINKING TIMEOUT IN FAULT MODE
#define ERROR_BLINKING_TIMEOUT 200

class simplefocDxlCore
{
public:
    // *** Functions

    // Constructor
    simplefocDxlCore(BLDCMotor *_motor);
    // INIT
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

    void attachHarware(byte nrst_drv_pin,
                       byte nslp_drv_pin,
                       byte fault_drv_pin,
                       byte led_pin,
                       byte temperature_pin,
                       byte input_voltage_pin);

    // Factory reset memory
    void factoryResetMem();

    // load default parameters in memory
    void loadDefaultMem();

    // Execute the command in the DXL packet
    void executePacketCommand();

    // Blink status led
    void blinkStatus(uint8_t nb, uint16_t delay_);

    // SET FAULT MODE
    // @param mode : 1=fault 0=normal
    void setFaultMode(bool mode);

    // *** Variables
private:
    // *** Functions

    // Refresh data from motor and update DXL memory in RAM
    void refreshRAMData();

    // Refresh Current data from Device
    void refreshPresentData();
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


    // DRIVER SPECIAL PINS

    // NOT RESET PIN
    byte _nrst_drv_pin;
    // NOT SLEEP PIN
    byte _nslp_drv_pin;
    // FAULT PIN
    byte _fault_drv_pin;
    // Time record
    long time_record;
    // Fault_mode_time
    long fault_mode_time;

    // Refresh Counter
    uint8_t rcount = 0;

    // Hardware Error flag
    byte hardware_error;

    // Fault Mode
    bool fault_mode = false;

};

#endif