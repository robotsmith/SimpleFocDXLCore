#ifndef DXLMEMORY_H
#define DXLMEMORY_H
#include "Arduino.h"
#include <EEPROM.h>

// EEPROM
#define EEPROM_ENABLED

#define EEPROM_LENGTH E2END
#define EEPROM_FIRST_ADDRESS 1024
#define EEPROM_CRCL EEPROM_FIRST_ADDRESS + 150
#define EEPROM_CRCH EEPROM_FIRST_ADDRESS + 151
#define EEPROM_ADDRESSES 64

// RAM Storage
#define DXL_DATA_SIZE 150

// ADDRESSES
// DATA ADDRESSES
#define ADD_MODEL_NUMBER 0
#define ADD_MODEL_INFORMATION 2
#define ADD_VERSION_OF_FIRMWARE 6
#define ADD_ID 7
#define ADD_BAUDRATE 8
#define ADD_RETURN_DELAY_TIME 9
#define ADD_DRIVE_MODE 10
#define ADD_OPERATING_MODE 11
#define ADD_SECONDARY_ID 12
#define ADD_PROTOCOL_VERSION 13
#define ADD_HOMING_OFFSET 20
#define ADD_MOVING_THRESHOLD 24
#define ADD_TEMPERATURE_LIMIT 31
#define ADD_MAX_VOLTAGE_LIMIT 32
#define ADD_MIN_VOLTAGE_LIMIT 34
#define ADD_PWM_LIMIT 36
#define ADD_CURRENT_LIMIT 38
#define ADD_ACCELERATION_LIMIT 40
#define ADD_VELOCITY_LIMIT 44
#define ADD_MAX_POSITION_LIMIT 48
#define ADD_MIN_POSITION_LIMIT 52
#define ADD_SHUTDOWN 63
// RAM
#define ADD_TORQUE_ENABLE 64
#define ADD_LED 65
#define ADD_STATUS_RETURN_LEVEL 68
#define ADD_REGISTERED_INSTRUCTION 69
#define ADD_HARDWARE_ERROR_STATUS 70
#define ADD_VELOCITY_I_GAIN 76
#define ADD_VELOCITY_P_GAIN 78
#define ADD_POSITION_D_GAIN 80
#define ADD_POSITION_I_GAIN 82
#define ADD_POSITION_P_GAIN 84
#define ADD_FEEDFORWARD_ACCELERATION_GAIN 88
#define ADD_FEEDFORWARD_VELOCITY_GAIN 90
#define ADD_BUS_WATCHDOG 98
#define ADD_GOAL_PWM 100
#define ADD_GOAL_CURRENT 102
#define ADD_GOAL_VELOCITY 104
#define ADD_PROFILE_ACCELERATION 108
#define ADD_PROFILE_VELOCITY 112
#define ADD_GOAL_POSITION 116
#define ADD_REALTIME_TICK 120
#define ADD_MOVING 122
#define ADD_MOVING_STATUS 123
#define ADD_PRESENT_PWM 124
#define ADD_PRESENT_CURRENT 126
#define ADD_PRESENT_VELOCITY 128
#define ADD_PRESENT_POSITION 132
#define ADD_VELOCITY_TRAJECTORY 136
#define ADD_POSITION_TRAJECTORY 140
#define ADD_PRESENT_INPUT_VOLTAGE 144
#define ADD_PRESENT_TEMPERATURE 146

/*
#define ADD_EXTERNAL_PORT_DATA_1 152

#define ADD_EXTERNAL_PORT_DATA_2 154
#define ADD_EXTERNAL_PORT_DATA_3 156
#define ADD_INDIRECT_ADDR_1 168
#define ADD_INDIRECT_DATA_1 224
*/
//#define ADD_INDIRECT_ADDR_29 578

/*
Change the ram and flash of the Dynamixel device
*/
class dxlMemory
{
public:
    // *** Functions
    // Contructor of dxl memory
    dxlMemory();

    /* Load parameters from EEPROM or default
    @return 1 if memory is not correct (CRC not correct)
    */
    bool load();
    /* Check EEPROM integrity with CRC method
    @return true if there is an error on CRC read
    */
    bool CheckEEPROM_CRC();
    /* Load EEPROMto memory
    @return true if there is an error on loading
    */
    bool loadEEPROM();

    // Clear Memory
    void clear();

    uint8_t getValueFromDxlData(uint16_t address);
    uint32_t getValueFromDxlData(uint16_t address, uint8_t len);
    void store(uint16_t address, uint8_t value);
    void store(uint16_t address, int value);
    void store(uint16_t address, uint16_t value);
    void store(uint16_t address, uint32_t value);
    // memory write with data segment
    void store(uint16_t address, uint16_t wSize, uint8_t *value);
    // READ Data from data_dyn
    void memRead(uint16_t address, uint16_t readSize, uint8_t *outData, uint16_t *outDataSize, uint16_t maxSize);
    void memRead(uint16_t address, uint16_t *readSize, uint8_t *outData, uint16_t *outDataSize, uint16_t maxSize);

    // EEPROM MANAGEMENT

    uint8_t writeEEPROM(uint16_t address, uint8_t value);
    void data2EEPROM();
    void storeDataDynCRC();

    // *** Vars
    uint8_t dyn_data[DXL_DATA_SIZE];
};
#endif