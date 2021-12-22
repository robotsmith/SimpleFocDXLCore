#ifndef DYNAMIXEL_PROTOCOL_V2_H
#define DYNAMIXEL_PROTOCOL_V2_H
#include <common/dynamixel_storage.h>
#include <EEPROM.h>
#include "Arduino.h"

//COM
#define HALF_DUPLEX_MODE


// EEPROM
#define EEPROM_ENABLED

#define EEPROM_LENGTH E2END
#define EEPROM_FIRST_ADDRESS 10
// EEPROM 0 ADDRESS is initialized flag

#define EEPROM_ADDRESSES 64

// RAM Storage
#define DYN_DATA_SIZE 150

// COM BUFFERS
#define INBUFSIZE 100
#define OUTBUFSIZE 255
#define PARAMSIZE 16

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

//— Instruction —
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_STATUS 0x55
#define INST_RESET 0x06
#define INST_DIGITAL_RESET 0x07
#define INST_SYSTEM_READ 0x0C
#define INST_SYSTEM_WRITE 0x0D
#define INST_SYNC_WRITE 0x83
#define INST_SYNC_REG_WRITE 0x84

#define P_HEADER_1 0xFF
#define P_HEADER_2 0xFF
#define P_HEADER_3 0xFD
#define P_RESERVED 0x00

class dynamixelDevice
{
public:
  dynamixelDevice(HardwareSerial &serial);
  // DATA
  uint8_t *data;
  HardwareSerial *com_port = nullptr;
  uint8_t current_error;
  bool parameter_available();

  uint16_t iparam;
  unsigned int packet_len;
  unsigned short packetcrc;
  bool packetAvailable = false;
  uint16_t cindex;
  uint16_t rpsize;
  uint8_t instruction;
  bool newparameter;
  uint8_t dyn_data[DYN_DATA_SIZE];
  unsigned char inBuffer[INBUFSIZE];
  unsigned char outBuffer[OUTBUFSIZE];
  unsigned char param[PARAMSIZE];
  // FUNCTIONS

  // COMMUNICATION

  /*
    EXECUTE MASTER PACKET COMMAND
  */
  int execute_command();
  int update();
  void SERevent();
  /*
     CRC CHECK
     Note:  http://support.robotis.com/en/product/actuator/dynamixel_pro/communication/crc.htm
  */
  unsigned short packet_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

  // MEMORY MANAGEMENT (linked to *data)
  void load_default();
  void initMem();

  uint8_t getMemValue(uint16_t address);
  uint32_t getMemValue(uint16_t address, uint8_t len);
  void memWrite(uint16_t address, uint8_t value);
  void memWrite(uint16_t address, int value);
  void memWrite(uint16_t address, uint16_t value);
  void memWrite(uint16_t address, uint32_t value);
  // memory write with data segment
  void memWrite(uint16_t address, uint16_t wSize, uint8_t *value);
  // READ Data from data_dyn
  void memRead(uint16_t address, uint16_t *readSize, uint8_t *outData, uint16_t *outDataSize);
  void memRead(uint16_t address, uint16_t readSize, uint8_t *outData, uint16_t *outDataSize);

  // EEPROM MANAGEMENT
  uint8_t loadEEPROM();
  bool checkDataDynCRC();
  uint8_t writeEEPROM(uint16_t address, uint8_t value);
  void data2EEPROM();
  void storeDataDynCRC();
};

#endif
