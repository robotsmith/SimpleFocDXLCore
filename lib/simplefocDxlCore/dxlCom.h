#ifndef DXLCOM_H
#define DXLCOM_H

// @todo: faire une classe packet

#include "Arduino.h"
#include "dxlMemory.h"
#include "dxlUtils.h"

// COM
#define HALF_DUPLEX_MODE

// COM BUFFERS
#define INBUFSIZE 100
#define OUTBUFSIZE 255
#define PARAMSIZE 16

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

class dxlCom
{
public:
  // FUNCTIONS

  dxlCom(HardwareSerial &serial, dxlMemory *_dxlmem);

  // VARIABLES
  HardwareSerial *com_port = nullptr;
  dxlMemory *dxlmem = nullptr;

  // Input packet management
  bool packetAvailable = false; // if packet is available
  uint8_t instruction;
  unsigned int packet_len; // packet lenght
  unsigned short packetcrc;

  // Output packet management
  void makeNewPacket();
  void updatepacketParameter();
  void endpacket();

  // IN/OUT BUFFERS
  unsigned char inBuffer[INBUFSIZE];
  uint16_t cindex; // current input buffer index
  unsigned char outBuffer[OUTBUFSIZE];
  uint16_t rpsize; // current outbuffer size
  unsigned char param[PARAMSIZE];
  uint16_t iparam; // current size dxl parameter index

  // Device Identifier
  unsigned int id;
  void setId(unsigned int _id);
  // ERRORS
  uint8_t current_error; // current error code
  // DATA
  /*

  */

  // COMMUNICATION

  /*
    EXECUTE MASTER PACKET COMMAND
  */
  int execute_command();
  int update();
  void SERevent();
};

/*
class statusPacket
     You MUST respect the following protocol: constructor + addID + addError (optionnal) + endpacket
     */
class statusPacket
{
public:
  statusPacket(unsigned char *_buffer, unsigned char _id = 0);

  void addId(unsigned char id);
  void addError(unsigned char error);

  void endPacket();

  //  Vars
  unsigned char *buffer;
  uint16_t parameter_size;
  uint16_t size;
};

#endif