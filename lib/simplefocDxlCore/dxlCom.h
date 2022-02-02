#ifndef DXLCOM_H
#define DXLCOM_H

// @todo: faire une classe packet

#include <Arduino.h>
#include <dxlMemory.h>
#include <dxlUtils.h>
#include <EEPROM.h>
// COM
#define HALF_DUPLEX_MODE

// COM BUFFERS
#define PACKET_BUFFER_SIZE 180

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

// PACKET MANAGEMENT
#define PARAM_GAP 8
/*
  This class is a packet description with everything to decode or recode
*/
class dxlPacket
{
public:
  // Constructor
  dxlPacket();
  // Clear packet
  void clear();

  /* Return true if the packet is complete
  @return true if the packet is complete
  */
  bool isComplete() { return _complete; }
  /*
  Store a byte in the packet. The goal here is to decode an incoming message
  @param b byte to decode
  */
  uint8_t storeByte(byte b);
  /*
  To check what kind of packet it is
  @return type 0 : command packet 1 : status packet
  */
  uint8_t getType() { return _type; }
  /*
  Define the type of packet
  @param type 0 : command packet 1 : status packet
  */
  void setType(uint8_t type) { _type = type; }

  // Data raw buffer
  uint8_t buffer[PACKET_BUFFER_SIZE];

  /* Push data into last element of the buffer
  @param element data to push
  @return error true if data does not fit
  */
  bool pushback(char element);
  /* Pop back info
  @return the first element of the packet
  */
  char pop_front();
  /* Check if the packet is empty or not
  @return true if the packet is empty
  */
  bool isEmpty();
  /* Get the size of the buffer
  @return size : length of the buffer
  */
  uint16_t size() { return currentSize; }
  /* Set ID of the DXL device
  @param id ID of the packet's DXL device
  */
  void setId(uint8_t id) { buffer[4] = id; }
  /* Get current packet's device ID of the packet
  @return id of the packet's device ID
  */
  uint8_t getId() { return buffer[4]; }
  /* Close packet and fill with informations
   */
  void close();
  /* Get instruction of the packet
  @return instruction
  */
  uint8_t instruction() { return buffer[7]; }
  // size of the buffer
  uint16_t currentSize;

private:
  // Packet type: type 0 = command packet / 1 = status packet
  uint8_t _type;
  // is complete flag
  bool _complete;

  // Current reading index
  uint16_t idx;

  // Expected size
  uint16_t ExpectedSize;
  // crc
  uint16_t crc;
};

// inline unsigned char param[PARAMSIZE];
class dxlCom
{
public:
  // FUNCTIONS

  // Constructor
  dxlCom();

  /* Attach serial port to the communication handler
   */
  void attach(HardwareSerial &serial);
  /* Packet availability
  @return true if available
  */
  bool packetAvailable();
  /* Send the status packet will all informations
   */
  void sendOutPacket();
  /* Close Status packet, fill with all informations needed to be correct
   */
  void closeStatusPacket();
  /* Set ID of the attached device
  @param id ID of the packet
  */
  void setId(unsigned int _id);

  // *** VARIABLES

  // Hardware Serial stream port
  HardwareSerial *dxlserial = nullptr;

  // check serial data
  void checkSerial();

  // Incomming packet
  dxlPacket inPacket;
  // Output packet
  dxlPacket outPacket;

private:
  bool packetIsAvailable = false;

  /*
  //dxlMemory *dxlmem = nullptr;

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

  void setId(unsigned int _id);
  // ERRORS
  uint8_t current_error; // current error code
  // DATA
  incomingPacket inPacket;


  // COMMUNICATION

$
    //EXECUTE MASTER PACKET COMMAND

  int execute_command();
  int update();
  void SERevent();

private:
  // Device Identifier
  unsigned int id;
  */
};

/*
class statusPacket
     You MUST respect the following protocol: constructor + addID + addError (optionnal) + endpacket

class statusPacket
{
public:
  statusPacket(unsigned char *_buffer, unsigned char _id);

  void setId(unsigned char id);
  void setError(unsigned char error);

  void endPacket();

  //  Vars
  unsigned char *buffer;
  uint16_t parameter_size;
  uint16_t size;
};

class incomingPacket
{

public:
  incomingPacket(unsigned char *_buffer, unsigned char _id);
  incomingPacket()
  {
  }
  void setbuffer(unsigned char *_buffer);
  void setId(unsigned char _id);
  uint8_t storeByte(byte b);
  //  Vars
  unsigned char *buffer;
  uint16_t parameter_size;
  uint16_t size;

  uint16_t packet_lenght;

  unsigned short packetcrc;
  bool packetAvailable;
  unsigned char param[PARAMSIZE];
  uint16_t iparam; // current size dxl parameter index
  bool available();
  // Packet structure
  uint8_t id;
  uint8_t instruction;
};
*/
#endif
