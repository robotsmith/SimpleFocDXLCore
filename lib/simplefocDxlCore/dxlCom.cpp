#include "dxlCom.h"

//

dxlCom::dxlCom(HardwareSerial &serial, dxlMemory *_dxlmem)
{
  com_port = &serial;
  dxlmem = _dxlmem;
}
/*
bool dxlCom::parameter_available()
{
  if (newparameter)
  {
    newparameter = false;
    return true;
  }

  return false;
}
*/
int dxlCom::update()
{

  if (packetAvailable)
  {
    // Init size of response Packet
    rpsize = 0;

    // RESET BUFFER
    if (packetAvailable)
    {
      execute_command();

      for (uint16_t i = 0; i < rpsize; i++)
        com_port->print((char)outBuffer[i]);

      // Reinit
      packetAvailable = false;
      cindex = 0;
      iparam = 0;
      packetcrc = 0;
    }
  }

  return 0;
}
/* function setId()
      @param unsigned int _id : Dynamixel device indentifier (id)
*/
void dxlCom::setId(unsigned int _id)
{
  id = _id;
}
// Get incomming packet
void dxlCom::SERevent()
{
#ifdef HALF_DUPLEX_MODE
  com_port->enableHalfDuplexRx();
#endif
  while (com_port->available())
  {

    // get the new byte:
    char inChar = (char)com_port->read();

    inBuffer[cindex] = inChar;

    // com_port->print((char)cindex);
    bool abortPacket = false;
    switch (cindex)
    {
    case 0:
      if (inChar != P_HEADER_1)
      {
        abortPacket = true;
      }

      break;
    case 1:
      if (inChar != P_HEADER_2)
        abortPacket = true;

      break;
    case 2:
      if (inChar != P_HEADER_3)
        abortPacket = true;

      break;
    case 3:
      if (inChar != P_RESERVED)
        abortPacket = true;
      break;
    case 4:
      if (inChar != id)
        abortPacket = true;
      /* com_port->print((char)0xAA);
        com_port->print((char)inChar);
        com_port->print((char)getMemValue(ADD_ID));
        com_port->print((char)0xAA);*/
      break;
    case 5:
      packet_len = inChar;
      break;
    case 6:
      packet_len = packet_len + (unsigned int)(inChar << 8);
      break;
    case 7: // INSTRUCTION
      instruction = inChar;
      break;
    default:
      break;
    }
    if (cindex > 7)
    {
      //[HEADERS ID
      if (cindex - 7 < packet_len - 2)
      { // this is a parameter
        param[iparam] = inChar;
        iparam++;
      }
      else
      { // we should be in CRC here
        if (cindex - 8 - (packet_len - 3) == 0)
        {
          packetcrc = inChar;
        }
        else if (cindex - 8 - (packet_len - 3) == 1)
        {
          packetcrc += inChar << 8;

          unsigned short crc = crc_conversion(0, inBuffer, cindex - 1);

          if (crc == packetcrc)
          {
            packetAvailable = true;
          }
          else
            abortPacket = true;
        }
      }
    }
    if (abortPacket)
    {             // IGNORE PACKET
      cindex = 0; // restart
      iparam = 0;
      packet_len = 0;
      instruction = 0;
    }
    else
    {
      cindex++;
      if (cindex > INBUFSIZE)
        cindex = 0;
    }
  }
}

/*
class constructor statusPacket
     @param unsigned char *buffer  : where is store the memory buffer
*/
statusPacket::statusPacket(unsigned char *_buffer, unsigned char _id = 0)
{
  // Store buffer
  buffer = _buffer;
  // HEADER COMPOSITIONS OF ANSWER
  buffer[0] = P_HEADER_1;
  buffer[1] = P_HEADER_2;
  buffer[2] = P_HEADER_3;
  buffer[3] = P_RESERVED;
  buffer[4] = _id;
  // Skip ID
  //  Skip lenght

  buffer[7] = 0x55; // Status instruction

  size = 9;              // update size
  parameter_size = size; // get current size in order to check the difference afterward
}
/*
function addError
     @param unsigned char error : Dynamixel packet error flag
*/
void statusPacket::addError(unsigned char error)
{
  buffer[8] = error; // Error
}

/*
function addId
     @param unsigned char id  : Dynamixel id
*/
void statusPacket::addId(unsigned char id)
{
  buffer[4] = id;
}

/*
function endPacket

*/
void statusPacket::endPacket()
{
  // Get the parameter size
  parameter_size = size - parameter_size;
  // SIZE
  buffer[5] = (parameter_size + 4) & 0xff;
  buffer[6] = (parameter_size + 4) >> 8;

  // CRC
  unsigned short crc = crc_conversion(0, buffer, size);
  buffer[size] = crc & 0xFF;
  size++;
  buffer[size] = crc >> 8;
  size++;
}