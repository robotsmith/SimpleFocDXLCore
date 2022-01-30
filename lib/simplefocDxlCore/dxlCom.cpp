#include "dxlCom.h"

//

dxlCom::dxlCom()
{
  // Set packet type
  inPacket.setType(0);  // Command packet
  outPacket.setType(1); // Status packet
}

void dxlCom::attach(HardwareSerial &serial)
{
  dxlserial = &serial;
}

bool dxlCom::packetAvailable()
{
  return packetIsAvailable;
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
void dxlCom::checkSerial()
{
#ifdef HALF_DUPLEX_MODE
  dxlserial->enableHalfDuplexRx();
#endif
  while (dxlserial->available())
  {
    // get the new byte:
    char inChar = (char)dxlserial->read();
    uint8_t current_error = inPacket.storeByte(inChar);
  }
}
void dxlCom::sendOutPacket()
{

  while (!outPacket.isEmpty())
    dxlserial->print((char)outPacket.pop_front());
}
/* function setId()
      @param unsigned int _id : Dynamixel device indentifier (id)
*/
void dxlCom::setId(unsigned int _id)
{
  inPacket.setId(_id);
  outPacket.setId(_id);
}

dxlPacket::dxlPacket()
{
  // Initialize
  clear();
}

// Clear packet
void dxlPacket::clear()
{
  // Initialize the exportation index
  idx = 0;
  // Initialize the parameter index
  // Initialize
  buffer[0] = (P_HEADER_1);
  buffer[1] = (P_HEADER_2);
  buffer[2] = (P_HEADER_3);
  buffer[3] = (P_RESERVED);
  buffer[4] = 0; // ID
  // buffer[5] = (0); // length
  // buffer[6] = (0); // length
  // buffer[7] = 0 push_back(0);

  if (_type == 1) // Status packet includes an ERROR byte
  {
    currentSize = 9;
  }
  else // Command packet does not includes an ERROR byte
    currentSize = 8;
}
bool dxlPacket::pushback(char element)
{
  if (idx < PACKET_BUFFER_SIZE)
  {
    buffer[idx] = element;
    idx++;
    return false;
  }

  // Issue
  return true;
}
/* Pop back info
@return the first element of the packet
*/
char dxlPacket::pop_front()
{
  if (idx < PACKET_BUFFER_SIZE)
  {
    // Increment le reading buffer
    idx++;
  }
  else
    return 0xFF;

  return buffer[idx];
}
/* Check if the packet is empty or not
@return true if the packet is empty
*/
bool dxlPacket::isEmpty()
{
  // If the reading buffer is > to the buffer size then the packet is empty
  if (idx > currentSize)
    return true;
  // Not empty
  return false;
}
uint8_t dxlPacket::storeByte(byte b)
{

  // Add the byte to the buffer
  buffer[currentSize] = b;

  bool abortPacket = false;
  if ((currentSize == 0 && (b != P_HEADER_1)) ||
      (currentSize == 1 && (b != P_HEADER_2)) ||
      (currentSize == 2 && (b != P_HEADER_3)) ||
      (currentSize == 3 && (b != P_RESERVED)))
  {
    abortPacket = true;
  }
  else if (currentSize == 5)
  {
    ExpectedSize = b;
  }
  else if (currentSize == 6)
  {
    ExpectedSize = ExpectedSize + (unsigned int)(b << 8);
  }
  else if (currentSize > 7)
  {
    //[HEADERS ID
    if (currentSize - 7 < ExpectedSize - 2)
    { // this is a parameter
    }
    else
    { // we should be in CRC here
      if (currentSize - 8 - (ExpectedSize - 3) == 0)
      {
        crc = b;
      }
      else if (currentSize - 8 - (ExpectedSize - 3) == 1)
      {
        crc += b << 8;

        unsigned short crc = crc_conversion(0, buffer, currentSize - 1);

        if (crc == crc)
        {
          _complete = true;
          //Complete informations
        
        }
        else
          abortPacket = true;
      }
    }
  }
  if (abortPacket)
  { // IGNORE PACKET
    clear();
  }
  else
  {
    currentSize++;
    if (currentSize > PACKET_BUFFER_SIZE)
      clear();
  }
}
