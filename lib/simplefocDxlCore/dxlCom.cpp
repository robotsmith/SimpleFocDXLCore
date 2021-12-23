#include "dxlCom.h"

//

dxlCom::dxlCom(HardwareSerial &serial, dxlMemory *_dxlmem)
{
  com_port = &serial;
  dxlmem = _dxlmem;

  inPacket.setbuffer(inBuffer);
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
  inPacket.setId(id);
}
/* function storeByteInPacket
      @param byte b : a byte to check
      Check and if correct store byte
*/

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
    uint8_t current_error = inPacket.storeByte(inChar);
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
function setError
     @param unsigned char error : Dynamixel packet error flag
*/
void statusPacket::setError(unsigned char error)
{
  buffer[8] = error; // Error
}

/*
function setId
     @param unsigned char id  : Dynamixel id
*/
void statusPacket::setId(unsigned char id)
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
/* Class constructor
 */
incomingPacket::incomingPacket(unsigned char *_buffer, unsigned char _id = 0)
{
  buffer = _buffer;
  id = _id;
  header[4] = id;
  packetAvailable = false;
}

/*
function incomingPacket
@param byte b : byte inserted
@return error : {0 : no error; 1 : wrong header, 2 : wrong CRC}
*/
uint8_t incomingPacket::storeByte(byte b)
{
  // store in buffer
  buffer[size] = b;

  // check
  if (size < 5) // still in header
  {
    if (b == header[size])
    {
      size++;
      return 0; // ok
    }
    else
    {
      size = 0; // reinit size
      return 1; // wrong header
    }
  }
  else if (size == 5) // packet lenght L
  {
    packet_lenght = b;
  }
  else if (size == 6) // packet lenght H
  {
    packet_lenght += (unsigned int)(b << 8);
  }
  else if (size == 7) // Instruction
  {
    instruction = b;
  }
  else
  {
    //[HEADERS ID
    if (size - 7 < packet_lenght - 2)
    { // this is a parameter
      param[iparam] = b;
      iparam++;
    }
    else
    { // we should be in CRC here
      if (size - 5 - packet_lenght == 0)
      {
        packetcrc = b;
      }
      else if (size - 8 - (packet_lenght - 3) == 1)
      {
        packetcrc += b << 8;

        unsigned short crc = crc_conversion(0, buffer, size - 1);

        if (crc == packetcrc)
        {
          packetAvailable = true;
        }
        else // CRC ISSUE
        {
          size = 0;
          return 2;
        }
      }
    }
  }
  size++;

  return 0;
}

/*
function available
@return availability : {false : not available , true : available}
Also clear packetAvailable flag
*/
bool incomingPacket::available()
{
  if (available)
  {
    packetAvailable = false;
    return true;
  }
  return false;
}

void incomingPacket::setbuffer(unsigned char *_buffer)
{
  buffer = _buffer;
}
void incomingPacket::setId(unsigned char _id)
{
  id = _id;
}