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
      if (inChar != getMemValue(ADD_ID))
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
   ERRORS:
   -1 : incorrect instruction
*/
int dxlCom::execute_command()
{

  // HEADER COMPOSITIONS OF ANSWER
  outBuffer[0] = P_HEADER_1;
  outBuffer[1] = P_HEADER_2;
  outBuffer[2] = P_HEADER_3;
  outBuffer[3] = P_RESERVED;
  rpsize = 4; // update size
  // ID
  memRead(ADD_ID, 1, outBuffer, &rpsize);

  // Skip lenght

  outBuffer[7] = 0x55;          // Status instruction
  outBuffer[8] = current_error; // Error
  rpsize = 9;                   // update size

  uint16_t parameter_size = rpsize;
  // Execute instruction

  if (instruction == INST_PING)
  {
    // Read model
    memRead(ADD_MODEL_NUMBER, 2, outBuffer, &rpsize);
    // Read firmware version 11th byte
    memRead(ADD_VERSION_OF_FIRMWARE, 1, outBuffer, &rpsize);
  }
  else if (instruction == INST_READ)
  {
    // ADDRESS READ
    uint16_t address = *(param) + (*(param + 1) << 8);
    uint16_t readsize = *(param + 2) + (*(param + 3) << 8);
    // FILL READ DATA
    memRead(address, readsize, outBuffer, &rpsize);
  }
  else if (instruction == INST_WRITE)
  {
    newparameter = true;
    uint16_t wAddress = *(param) + (*(param + 1) << 8);
    // Remove address 2 bytes from size and select the right parameters (+2 bytes)
    memWrite(wAddress, iparam - 2, (param + 2));
  }
  else
  {
    // ERROR : UNKNOWN INSTRUCTION
    outBuffer[8] = 0x02;
  }
  parameter_size = rpsize - parameter_size;
  // SIZE
  outBuffer[5] = (parameter_size + 4) & 0xff;
  outBuffer[6] = (parameter_size + 4) >> 8;

  // CRC
  unsigned short crc = crc_conversion(0, outBuffer, rpsize);
  outBuffer[rpsize] = crc & 0xFF;
  rpsize++;
  outBuffer[rpsize] = crc >> 8;
  rpsize++;
  // END PACKET

  return 0;
}
