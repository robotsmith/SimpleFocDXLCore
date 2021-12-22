#include "dynamixel_protocol_V2.h"

#include "Arduino.h"

//

dynamixelDevice::dynamixelDevice(HardwareSerial &serial)
{
  com_port = &serial;

  // MEMORY INIT
  data = dyn_data; // link data
  initMem();

  // Init error flag
  current_error = 0;

  // LOAD EEPROM
  loadEEPROM();

  // EEPROM CRC IS CORRECT?
  // If not could be because first init or memory issue

  if (!checkDataDynCRC())
  {

    // Erase RAM memory
    initMem();
    // Load default parameters
    load_default();
    // Store into EEPROM
    data2EEPROM();
    // Compute and store EEPROM CRC

    storeDataDynCRC();
    current_error = 1;
  }
  /*
    load_default();
    data2EEPROM();
  */
  // INIT PACKET COM INFO
  packetAvailable = false;
  cindex = 0;
  iparam = 0;
  packetcrc = 0;
  packet_len = 0;
  newparameter = false;
}
bool dynamixelDevice::parameter_available()
{
  if (newparameter)
  {
    newparameter = false;
    return true;
  }

  return false;
}

int dynamixelDevice::update()
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
void dynamixelDevice::SERevent()
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

          unsigned short crc = packet_crc(0, inBuffer, cindex - 1);

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

void dynamixelDevice::initMem()
{
  for (uint16_t i = 0; i < DYN_DATA_SIZE; i++)
    *(data + i) = 0;
}
uint8_t dynamixelDevice::getMemValue(uint16_t address)
{
  return *(data + address);
}
uint32_t dynamixelDevice::getMemValue(uint16_t address, uint8_t len)
{
  uint32_t tmp = 0;
  if (address + len > DYN_DATA_SIZE)
    return 0;
  for (uint8_t i = 0; i < len; i++)
  {
    tmp = tmp | (*(data + address + i) << (8 * i));
  }

  return tmp;
}
void dynamixelDevice::memWrite(uint16_t address, uint8_t value)
{
  if (address + 1 < DYN_DATA_SIZE)
  {
    *(data + address) = value & 0xff;

    // IF ROM data, save into EEPROM
#ifdef EEPROM_ENABLED
    if (address < EEPROM_ADDRESSES)
    {
      writeEEPROM(address, value);
    }
#endif
  }
}
void dynamixelDevice::memWrite(uint16_t address, int value)
{
  memWrite(address, (uint8_t)value);
}
void dynamixelDevice::memWrite(uint16_t address, uint16_t value)
{
  if (address + 2 < DYN_DATA_SIZE)
  {
    for (uint8_t i = 0; i < 2; i++)
    {
      memWrite(address + i, (uint8_t)((value >> (8 * i)) & 0xff));
    }
  }
}
void dynamixelDevice::memWrite(uint16_t address, uint32_t value)
{
  if (address + 4 < DYN_DATA_SIZE)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      memWrite(address + i, (uint8_t)((value >> (8 * i)) & 0xff));
    }
  }
}
// memory write with data segment
void dynamixelDevice::memWrite(uint16_t address, uint16_t wSize, uint8_t *value)
{
  if (address + (wSize) < DYN_DATA_SIZE)
  {
    for (uint8_t i = 0; i < (wSize); i++)
    {
      memWrite(address + i, (uint8_t) * (value + i));
    }
  }
}
// READ Data from data_dyn
void dynamixelDevice::memRead(uint16_t address, uint16_t *readSize, uint8_t *outData, uint16_t *outDataSize)
{
  // Resize max lenght
  uint16_t sizetoread = *readSize;
  if (address + *readSize >= DYN_DATA_SIZE)
  {
    sizetoread = DYN_DATA_SIZE - address;
  }
  else if (OUTBUFSIZE > sizetoread + *outDataSize)
    sizetoread = OUTBUFSIZE - *outDataSize;
  for (uint16_t i = 0; i < sizetoread; i++)
  {
    *(outData + *outDataSize) = *(data + i + address);
    *(outDataSize) += 1;
  }
}
void dynamixelDevice::memRead(uint16_t address, uint16_t readSize, uint8_t *outData, uint16_t *outDataSize)
{
  // Resize max lenght
  uint16_t sizetoread = readSize;
  if (address + readSize >= DYN_DATA_SIZE)
  {
    sizetoread = DYN_DATA_SIZE - address;
  }
  else if (OUTBUFSIZE > sizetoread + *outDataSize)
    sizetoread = OUTBUFSIZE - *outDataSize;
  for (uint16_t i = 0; i < sizetoread; i++)
  {
    *(outData + *outDataSize) = *(data + i + address);
    *(outDataSize) += 1;
  }
}

/*
   ERRORS:
   -1 : incorrect instruction
*/
int dynamixelDevice::execute_command()
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
  unsigned short crc = packet_crc(0, outBuffer, rpsize);
  outBuffer[rpsize] = crc & 0xFF;
  rpsize++;
  outBuffer[rpsize] = crc >> 8;
  rpsize++;
  // END PACKET

  return 0;
}

/*
   CRC
*/

unsigned short dynamixelDevice::packet_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) // http://support.robotis.com/en/product/actuator/dynamixel_pro/communication/crc.htm
{
  unsigned short i, j;
  unsigned short crc_table[256] = {
      0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
      0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
      0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
      0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
      0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
      0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
      0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
      0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
      0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
      0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
      0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
      0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
      0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
      0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
      0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
      0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
      0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
      0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
      0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
      0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
      0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
      0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
      0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
      0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
      0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
      0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
      0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
      0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
      0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}
void dynamixelDevice::load_default()
{

  // Clear data
  for (uint16_t i = 0; i < DYN_DATA_SIZE; i++)
  {
    dyn_data[i] = 0;
  }
  // MODEL NUMBER
  memWrite(ADD_MODEL_NUMBER, (uint16_t)0x0406); // XM430
  // MODEL FIRMWARE
  memWrite(ADD_VERSION_OF_FIRMWARE, (uint8_t)0x01);
  // ID
  memWrite(ADD_ID, (uint8_t)0x00);
  // BAUDRATE
  memWrite(ADD_BAUDRATE, (uint8_t)0x02); // 115200
  // Drive mode
  memWrite(ADD_DRIVE_MODE, (uint8_t)0x08); // TORQUE ON GOAL UPDATE
  // control mode
  memWrite(ADD_OPERATING_MODE, (uint8_t)0x03); // POSITION CONTROL
  // protocol mode
  memWrite(ADD_PROTOCOL_VERSION, (uint8_t)0x02); // PROTOCOL 2

  // MAX temperature
  memWrite(ADD_TEMPERATURE_LIMIT, (uint8_t)0x50);
  // Max voltage
  memWrite(ADD_MAX_VOLTAGE_LIMIT, (uint16_t)0x0082); // 13V
  // Min voltage
  memWrite(ADD_MIN_VOLTAGE_LIMIT, (uint16_t)0x0050); // 8V

  // PWM limit

  // CURRENT LIMIT

  // ACCELERATION LIMIT

  // VELOCITY LIMIT

  // POSITION LIMIT
  memWrite(ADD_MAX_POSITION_LIMIT, (uint16_t)0xFFFFF);
  memWrite(ADD_MIN_POSITION_LIMIT, (uint16_t)0x0);
  // TORQUE
  memWrite(ADD_TORQUE_ENABLE, (uint8_t)0x00); // 58.4 rev/min
  // RAM
}

// EEPROM MANAGEMENT
uint8_t dynamixelDevice::loadEEPROM()
{
  eeprom_buffer_fill();
  for (uint16_t i = 0; i < DYN_DATA_SIZE; i++)
  {
    if (i + EEPROM_FIRST_ADDRESS > EEPROM_LENGTH)
      return 0x07;
    //*(data + i) = EEPROM.read(i + EEPROM_FIRST_ADDRESS);
    *(data + i) = eeprom_buffered_read_byte(i + EEPROM_FIRST_ADDRESS);
  }
  return 0;
}

uint8_t dynamixelDevice::writeEEPROM(uint16_t address, uint8_t value)
{
  if (address > EEPROM_LENGTH)
    return 0x07;
  // EEPROM.write(address + EEPROM_FIRST_ADDRESS, value);
  EEPROM[address + EEPROM_FIRST_ADDRESS] = value;
  /*eeprom[_buffered_write_byte(address + EEPROM_FIRST_ADDRESS, value);
    eeprom_buffer_flush();*/
  // Compute CRC
  storeDataDynCRC();
  return 0;
}
void dynamixelDevice::data2EEPROM()
{
  for (uint16_t i = 0; i < DYN_DATA_SIZE; i++)
  {
    // EEPROM.update(i + EEPROM_FIRST_ADDRESS, *(data + i));
    eeprom_buffered_write_byte(i + EEPROM_FIRST_ADDRESS, *(data + i));
  }
  // FLAG INITIALIZED
  // EEPROM.update(0, 0x01);
  eeprom_buffered_write_byte(0, 0x01);
  eeprom_buffer_flush();
  // Compute CRC
  storeDataDynCRC();
}

void dynamixelDevice::storeDataDynCRC()
{

  unsigned short crc = packet_crc(0, dyn_data, EEPROM_ADDRESSES - 1);

  // WRITE CRC
  EEPROM[EEPROM_CRCL] = crc & 0xFF;
  EEPROM[EEPROM_CRCH] = crc >> 8;
}
bool dynamixelDevice::checkDataDynCRC()
{
  eeprom_buffer_fill();
  // GET CRC
  unsigned short crc = packet_crc(0, dyn_data, EEPROM_ADDRESSES - 1); // CRC WITHOUT CRC VALUE IN EEPROM (beginning after EEPROM_FIRST_ADDRESS )
  char crcl = eeprom_buffered_read_byte(EEPROM_CRCL);
  char crch = eeprom_buffered_read_byte(EEPROM_CRCH);
  unsigned short crc_in_eeprom = crcl + (crch << 8);
  // EEPROM GOT THE RIGHT CRC ??
  if (crc == crc_in_eeprom)
    return true;

  // ELSE
  return false;
}
