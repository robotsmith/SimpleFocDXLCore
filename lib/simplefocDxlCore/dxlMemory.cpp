#include "dxlMemory.h"

dxlMemory::dxlMemory() { clear(); }

bool dxlMemory::load()
{
  return loadEEPROM();
}

/*
bool dxlMemory::CheckEEPROM_CRC()
{
  eeprom_buffer_fill();
  // GET CRC
  unsigned short crc = crc_conversion(0, DXL_data, EEPROM_ADDRESSES - 1); // CRC WITHOUT CRC VALUE IN EEPROM (beginning after EEPROM_FIRST_ADDRESS )
  char crcl = eeprom_buffered_read_byte(EEPROM_CRCL);
  char crch = eeprom_buffered_read_byte(EEPROM_CRCH);
  unsigned short crc_in_eeprom = crcl + (crch << 8);
  // EEPROM GOT THE RIGHT CRC ??
  if ((crc == crc_in_eeprom) && (crc != 0))
    return false;

  // ELSE
  return true;
}
*/
// EEPROM MANAGEMENT
bool dxlMemory::loadEEPROM()
{

  eeprom_buffer_fill();
  for (uint16_t i = 0; i < EEPROM_ADDRESSES; i++)
  {
    if (i + EEPROM_FIRST_ADDRESS > EEPROM_LENGTH)
      return true;
    //*(data + i) = EEPROM.read(i + EEPROM_FIRST_ADDRESS);
    *(DXL_data + i) = eeprom_buffered_read_byte(i + EEPROM_FIRST_ADDRESS);
  }
  // Check CRC
  unsigned short crc = crc_conversion(0, DXL_data, EEPROM_ADDRESSES - 1); // CRC WITHOUT CRC VALUE IN EEPROM (beginning after EEPROM_FIRST_ADDRESS )
  char crcl = eeprom_buffered_read_byte(EEPROM_CRCL);
  char crch = eeprom_buffered_read_byte(EEPROM_CRCH);
  unsigned short crc_in_eeprom = crcl + (crch << 8);
  // EEPROM GOT THE RIGHT CRC ??
  if ((crc != crc_in_eeprom) || (crc == 0))
    return true; // ISSUE

  // OK
  return false;
}

void dxlMemory::data2EEPROM()
{
  for (uint16_t i = 0; i < EEPROM_ADDRESSES; i++)
  {
    // EEPROM.update(i + EEPROM_FIRST_ADDRESS, *(data + i));
    eeprom_buffered_write_byte(i + EEPROM_FIRST_ADDRESS, *(DXL_data + i));
  }

  // Compute and store CRC
  uint16_t crc = DataDynCRC();
  eeprom_buffered_write_byte(EEPROM_CRCL, crc & 0xFF);
  eeprom_buffered_write_byte(EEPROM_CRCH, crc >> 8);

  // Store into EEPROM
  eeprom_buffer_flush();
}

void dxlMemory::clear()
{
  for (uint16_t i = 0; i < DXL_DATA_SIZE; i++)
    *(DXL_data + i) = 0;
}

uint8_t dxlMemory::getValueFromDxlData(uint16_t address)
{
  return *(DXL_data + address);
}
uint32_t dxlMemory::getValueFromDxlData(uint16_t address, uint8_t len)
{
  uint32_t tmp = 0;
  if (address + len > DXL_DATA_SIZE)
    return 0;
  for (uint8_t i = 0; i < len; i++)
  {
    tmp = tmp | (*(DXL_data + address + i) << (8 * i));
  }

  return tmp;
}
void dxlMemory::store(uint16_t address, uint8_t value)
{
  if (address + 1 < DXL_DATA_SIZE)
  {
    *(DXL_data + address) = value & 0xff;

    // IF ROM data, save into EEPROM
#ifdef EEPROM_ENABLED
    if (address < EEPROM_ADDRESSES)
    {
      storeToEEPROM = true;
    }
#endif
  }
}
void dxlMemory::store(uint16_t address, int value)
{
  store(address, (uint8_t)value);
}
void dxlMemory::store(uint16_t address, uint16_t value)
{
  if (address + 2 < DXL_DATA_SIZE)
  {
    for (uint8_t i = 0; i < 2; i++)
    {
      store(address + i, (uint8_t)((value >> (8 * i)) & 0xff));
    }
  }
}
void dxlMemory::store(uint16_t address, uint32_t value)
{
  if (address + 4 < DXL_DATA_SIZE)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      store(address + i, (uint8_t)((value >> (8 * i)) & 0xff));
    }
  }
}
// memory write with data segment
void dxlMemory::store(uint16_t address, uint16_t wSize, uint8_t *value)
{
  if (address + (wSize) < DXL_DATA_SIZE)
  {
    for (uint8_t i = 0; i < (wSize); i++)
    {
      store(address + i, (uint8_t) * (value + i));
    }
  }
}
// READ Data from data_dyn
void dxlMemory::memRead(uint16_t address, uint16_t *readSize, uint8_t *outData, uint16_t *outDataSize)
{
  // Resize max lenght
  uint16_t sizetoread = *readSize;
  if (address + *readSize >= DXL_DATA_SIZE)
  {
    sizetoread = DXL_DATA_SIZE - address;
  }
  for (uint16_t i = 0; i < sizetoread; i++)
  {
    *(outData + *outDataSize) = *(DXL_data + i + address);
    *(outDataSize) += 1;
  }
}
void dxlMemory::memRead(uint16_t address, uint16_t readSize, uint8_t *outData, uint16_t *outDataSize)
{
  // Resize max lenght
  uint16_t sizetoread = readSize;
  if (address + readSize >= DXL_DATA_SIZE)
  {
    sizetoread = DXL_DATA_SIZE - address;
  }
  for (uint16_t i = 0; i < sizetoread; i++)
  {
    *(outData + *outDataSize) = *(DXL_data + i + address);
    *(outDataSize) += 1;
  }
}

uint8_t dxlMemory::writeEEPROM(uint16_t address, uint8_t value)
{
  if (address > EEPROM_LENGTH)
    return 0x07;
  // EEPROM.write(address + EEPROM_FIRST_ADDRESS, value);
  EEPROM[address + EEPROM_FIRST_ADDRESS] = value;
  /*eeprom[_buffered_write_byte(address + EEPROM_FIRST_ADDRESS, value);
    eeprom_buffer_flush();*/
  // Compute CRC
  uint16_t crc = DataDynCRC();
  // WRITE CRC
  EEPROM[EEPROM_CRCL] = crc & 0xFF;
  EEPROM[EEPROM_CRCH] = crc >> 8;

  return 0;
}

uint16_t dxlMemory::DataDynCRC()
{

  unsigned short crc = crc_conversion(0, DXL_data, EEPROM_ADDRESSES - 1);

  return crc;
}
