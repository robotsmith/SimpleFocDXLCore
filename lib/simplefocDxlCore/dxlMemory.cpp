#include "dxlMemory.h"

dxlMemory::dxlMemory() { clear(); }

bool dxlMemory::load()
{
  if (CheckEEPROM_CRC())
  {
    return true; // There is an CRC error
  }
  // Clear RAM
  clear();
  // Load EEPROM
  if (loadEEPROM())
    return true; // there is an issue in loading

  return false;
}

bool dxlMemory::CheckEEPROM_CRC()
{
  eeprom_buffer_fill();
  // GET CRC
  unsigned short crc = crc_conversion(0, dyn_data, EEPROM_ADDRESSES - 1); // CRC WITHOUT CRC VALUE IN EEPROM (beginning after EEPROM_FIRST_ADDRESS )
  char crcl = eeprom_buffered_read_byte(EEPROM_CRCL);
  char crch = eeprom_buffered_read_byte(EEPROM_CRCH);
  unsigned short crc_in_eeprom = crcl + (crch << 8);
  // EEPROM GOT THE RIGHT CRC ??
  if (crc == crc_in_eeprom)
    return false;

  // ELSE
  return true;
}

// EEPROM MANAGEMENT
bool dxlMemory::loadEEPROM()
{
  eeprom_buffer_fill();
  for (uint16_t i = 0; i < DXL_DATA_SIZE; i++)
  {
    if (i + EEPROM_FIRST_ADDRESS > EEPROM_LENGTH)
      return true;
    //*(data + i) = EEPROM.read(i + EEPROM_FIRST_ADDRESS);
    *(dyn_data + i) = eeprom_buffered_read_byte(i + EEPROM_FIRST_ADDRESS);
  }
  return false;
}

void dxlMemory::data2EEPROM()
{
  for (uint16_t i = 0; i < DXL_DATA_SIZE; i++)
  {
    // EEPROM.update(i + EEPROM_FIRST_ADDRESS, *(data + i));
    eeprom_buffered_write_byte(i + EEPROM_FIRST_ADDRESS, *(dyn_data + i));
  }
  // FLAG INITIALIZED
  // EEPROM.update(0, 0x01);
  eeprom_buffered_write_byte(0, 0x01);
  eeprom_buffer_flush();
  // Compute CRC
  storeDataDynCRC();
}

void dxlMemory::clear()
{
  for (uint16_t i = 0; i < DXL_DATA_SIZE; i++)
    *(dyn_data + i) = 0;
}

uint8_t dxlMemory::getValueFromDxlData(uint16_t address)
{
  return *(dyn_data + address);
}
uint32_t dxlMemory::getValueFromDxlData(uint16_t address, uint8_t len)
{
  uint32_t tmp = 0;
  if (address + len > DXL_DATA_SIZE)
    return 0;
  for (uint8_t i = 0; i < len; i++)
  {
    tmp = tmp | (*(dyn_data + address + i) << (8 * i));
  }

  return tmp;
}
void dxlMemory::store(uint16_t address, uint8_t value)
{
  if (address + 1 < DXL_DATA_SIZE)
  {
    *(dyn_data + address) = value & 0xff;

    // IF ROM data, save into EEPROM
#ifdef EEPROM_ENABLED
    if (address < EEPROM_ADDRESSES)
    {
      writeEEPROM(address, value);
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
void dxlMemory::memRead(uint16_t address, uint16_t *readSize, uint8_t *outData, uint16_t *outDataSize, uint16_t maxSize)
{
  // Resize max lenght
  uint16_t sizetoread = *readSize;
  if (address + *readSize >= DXL_DATA_SIZE)
  {
    sizetoread = DXL_DATA_SIZE - address;
  }
  else if (maxSize > sizetoread + *outDataSize)
    sizetoread = maxSize - *outDataSize;
  for (uint16_t i = 0; i < sizetoread; i++)
  {
    *(outData + *outDataSize) = *(dyn_data + i + address);
    *(outDataSize) += 1;
  }
}
void dxlMemory::memRead(uint16_t address, uint16_t readSize, uint8_t *outData, uint16_t *outDataSize, uint16_t maxSize)
{
  // Resize max lenght
  uint16_t sizetoread = readSize;
  if (address + readSize >= DXL_DATA_SIZE)
  {
    sizetoread = DXL_DATA_SIZE - address;
  }
  else if (maxSize > sizetoread + *outDataSize)
    sizetoread = maxSize - *outDataSize;
  for (uint16_t i = 0; i < sizetoread; i++)
  {
    *(outData + *outDataSize) = *(dyn_data + i + address);
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
  storeDataDynCRC();
  return 0;
}

void dxlMemory::storeDataDynCRC()
{

  unsigned short crc = crc_conversion(0, dyn_data, EEPROM_ADDRESSES - 1);

  // WRITE CRC
  EEPROM[EEPROM_CRCL] = crc & 0xFF;
  EEPROM[EEPROM_CRCH] = crc >> 8;
}
