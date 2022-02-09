# SimpleFoc DXL Core

### COMMANDS

See [https://emanual.robotis.com/docs/en/dxl/protocol2/#instruction-packet](Dynamixel Protocol 2.0)

- [x] 0x01 Ping Instruction
- [x] 0x02 Read Instruction to read data from the Device
- [x] 0x03 Write Instruction to write data on the Device
- [ ] 0x04 Reg Write
- [ ] 0x05 Action 
- [x] 0x06 Factory Reset
- [x] 0x08 Reboot Instruction to reboot the Device
- [ ] 0x10 Clear Instruction to reset certain information
- [ ] 0x20 Control Table Backup 
- [ ] 0x55 Status
- [ ] 0x82 Sync Read
- [ ] 0x83 Sync Write 
- [ ] 0x8A Fast Sync Read 
- [ ] 0x92 Bulk Read 
- [ ] 0x93 Bulk Write 
- [ ] 0x9A Fast Bulk Read 

### ERRORS

#### Protocol Errors

- [ ] 0x01 Result Fail Failed to process the sent Instruction Packet
- [x] 0x02 Instruction Error Undefined Instruction has been used 
- [x] 0x03 CRC Error CRC of the sent Packet does not match
- [ ] 0x04 Data Range Error Data
- [ ] 0x05 Data Length Error Attempt to write Data that is shorter than the data length
- [ ] 0x06 Data Limit Error Data
- [ ] 0x07 Access Error Attempt 

#### Hardware errors

- [x] Bit 7 - Unused, Always ‘0’
- [x] Bit 6 - Unused, Always ‘0’
- [ ] Bit 5 Overload Error
- [ ] Bit 4 Electrical Shock Error
- [ ] Bit 3 Motor Encoder Error Detects malfunction of the motor encoder
- [x] Bit 2 Overheating Error
- [x] Bit 1 - Unused, Always ‘0’
- [x] Bit 0 Input Voltage Error

### MEMORY XM430

See [https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/#hardware-error-status](XM430 Specific instructions)

#### EEPROM

Name / Address

- [x] MODEL_NUMBER 0
- [x] MODEL_INFORMATION 2
- [x] VERSION_OF_FIRMWARE 6
- [x] ID 7
- [ ] BAUDRATE 8
- [ ] RETURN_DELAY_TIME 9
- [ ] DRIVE_MODE 10
- [ ] OPERATING_MODE 11
- [ ] SECONDARY_ID 12
- [ ] PROTOCOL_VERSION 13
- [ ] HOMING_OFFSET 20
- [ ] MOVING_THRESHOLD 24
- [ ] TEMPERATURE_LIMIT 31
- [ ] MAX_VOLTAGE_LIMIT 32
- [ ] MIN_VOLTAGE_LIMIT 34
- [ ] PWM_LIMIT 36
- [ ] CURRENT_LIMIT 38
- [ ] ACCELERATION_LIMIT 40
- [x] VELOCITY_LIMIT 44
- [ ] MAX_POSITION_LIMIT 48
- [ ] MIN_POSITION_LIMIT 52
- [ ] SHUTDOWN 63

#### RAM

- [x] TORQUE_ENABLE 64
- [x] #define ADD_LED 65 (PEUT ETRE AMELIORER
- [ ] STATUS_RETURN_LEVEL 68
- [ ] REGISTERED_INSTRUCTION 69
- [x] HARDWARE_ERROR_STATUS 70
- [x] VELOCITY_I_GAIN 76
- [x] VELOCITY_P_GAIN 78
- [x] POSITION_D_GAIN 80
- [x] POSITION_I_GAIN 82
- [x] POSITION_P_GAIN 84
- [ ] FEEDFORWARD_ACCELERATION_GAIN 88
- [ ] FEEDFORWARD_VELOCITY_GAIN 90
- [ ] BUS_WATCHDOG 98
- [ ] GOAL_PWM 100
- [ ] GOAL_CURRENT 102
- [ ] GOAL_VELOCITY 104
- [ ] PROFILE_ACCELERATION 108
- [ ] PROFILE_VELOCITY 112
- [x] GOAL_POSITION 116
- [ ] REALTIME_TICK 120
- [ ] MOVING 122
- [ ] MOVING_STATUS 123
- [ ] PRESENT_PWM 124
- [ ] PRESENT_CURRENT 126
- [x] PRESENT_VELOCITY 128
- [x] PRESENT_POSITION 132
- [ ] VELOCITY_TRAJECTORY 136
- [ ] POSITION_TRAJECTORY 140
- [x] PRESENT_INPUT_VOLTAGE 144
- [x] PRESENT_TEMPERATURE 146