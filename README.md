# SimpleFoc DXL Core

### COMMANDS

### ERRORS

#### Protocol Errors

- [ ] 0x01 Result Fail Failed to process the sent Instruction Packet
- [x] 0x02 Instruction Error Undefined Instruction has been used (Action has been used without Reg Write)
- [x] 0x03 CRC Error CRC of the sent Packet does not match
- [ ] 0x04 Data Range Error Data to be written in the corresponding Address is outside the range of the minimum/maximum value
- [ ] 0x05 Data Length Error Attempt to write Data that is shorter than the data length of the corresponding Address (ex: when you attempt to only use 2 bytes of a item that has been defined as 4 bytes)
- [ ] 0x06 Data Limit Error Data to be written in the corresponding Address is outside of the Limit value
- [ ] 0x07 Access Error Attempt to write a value in an Address that is Read Only or has not been defined (Attempt to read a value in an Address that is Write Only or has not been defined) Attempt to write a value in the ROM domain while in a state of Torque Enable(ROM Lock)

#### Hardware errors

- [x] Bit 7 - Unused, Always ‘0’
- [x] Bit 6 - Unused, Always ‘0’
- [ ] Bit 5 Overload Error(default) Detects that persistent load that exceeds maximum output
- [ ] Bit 4 Electrical Shock Error(default) Detects electric shock on the circuit or insufficient power to operate the motor
- [ ] Bit 3 Motor Encoder Error Detects malfunction of the motor encoder
- [x] Bit 2 Overheating Error(default) Detects that internal temperature exceeds the configured operating temperature
- [x] Bit 1 - Unused, Always ‘0’
- [x] Bit 0 Input Voltage Error Detects that input voltage exceeds the configured operating voltage

### MEMORY

- [x] #define ADD_MODEL_NUMBER 0
    
- [x] #define ADD_MODEL_INFORMATION 2
    
- [x] #define ADD_VERSION_OF_FIRMWARE 6
    
- [x] #define ADD_ID 7
    
- [ ] #define ADD_BAUDRATE 8
    
- [ ] #define ADD_RETURN_DELAY_TIME 9
    
- [ ] #define ADD_DRIVE_MODE 10
    
- [ ] #define ADD_OPERATING_MODE 11
    
- [ ] #define ADD_SECONDARY_ID 12
    
- [ ] #define ADD_PROTOCOL_VERSION 13
    
- [ ] #define ADD_HOMING_OFFSET 20
    
- [ ] #define ADD_MOVING_THRESHOLD 24
    
- [ ] #define ADD_TEMPERATURE_LIMIT 31
    
- [ ] #define ADD_MAX_VOLTAGE_LIMIT 32
    
- [ ] #define ADD_MIN_VOLTAGE_LIMIT 34
    
- [ ] #define ADD_PWM_LIMIT 36
    
- [ ] #define ADD_CURRENT_LIMIT 38
    
- [ ] #define ADD_ACCELERATION_LIMIT 40
    
- [x] #define ADD_VELOCITY_LIMIT 44
    
- [ ] #define ADD_MAX_POSITION_LIMIT 48
    
- [ ] #define ADD_MIN_POSITION_LIMIT 52
    
- [ ] #define ADD_SHUTDOWN 63
    
- [ ] ## // RAM
    
- [x] #define ADD_TORQUE_ENABLE 64
    
- [x] #define ADD_LED 65 (PEUT ETRE AMELIORER
    
- [ ] #define ADD_STATUS_RETURN_LEVEL 68
    
- [ ] #define ADD_REGISTERED_INSTRUCTION 69
    
- [ ] #define ADD_HARDWARE_ERROR_STATUS 70
    
- [ ] #define ADD_VELOCITY_I_GAIN 76
    
- [ ] #define ADD_VELOCITY_P_GAIN 78
    
- [ ] #define ADD_POSITION_D_GAIN 80
    
- [ ] #define ADD_POSITION_I_GAIN 82
    
- [ ] #define ADD_POSITION_P_GAIN 84
    
- [ ] #define ADD_FEEDFORWARD_ACCELERATION_GAIN 88
    
- [ ] #define ADD_FEEDFORWARD_VELOCITY_GAIN 90
    
- [ ] #define ADD_BUS_WATCHDOG 98
    
- [ ] #define ADD_GOAL_PWM 100
    
- [ ] #define ADD_GOAL_CURRENT 102
    
- [ ] #define ADD_GOAL_VELOCITY 104
    
- [ ] #define ADD_PROFILE_ACCELERATION 108
    
- [ ] #define ADD_PROFILE_VELOCITY 112
    
- [ ] #define ADD_GOAL_POSITION 116
    
- [ ] #define ADD_REALTIME_TICK 120
    
- [ ] #define ADD_MOVING 122
    
- [ ] #define ADD_MOVING_STATUS 123
    
- [ ] #define ADD_PRESENT_PWM 124
    
- [ ] #define ADD_PRESENT_CURRENT 126
    
- [ ] #define ADD_PRESENT_VELOCITY 128
    
- [ ] #define ADD_PRESENT_POSITION 132
    
- [ ] #define ADD_VELOCITY_TRAJECTORY 136
    
- [ ] #define ADD_POSITION_TRAJECTORY 140
    
- [ ] #define ADD_PRESENT_INPUT_VOLTAGE 144
    
- [ ] #define ADD_PRESENT_TEMPERATURE 146