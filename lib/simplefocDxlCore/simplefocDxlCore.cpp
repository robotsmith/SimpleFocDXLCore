#include "simplefocDxlCore.h"
#define LIGHT_VERSION
simplefocDxlCore::simplefocDxlCore(BLDCMotor *_motor)
{
    // Associate simplefoc motor
    motor = _motor;
    // Init memory, if memory is not correct make a factory reset
    if (dxlmem.load())
    {
        factoryResetMem();
    }
}
void simplefocDxlCore::factoryResetMem()
{
    // Load default
    loadDefaultMem();
    // Store memory
}
void simplefocDxlCore::init()
{
    // Refresh parameter from simplefoc
    refreshMotorData();
    update_parameters();
}
void simplefocDxlCore::loadDefaultMem()
{

    // Clear memory data
    dxlmem.clear();

    // MODEL NUMBER
    dxlmem.store(ADD_MODEL_NUMBER, (uint16_t)0x0406); // XM430
    // MODEL FIRMWARE
    dxlmem.store(ADD_VERSION_OF_FIRMWARE, (uint8_t)0x26);
    // ID
    dxlmem.store(ADD_ID, (uint8_t)0x01);
    // BAUDRATE
    dxlmem.store(ADD_BAUDRATE, (uint8_t)0x02); // 115200
    // Drive mode
    dxlmem.store(ADD_DRIVE_MODE, (uint8_t)0x08); // TORQUE ON GOAL UPDATE
    // control mode
    dxlmem.store(ADD_OPERATING_MODE, (uint8_t)0x03); // POSITION CONTROL
    // protocol mode
    dxlmem.store(ADD_PROTOCOL_VERSION, (uint8_t)0x02); // PROTOCOL 2

    // MAX temperature
    dxlmem.store(ADD_TEMPERATURE_LIMIT, (uint8_t)0x50);
    // Max voltage
    dxlmem.store(ADD_MAX_VOLTAGE_LIMIT, (uint16_t)0x0082); // 13V
    // Min voltage
    dxlmem.store(ADD_MIN_VOLTAGE_LIMIT, (uint16_t)0x0050); // 8V

    // PWM limit

    // CURRENT LIMIT

    // ACCELERATION LIMIT

    // VELOCITY LIMIT
    dxlmem.store(ADD_VELOCITY_LIMIT, (uint32_t)(1000 / 0.02398));
    // POSITION LIMIT
    dxlmem.store(ADD_MAX_POSITION_LIMIT, (uint16_t)0xFFFFF);
    dxlmem.store(ADD_MIN_POSITION_LIMIT, (uint16_t)0x0);
    // TORQUE
    dxlmem.store(ADD_TORQUE_ENABLE, (uint8_t)0x00); // 58.4 rev/min
}

void simplefocDxlCore::attachSerial(HardwareSerial &serial)
{
    dxlcom.attach(serial);
}
void simplefocDxlCore::update()
{
    // Check incoming serial
    dxlcom.checkSerial();

    // Check if a incoming packet is available
    if (dxlcom.packetAvailable())
    {

        // Is this packet for this device?
        if (dxlcom.inPacket.getId() == dxlmem.getValueFromDxlData(ADD_ID) || dxlcom.inPacket.getId() == 0xFE)
        {
            // Execute the packet
            executePacketCommand();
            // Update parameter if one is available
            if (pending_parameter)
            {
                update_parameters();
                pending_parameter = false;
            }
        }
        // Clear packet
        dxlcom.inPacket.clear();
    }

    // Update data from motor
    refreshMotorData();
}
void simplefocDxlCore::executePacketCommand()
{

    // Clear outpacket
    dxlcom.outPacket.clear();

    // Execute instructions
    if (dxlcom.inPacket.instruction() == INST_PING)
    {
        // Add model number to the packet
        dxlmem.memRead(ADD_MODEL_NUMBER, 2, dxlcom.outPacket.buffer, &(dxlcom.outPacket.currentSize));
        // Add firmware version to the packet
        dxlmem.memRead(ADD_VERSION_OF_FIRMWARE, 1, dxlcom.outPacket.buffer, &(dxlcom.outPacket.currentSize));
    }
    else if (dxlcom.inPacket.instruction() == INST_READ)
    {
        // ADDRESS READ
        uint16_t address = *(dxlcom.inPacket.buffer + PARAM_GAP) + (*(dxlcom.inPacket.buffer + 1 + PARAM_GAP) << 8);
        uint16_t readsize = *(dxlcom.inPacket.buffer + 2 + PARAM_GAP) + (*(dxlcom.inPacket.buffer + 3 + PARAM_GAP) << 8);
        // FILL READ DATA
        if (readsize + address + dxlcom.outPacket.currentSize + 2 > PACKET_BUFFER_SIZE)
            readsize = PACKET_BUFFER_SIZE - address - dxlcom.outPacket.currentSize - 2;
        dxlmem.memRead(address, readsize, dxlcom.outPacket.buffer, &(dxlcom.outPacket.currentSize));
    }
    else if (dxlcom.inPacket.instruction() == INST_WRITE)
    {
        // A new parameter is available
        pending_parameter = true;

        // Writ address
        uint16_t wAddress = *(dxlcom.inPacket.buffer + PARAM_GAP) + (*(dxlcom.inPacket.buffer + 1 + PARAM_GAP) << 8);
        // Remove address 4 (CRC + size position) bytes from size and select the right parameters (+2 bytes)
        // |L=8:HEADERS,ID,... (PARAM_GAP)|L=2:ADD|L=2:size to read|L=2:CRC|
        uint16_t size = dxlcom.inPacket.currentSize - PARAM_GAP - 3;
        dxlmem.store(wAddress, (size), (dxlcom.inPacket.buffer + PARAM_GAP + 2));
    }
    else if (dxlcom.inPacket.instruction() == INST_REBOOT)
    {
        NVIC_SystemReset();
    }
    else
    {
        // $TODO
    }
    // Set ID before sending
    dxlcom.outPacket.setId(dxlmem.getValueFromDxlData(ADD_ID));
    // Finish packet
    dxlcom.closeStatusPacket();


    dxlcom.sendOutPacket();

}
void simplefocDxlCore::refreshMotorData()
{

    // ADD_VELOCITY_LIMIT
    // dxlmem.memWrite(ADD_VELOCITY_LIMIT, (uint32_t)(motor->velocity_limit / 0.02398));

    // ADD_VELOCITY_I_GAIN
    dxlmem.store(ADD_VELOCITY_I_GAIN, (uint16_t)(motor->PID_velocity.I * 128));
    // ADD_VELOCITY_P_GAIN
    dxlmem.store(ADD_VELOCITY_P_GAIN, (uint16_t)(motor->PID_velocity.P * 128));
    // ADD_POSITION_D_GAIN
    dxlmem.store(ADD_POSITION_D_GAIN, (uint16_t)(motor->P_angle.D * 128));
    // ADD_POSITION_I_GAIN
    dxlmem.store(ADD_POSITION_I_GAIN, (uint16_t)(motor->P_angle.I * 128));
    // ADD_POSITION_P_GAIN
    dxlmem.store(ADD_POSITION_P_GAIN, (uint16_t)(motor->P_angle.P * 16));
    // ADD_MOVING
    // ADD_MOVING_STATUS
    // ADD_PRESENT_PWM
    // ADD_PRESENT_CURRENT
    // dxlmem.store(ADD_PRESENT_CURRENT, (uint32_t)(abs(motor->current_sp*2.69)));
    // ADD_PRESENT_VELOCITY
    dxlmem.store(ADD_PRESENT_VELOCITY, (uint32_t)(abs(motor->shaft_velocity / 0.02398)));
    // ADD_PRESENT_POSITION
    dxlmem.store(ADD_PRESENT_POSITION, (uint32_t)(motor->shaft_angle / (2 * 3.1415 / 4095)));
    // ADD_VELOCITY_TRAJECTORY
    // ADD_POSITION_TRAJECTORY

#ifndef LIGHT_VERSION
    // ADD_PRESENT_INPUT_VOLTAGE
    // R2 = 2200  / R1 = 10000 / MCU voltage 3.3V / DXL multiplier 10
    uint16_t involtage = (double)(analogRead(_in_voltage) * 0.18);
    dxlmem.store(ADD_PRESENT_INPUT_VOLTAGE, (uint16_t)(involtage));

    // ADD_PRESENT_TEMPERATURE MCP9700T-E/TT
    float voltage = float(analogRead(_temp_pin)) * 3300.0 / 1024.0;
    float temperature = (voltage - 500.0) / 10;
    // uint8_t temperature = (double)(analogRead(_temp_pin) * 0.0806);
    dxlmem.store(ADD_PRESENT_TEMPERATURE, (uint8_t)temperature);
#endif
}

void simplefocDxlCore::update_parameters()
{

    // ADD_MAX_VOLTAGE_LIMIT
    // ADD_MIN_VOLTAGE_LIMIT
    // ADD_PWM_LIMIT
    // ADD_CURRENT_LIMIT
    // ADD_ACCELERATION_LIMIT
    // ADD_VELOCITY_LIMIT
    volatile uint32_t stored_vel = dxlmem.getValueFromDxlData(ADD_VELOCITY_LIMIT, 4);
    double velocity = stored_vel * 0.02398;
    motor->velocity_limit = (double)velocity;
    // ADD_MAX_POSITION_LIMIT
    // ADD_MIN_POSITION_LIMIT
    // ADD_SHUTDOWN

    //***RAM
    // ADD_TORQUE_ENABLE
    if (dxlmem.getValueFromDxlData(ADD_TORQUE_ENABLE, 1))
    {
        motor->enable();
    }
    else
        motor->disable();

    // ADD_LED
    if (dxlmem.getValueFromDxlData(ADD_LED, 1) == 1)
    {
        digitalWrite(_led_pin, HIGH);
    }
    else
        digitalWrite(_led_pin, LOW);
    // ADD_HARDWARE_ERROR_STATUS
    // ADD_VELOCITY_I_GAIN
    motor->PID_velocity.I = (double)dxlmem.getValueFromDxlData(ADD_VELOCITY_I_GAIN, 2) / 128;
    // ADD_VELOCITY_P_GAIN
    motor->PID_velocity.P = (double)dxlmem.getValueFromDxlData(ADD_VELOCITY_P_GAIN, 2) / 128;
    // ADD_POSITION_D_GAIN
    motor->P_angle.D = (double)dxlmem.getValueFromDxlData(ADD_POSITION_D_GAIN, 2) / 128;
    // ADD_POSITION_I_GAIN
    motor->P_angle.I = (double)dxlmem.getValueFromDxlData(ADD_POSITION_I_GAIN, 2) / 128;
    // ADD_POSITION_P_GAIN
    motor->P_angle.P = (double)dxlmem.getValueFromDxlData(ADD_POSITION_P_GAIN, 2) / 16;
    // ADD_FEEDFORWARD_ACCELERATION_GAIN
    // ADD_FEEDFORWARD_VELOCITY_GAIN
    // ADD_BUS_WATCHDOG
    // ADD_GOAL_PWM
    // ADD_GOAL_CURRENT
    // ADD_GOAL_VELOCITY
    // ADD_PROFILE_ACCELERATION
    // ADD_PROFILE_VELOCITY
    // ADD_GOAL_POSITION
    motor->target = (double)((uint32_t)dxlmem.getValueFromDxlData(ADD_GOAL_POSITION, 4)) * 2 * 3.1415 / 4095;

    // ADD_REALTIME_TICK

    //*** FEEDBACK
    // ADD_MOVING
    // ADD_MOVING_STATUS
    // ADD_PRESENT_PWM
    // ADD_PRESENT_CURRENT
    // ADD_PRESENT_VELOCITY

    // ADD_PRESENT_POSITION
    // ADD_VELOCITY_TRAJECTORY
    // ADD_POSITION_TRAJECTORY
    // ADD_PRESENT_INPUT_VOLTAGE

    // ADD_PRESENT_TEMPERATURE
}
