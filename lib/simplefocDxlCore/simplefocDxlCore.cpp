#include "simplefocDxlCore.h"

simplefocDxlCore::simplefocDxlCore()
{
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
void simplefocDxlCore::update()
{
    // Simple foc

    // Dyn bridge
    dxl->update();
    if (dxl->parameter_available())
        update_parameters();

    update_parameters_from_motor();

    // Check serial
    dxl->SERevent();
}
void simplefoc_dxl_servo::update_parameters_from_motor()
{

    // ADD_VELOCITY_LIMIT
    // dxl->memWrite(ADD_VELOCITY_LIMIT, (uint32_t)(motor->velocity_limit / 0.02398));

    // ADD_VELOCITY_I_GAIN
    dxl->memWrite(ADD_VELOCITY_I_GAIN, (uint16_t)(motor->PID_velocity.I * 128));
    // ADD_VELOCITY_P_GAIN
    dxl->memWrite(ADD_VELOCITY_P_GAIN, (uint16_t)(motor->PID_velocity.P * 128));
    // ADD_POSITION_D_GAIN
    dxl->memWrite(ADD_POSITION_D_GAIN, (uint16_t)(motor->P_angle.D * 128));
    // ADD_POSITION_I_GAIN
    dxl->memWrite(ADD_POSITION_I_GAIN, (uint16_t)(motor->P_angle.I * 128));
    // ADD_POSITION_P_GAIN
    dxl->memWrite(ADD_POSITION_P_GAIN, (uint16_t)(motor->P_angle.P * 16));
    // ADD_MOVING
    // ADD_MOVING_STATUS
    // ADD_PRESENT_PWM
    // ADD_PRESENT_CURRENT
    // ADD_PRESENT_VELOCITY

    // ADD_PRESENT_POSITION
    dxl->memWrite(ADD_PRESENT_POSITION, (uint16_t)(motor->shaft_angle / (2 * 3.1415 / 4095)));
    // ADD_VELOCITY_TRAJECTORY
    // ADD_POSITION_TRAJECTORY
    // ADD_PRESENT_INPUT_VOLTAGE
    // ADD_PRESENT_TEMPERATURE
}

void simplefocDxlCore::update_parameters()
{

    // ADD_MAX_VOLTAGE_LIMIT
    // ADD_MIN_VOLTAGE_LIMIT
    // ADD_PWM_LIMIT
    // ADD_CURRENT_LIMIT
    // ADD_ACCELERATION_LIMIT
    // ADD_VELOCITY_LIMIT
    motor->velocity_limit = (double)dxl->getMemValue(ADD_VELOCITY_LIMIT, 4) * 0.02398;
    // ADD_MAX_POSITION_LIMIT
    // ADD_MIN_POSITION_LIMIT
    // ADD_SHUTDOWN

    //***RAM
    // ADD_TORQUE_ENABLE
    motor->enabled = dxl->getMemValue(ADD_TORQUE_ENABLE, 1);
    // ADD_LED

    // ADD_HARDWARE_ERROR_STATUS
    // ADD_VELOCITY_I_GAIN
    motor->PID_velocity.I = (double)dxl->getMemValue(ADD_VELOCITY_I_GAIN, 2) / 128;
    // ADD_VELOCITY_P_GAIN
    motor->PID_velocity.P = (double)dxl->getMemValue(ADD_VELOCITY_P_GAIN, 2) / 128;
    // ADD_POSITION_D_GAIN
    motor->P_angle.D = (double)dxl->getMemValue(ADD_POSITION_D_GAIN, 2) / 128;
    // ADD_POSITION_I_GAIN
    motor->P_angle.I = (double)dxl->getMemValue(ADD_POSITION_I_GAIN, 2) / 128;
    // ADD_POSITION_P_GAIN
    motor->P_angle.P = (double)dxl->getMemValue(ADD_POSITION_P_GAIN, 2) / 16;
    // ADD_FEEDFORWARD_ACCELERATION_GAIN
    // ADD_FEEDFORWARD_VELOCITY_GAIN
    // ADD_BUS_WATCHDOG
    // ADD_GOAL_PWM
    // ADD_GOAL_CURRENT
    // ADD_GOAL_VELOCITY
    // ADD_PROFILE_ACCELERATION
    // ADD_PROFILE_VELOCITY
    // ADD_GOAL_POSITION
    motor->target = (double)((uint32_t)dxl->getMemValue(ADD_GOAL_POSITION, 4)) * 2 * 3.1415 / 4095;

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
