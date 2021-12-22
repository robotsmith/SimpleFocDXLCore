#ifndef SERVO_SIMPLEFOC2DXL_H
#define SERVO_SIMPLEFOC2DXL_H

#include <Wire.h>
#include <BLDCMotor.h>
#include <drivers/BLDCDriver3PWM.h>
#include <sensors/MagneticSensorI2C.h>
#include <dynamixel_protocol_V2.h>
// configuration for AS5600 sensor

class simplefoc_dxl_servo
{
public:
  simplefoc_dxl_servo(BLDCMotor *_motor, dynamixelDevice *_dxl);
  void update();
  void update_parameters_from_motor();

private:
  void update_parameters();

  BLDCMotor *motor;
  dynamixelDevice *dxl;
};
#endif
