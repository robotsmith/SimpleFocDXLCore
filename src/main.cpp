#include <Arduino.h>

#include <Wire.h>
#include <BLDCMotor.h>
#include <drivers/BLDCDriver3PWM.h>
#include <sensors/MagneticSensorI2C.h>
#include "dynamixel_protocol_V2.h"
#include "simplefoc2dxl.h"
#if __has_include("hal_conf_extra.h")
#include "hal_conf_extra.h"
#endif

#define SERIAL_BAUDRATE 1000000

#define DRV_IN1 PA6
#define DRV_IN2 PA7
#define DRV_IN3 PB0
#define DRV_EN PB1
#define DRV_NSLP PB2
#define DRV_FLT PB14
#define DRV_NRST PB13
#define STS_LED PB14

MagneticSensorI2CConfig_s MySensorConfig = {
    .chip_address = 0x40,
    .bit_resolution = 12,
    .angle_register = 0x0C,
    .data_start_bit = 11};
MagneticSensorI2C sensor = MagneticSensorI2C(MySensorConfig);
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(DRV_IN1, DRV_IN2, DRV_IN3, DRV_EN);
// COM
#ifdef HALF_DUPLEX_MODE
HardwareSerial Serial1(PA9);
#else
HardwareSerial Serial1(PA10, PA9);
#endif
// DYNAMIXEL DEVICE
dynamixelDevice dyn(Serial1);
// WRAPPER
simplefoc_dxl_servo dxlservo(&motor, &dyn);

// SETUP
void setup()
{

  pinMode(DRV_FLT, INPUT);
  pinMode(DRV_NSLP, OUTPUT);
  pinMode(DRV_NRST, OUTPUT);
  pinMode(STS_LED, OUTPUT);

  digitalWrite(DRV_NRST, HIGH);
  digitalWrite(DRV_NSLP, HIGH);
  digitalWrite(STS_LED, LOW);

  Wire.setSDA(PB11);
  Wire.setSCL(PB10);
  // Wire.setSCL(PB6);
  // Wire.setSDA(PB7);
  // initialise magnetic sensor hardware
  sensor.init(&Wire);
  Wire.setClock(400000);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // control loop type and torque mode
  // motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::angle;
  // motor.motion_downsample = 0.0;

  // velocity loop PID
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 0.0;
  motor.PID_velocity.limit = 3.0;

  // Low pass filtering time constant
  motor.LPF_velocity.Tf = 0.02;
  // angle loop PID
  motor.P_angle.P = 50.0;
  motor.P_angle.I = 1000.0;
  motor.P_angle.D = 0.001;
  motor.P_angle.output_ramp = 0.0;
  motor.P_angle.limit = 50.0;
  // Low pass filtering time constant
  motor.LPF_angle.Tf = 0.01;

  motor.P_angle.limit = 100.0;
  // Limits
  motor.velocity_limit = 50.0;
  motor.voltage_limit = 3.0;
  motor.current_limit = 0.0;

  // motor phase resistance
  // motor.phase_resistance = 4.823;
  // pwm modulation settings
  motor.foc_modulation = FOCModulationType::SinePWM;
  // motor.modulation_centered = 1.0;

  // Limits
  motor.velocity_limit = 20.0;
  motor.voltage_limit = 3.0;
  motor.current_limit = 0;

  // use monitoring with serial for motor init
  // monitoring port
#ifdef HALF_DUPLEX_MODE
  Serial1.setHalfDuplex();
#endif
  Serial1.begin(SERIAL_BAUDRATE);
  Serial1.print("CPU:");
  Serial1.println(F_CPU);
  // comment out if not needed
  // motor.useMonitoring(Serial1);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  motor.enabled = false;
  dxlservo.update_parameters_from_motor();
  // set the inital target value
  // motor.target = 2;

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  // Serial1.println(F("Motor ready"));

  _delay(200);
}

void loop()
{
  // long tmp = micros();
  motor.loopFOC();
  motor.move();
  // tmp = micros() - tmp;
  // Serial1.println(tmp);
  dxlservo.update();
}
