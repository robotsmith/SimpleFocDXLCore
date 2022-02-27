#include <Arduino.h>
#include <Wire.h>
#include <BLDCMotor.h>
#include <drivers/BLDCDriver3PWM.h>
#include <sensors/MagneticSensorI2C.h>
#include "simplefocDxlCore.h"


// SERIAL
#define SERIAL_BAUDRATE 1000000
// I2C
#define I2C_SPEED 400000


#define DRV_IN1 PA6
#define DRV_IN2 PA7
#define DRV_IN3 PB0
#define DRV_EN PB1
#define DRV_NSLP PB2
#define DRV_FLT PB15
#define DRV_NRST PB13
#define STS_LED PB14
#define INVOLTAGE_PIN PA0
#define TEMPERATURE_PIN PA1

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
simplefocDxlCore mydxl(&motor);

// I2C
//            SDA  SCL
TwoWire Wire2(PB11, PB10);
// SETUP
void setup()
{


  // Attach Hardware
  mydxl.attachHarware(DRV_NRST, DRV_NSLP, DRV_FLT, STS_LED, TEMPERATURE_PIN, INVOLTAGE_PIN);


  //  initialise magnetic sensor hardware
  sensor.init(&Wire2);
  Wire2.setClock(I2C_SPEED);
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
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 0.0;
  motor.PID_velocity.limit = 300.0;

  // Low pass filtering time constant
  motor.LPF_velocity.Tf = 0.02;
  // angle loop PID
  motor.P_angle.P = 50.0;
  motor.P_angle.I = 500.0;
  motor.P_angle.D = 0.001;
  motor.P_angle.output_ramp = 0.0;
  motor.P_angle.limit = 50.0;
  // Low pass filtering time constant
  motor.LPF_angle.Tf = 0.02;

  motor.P_angle.limit = 100.0;

  // motor phase resistance
  // motor.phase_resistance = 4.823;
  // pwm modulation settings
  motor.foc_modulation = FOCModulationType::SinePWM;
  // motor.modulation_centered = 1.0;

  // Limits
  motor.velocity_limit = 1000.0 / 9.5493; // Rad/s -> RPM : 1000RPM
  motor.voltage_limit = 4.0;
  motor.current_limit = 0.0;

  // use monitoring with serial for motor init
  // monitoring port
#ifdef HALF_DUPLEX_MODE
  Serial1.setHalfDuplex();
#endif
  Serial1.begin(SERIAL_BAUDRATE);
  mydxl.attachSerial(Serial1);

  // INIT DXL DEVICE
  mydxl.init();
  _delay(200);
}

void loop()
{


  motor.loopFOC();
  motor.move();

  // Update DXL Device
  mydxl.update();

}
