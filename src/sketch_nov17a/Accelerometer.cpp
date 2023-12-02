//#include "esp32-hal-gpio.h"
#include "Accelerometer.h"

#include <Arduino.h>
//#include <FunctionalInterrupt.h>
#include <SPI.h>
#include <Wire.h>

#include <math.h>

namespace Melty
{
namespace Navigation
{
namespace Accelerometer
{

Accelerometer::Accelerometer(uint8_t chip_select_pin) : 
    m_protocol(Protocol::SPI),
    m_data_ready(false),
    m_max_accel(100),
    m_chip_select_pin(chip_select_pin) {

  m_cal_adjustments = {};

  if (m_protocol == Protocol::SPI) {
    // Configure SPI bus with standard pins
    //SPI.setFrequency(Config::SPI_CLOCK_RATE);
    //SPI.setDataMode(SPI_MODE3);
    SPI.begin();

    pinMode(m_chip_select_pin, OUTPUT);
    digitalWrite(m_chip_select_pin, HIGH);
  } else { // I2C protocol
    Wire.begin();
  }
  

  // Set up the interrupt for data ready from the accelerometer
  //pinMode(Config::INTERRUPT_PIN, INPUT_PULLUP);
  //attachInterrupt(Config::INTERRUPT_PIN, std::bind(&Accelerometer::serviceInterrupt, this), FALLING);
}


float Accelerometer::convertToG(int16_t raw_value) {
  // Percentage of max value (signed 16 bit) times max reading
  return (raw_value / static_cast<float>(0x7FFF)) * static_cast<float>(m_max_accel);
}

void Accelerometer::serviceInterrupt() {
  m_data_ready.store(true);
}

bool Accelerometer::isDataReady() {
  return m_data_ready.load();
}

void Accelerometer::setFullScale(const Config::FullScale full_scale) {
  // NONE is a noop.  Don't change anything
  if (full_scale == Config::FullScale::NONE) {
    return;
  }

  Config::CtrlReg4 ctrl_reg_4{};
  ctrl_reg_4.value = readSPIByte(Config::CTRL_REG_4_ADDR);
  ctrl_reg_4.fields.fs = static_cast<uint8_t>(full_scale);
  writeSPIByte(Config::CTRL_REG_4_ADDR, ctrl_reg_4.value);

  switch(full_scale) {
    case Config::FullScale::G100:
      m_max_accel = 100;
      break;
    case Config::FullScale::G200:
      m_max_accel = 200;
      break;
    case Config::FullScale::G400:
      m_max_accel = 400;
      break;
    default:
      break;
  }
}

void Accelerometer::setPowerMode(const Config::PowerModes power_mode) {
  Config::CtrlReg1 ctrl_reg_1{};
  ctrl_reg_1.value = readSPIByte(Config::CTRL_REG_1_ADDR);
  ctrl_reg_1.fields.power_mode_select = static_cast<uint8_t>(power_mode);
  writeSPIByte(Config::CTRL_REG_1_ADDR, ctrl_reg_1.value);
}

void Accelerometer::setOutputDataRate(const Config::DataRates data_rate) {
  Config::CtrlReg1 ctrl_reg_1{};
  ctrl_reg_1.value = readSPIByte(Config::CTRL_REG_1_ADDR);
  ctrl_reg_1.fields.data_rate = static_cast<uint8_t>(data_rate);
  writeSPIByte(Config::CTRL_REG_1_ADDR, ctrl_reg_1.value);
}

void Accelerometer::setAxisEnables(const bool x, const bool y, const bool z) {
  Config::CtrlReg1 ctrl_reg_1{};
  ctrl_reg_1.value = readSPIByte(Config::CTRL_REG_1_ADDR);
  ctrl_reg_1.fields.x_enable = x;
  ctrl_reg_1.fields.x_enable = y;
  ctrl_reg_1.fields.x_enable = z;
  writeSPIByte(Config::CTRL_REG_1_ADDR, ctrl_reg_1.value);
}

bool Accelerometer::configureSPI() {

  // Reg 1: Power, data rates, axis enables
  Config::CtrlReg1 ctrl_reg_1{};
  ctrl_reg_1.fields.data_rate = static_cast<uint8_t>(Config::DataRates::HZ_400);
  ctrl_reg_1.fields.power_mode_select = static_cast<uint8_t>(Config::PowerModes::NORMAL);
  ctrl_reg_1.fields.x_enable = true;
  ctrl_reg_1.fields.y_enable = true;
  ctrl_reg_1.fields.z_enable = true;

  writeSPIByte(Config::CTRL_REG_1_ADDR, ctrl_reg_1.value);

  // Reg 2: Filter setup
  // Not doing anything with this for now

  // Reg 3: Interrupt control
  // TODO: Not using yet.  Interrupt is set to falling edge (active low)

  // Reg 4: block update, endianness, full scale, spi mode
  Config::CtrlReg4 ctrl_reg_4{};
  ctrl_reg_4.fields.bdu = true; // Don't update MSB/LSB registers between reads
  // Leave endianness as default
  ctrl_reg_4.fields.fs = static_cast<uint8_t>(Config::FullScale::G100);
  // Leave spi wire type as default (4-wire)

  writeSPIByte(Config::CTRL_REG_4_ADDR, ctrl_reg_4.value);

  // Reg 5: Sleep to wake - Leave as default (disabled)

  return true;
}

bool Accelerometer::configureI2C() {
  Wire.beginTransmission(I2C_ADDRESS);
  // Config register 1:
  Wire.write(Config::CTRL_REG_1_ADDR);
  const uint8_t config_reg_1 = Config::POWER_MODE_NORMAL |
                               static_cast<uint8_t>(Config::DataRates::HZ_1000) |
                               Config::X_ENABLE | Config::Y_ENABLE | Config::Z_ENABLE;
  Wire.write(config_reg_1);
  uint8_t status = Wire.endTransmission();

  return status == 0;
}

bool Accelerometer::configure() {
  if (m_protocol == Protocol::SPI) {
    return configureSPI();
  } else {
    return configureI2C();
  }
}

AccelerometerReading Accelerometer::calibrate(const uint16_t num_samples) {
  // reset any previous adjustments
  m_cal_adjustments = {};

  // Get the raw value of 1g, rounded up
  const int16_t one_g = static_cast<int16_t>(((1.0 / m_max_accel) * 0x7FFF) + 0.5);

  const auto moving_average = [] (int16_t last_average, int16_t new_reading, int sample_num) -> uint16_t {
    const float previous_weight = (sample_num - 1) / static_cast<float>(sample_num);
    const float new_weight = 1.0 / sample_num;
    const auto new_average = (previous_weight * last_average) + (new_weight * new_reading);
    return static_cast<int16_t>(new_average);
  };

  AccelerometerReading current_averages{};
  m_cal_adjustments = {};
  for (int sample = 0; sample < num_samples; ++sample) {
    const AccelerometerReading current_reading = read();
    current_averages.accel_x = moving_average(current_averages.accel_x, current_reading.accel_x, sample);
    current_averages.accel_y = moving_average(current_averages.accel_y, current_reading.accel_y, sample);
    current_averages.accel_z = moving_average(current_averages.accel_z, current_reading.accel_z, sample);

    // Sleep a bit so we don't hammer the accelerometer too fast
    // TODO: Dynamic sleep amount based on output data rate?
    delay(10);
  }

  // Set calibration adjustments such that adding these values will result in
  // 0x, 0y, 1z
  m_cal_adjustments.accel_x = -current_averages.accel_x;
  m_cal_adjustments.accel_y = -current_averages.accel_y;
  m_cal_adjustments.accel_z = one_g - current_averages.accel_z;

  return m_cal_adjustments;
}

uint8_t Accelerometer::readSPIByte(uint8_t address) {
  uint8_t value;
  digitalWrite(m_chip_select_pin, LOW);
  const uint8_t read_cmd = READ_BIT | SPI_INCREMENT_ADDRESS_BIT | address;
  SPI.transfer(read_cmd);
  value = SPI.transfer(0);
  digitalWrite(m_chip_select_pin, HIGH);
  return value;
}

void Accelerometer::writeSPIByte(uint8_t address, uint8_t data) {
  digitalWrite(m_chip_select_pin, LOW);
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(m_chip_select_pin, HIGH);
}

void Accelerometer::readSPI(AccelerometerReading &reading) {
  // TODO: This is strange and inefficient. Multibyte reads were causing issues, but
  //       doing this one byte at a time seems to work. Look into this later, but
  //       for now it works well enough.  ~93 microseconds for all 3 axes.
  reading.accel_x = readSPIByte(Config::X_AXIS_REG_ADDR);
  reading.accel_x |= (readSPIByte(Config::X_AXIS_REG_ADDR + 1) << 8);

  reading.accel_y = readSPIByte(Config::Y_AXIS_REG_ADDR);
  reading.accel_y |= (readSPIByte(Config::Y_AXIS_REG_ADDR + 1) << 8);

  reading.accel_z = readSPIByte(Config::Z_AXIS_REG_ADDR);
  reading.accel_z |= (readSPIByte(Config::Z_AXIS_REG_ADDR + 1) << 8);

  reading.valid = true;
}

void Accelerometer::readI2C(AccelerometerReading &reading) {
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(Config::X_AXIS_REG_ADDR | I2C_MULTI_BYTE_READ);
  Wire.endTransmission();
  const uint8_t len = 6;
  Wire.requestFrom(I2C_ADDRESS, len);
  reading.accel_x = Wire.read() | Wire.read() << 8;
  reading.accel_y = Wire.read() | Wire.read() << 8;
  reading.accel_z = Wire.read() | Wire.read() << 8;
  reading.valid = Wire.endTransmission() == 0;
}

AccelerometerReading Accelerometer::read() {
  AccelerometerReading reading{};

  if (m_protocol == Protocol::SPI) {
    readSPI(reading);
  } else {
    readI2C(reading);
  }

  // Apply calibration corrections
  reading.accel_x += m_cal_adjustments.accel_x;
  reading.accel_y += m_cal_adjustments.accel_y;
  reading.accel_z += m_cal_adjustments.accel_z;

  m_data_ready.store(false);
  return reading;
}

bool Accelerometer::checkConnection() {
  digitalWrite(m_chip_select_pin, LOW);
  const auto read_cmd = READ_BIT | WHOAMI_REG_ADDR;
  SPI.transfer(read_cmd);
  const auto whoami_value = SPI.transfer(0);
  digitalWrite(m_chip_select_pin, HIGH);

  return whoami_value == WHOAMI_VALUE;
}

}; // namespace Accelerometer
}; // Navigation
}; // namespace Melty
