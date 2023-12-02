#pragma once

//#include <SPI.h>

#include <atomic>

namespace Melty
{
namespace Navigation
{
namespace Accelerometer
{
/**
 * Struct to hold values from the accelerometer.
 */
struct AccelerometerReading {
  bool valid;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
};

/**
 * Communication protocol type to use when talking to the accelerometer.
 */
enum class Protocol {
  SPI = 0,
  I2C = 1
};

static constexpr uint8_t I2C_ADDRESS = 0x19;
static constexpr uint8_t I2C_MULTI_BYTE_READ = 0x80;

static constexpr uint8_t READ_BIT = 0x80;
static constexpr uint8_t SPI_INCREMENT_ADDRESS_BIT = 0x40;

// The accelerometer's WHOAMI register will always return 32.
// Use this to verify we have a good connection.
static constexpr uint8_t WHOAMI_REG_ADDR = 0x0F;
static constexpr uint8_t WHOAMI_VALUE = 0x32;
namespace Config
{
// Control register addresses
static constexpr uint8_t CTRL_REG_1_ADDR = 0x20;
static constexpr uint8_t CTRL_REG_2_ADDR = 0x21;
static constexpr uint8_t CTRL_REG_3_ADDR = 0x22;
static constexpr uint8_t CTRL_REG_4_ADDR = 0x23;
static constexpr uint8_t CTRL_REG_5_ADDR = 0x24;

static constexpr uint8_t STATUS_REG_ADDR = 0x27;

// Axis values are two bytes, low byte then high byte, 
// starting at the following offsets
static constexpr uint8_t X_AXIS_REG_ADDR = 0x28;
static constexpr uint8_t Y_AXIS_REG_ADDR = 0x2A;
static constexpr uint8_t Z_AXIS_REG_ADDR = 0x2C;

static constexpr uint8_t INTERRUPT_PIN = 34;

static constexpr uint32_t SPI_CLOCK_RATE = 10000000; // 10MHz SPI clock

static constexpr uint8_t POWER_MODE_NORMAL = 0x20;
static constexpr uint8_t X_ENABLE = 0x01;
static constexpr uint8_t Y_ENABLE = 0x02;
static constexpr uint8_t Z_ENABLE = 0x04;

enum class DataRates {
  HZ_50 = 0x00,
  HZ_100 = 0x01,
  HZ_400 = 0x02,
  HZ_1000 = 0x04
};

/**
 * Power modes for the chip.  Normal allows polling at the output data 
 * rate set in ctrl reg 1. The various low power modes poll from 0.5 to 
 * 10 Hz depending on the mode.
 */
enum class PowerModes {
  POWER_DOWN = 0,
  NORMAL = 1,
  LOW_POWER_0p5 = 2,
  LOW_POWER_1 = 3,
  LOW_POWER_2 = 4,
  LOW_POWER_5 = 5,
  LOW_POWER_10 = 6
};

enum class HighPassFilterCutoffFreq {
  HPC_8 = 0,
  HPC_16 = 1,
  HPC_32 = 2,
  HPC_64 = 3
};

enum class HighPassFilterModeSelect {
  NORMAL = 0,
  REF = 1,
  NORMAL_2 = 2
};

enum class InterruptConfig {
  INTERRUPT_SOURCE = 0,
  INTERRPUT_1_OR_2_SOUCE = 1,
  DATA_READY = 3,
  BOOT_RUNNING = 4
};

enum class FullScale {
  G100 = 0,
  G200 = 1,
  NONE = 2,
  G400 = 3
};

union CtrlReg1 {
  struct {
    uint8_t x_enable : 1;
    uint8_t y_enable : 1;
    uint8_t z_enable : 1;
    uint8_t data_rate : 2;
    uint8_t power_mode_select : 3;
  } fields;
    uint8_t value;
};

union CtrlReg2 {
  struct {
    uint8_t hpcf : 2; // High pass cutoff freq
    uint8_t hpen1 : 1; // High pass enable 1
    uint8_t hpen2 : 1; // High pass enable 2
    uint8_t fds : 1; // filtered data selection
    uint8_t hpm : 2; // high pass filter mode selection
    uint8_t boot : 1; // Reboot the chip
  } fields;
    uint8_t value;
};

union CtrlReg3 {
  struct {
    uint8_t i1_cfg : 2; // Interrupt source config
    uint8_t lir1 : 1; // latch interrupt request for int 1
    uint8_t i2_cfg : 2; // Interrupt source config
    uint8_t lir2 : 1; // latch interrupt request for int 2
    uint8_t pp_od : 1; // push-pull/open-drain (0 = push-pull, 1 = open drain)
    uint8_t ihl : 1; // interrupt active high/low (0 = high, 1 = low)
  } fields;
    uint8_t value;
};

union CtrlReg4 {
  struct {
    uint8_t sim : 1; // spi mode (0 = 4-wire, 1 = 3-wire)
    uint8_t reserved : 3;
    uint8_t fs : 2; // full scale (max g rating)
    uint8_t ble : 1; // Big/Little endian (0 = big endian, 1 = little endian)
    uint8_t bdu : 1; // Block data update (0 = continuous, 1 = latch registers between MSB/LSB reads)

  } fields;
    uint8_t value;
};

union CtrlReg5 {
  struct {
    uint8_t turnon : 2; // 0 = sleep-to-wake disabled, 3 = turned on, device is in low power mode
    uint8_t reserved : 6;

  } fields;
    uint8_t value;
};

}; // namespace Config
/**
 * This class encapsulates all interactions with the accelerometer.
 *
 * The accelerometer used in this project is the H3LIS331DL.  It is a
 * high G accelerometer that accepts SPI and I2C communication.
 *
 * I2C is deprecated and will eventually be ripped out since SPI is so much faster.
 */
class Accelerometer
{
public:
  Accelerometer(uint8_t chip_select_pin);
  ~Accelerometer() = default;

  /**
   * Convert a raw accelerometer reading to g.
   *
   * @param value Raw accelerometer value
   * @return Raw accelerometer value converted to g
   */
  float convertToG(const int16_t value);

  /**
   * Configure the accelerometer over SPI.
   *
   * @return true if successful, false if not
   */
  bool configure();

  /**
   * Calibrate the accelerometer.
   *
   * The H3LIS331DL comes factory calibrated, but I've noticed that mine
   * is a little off. Run an averaging filter on a number of samples to
   * determine average adjustments to bring the returns back in line with
   * a 0g, 0g, 1g  baseline.
   *
   * TODO: Is this even needed? Not sure if the error I'm seeing is just because
   *       the accelerometer is so high g and it will become an imperceptable
   *       part of the noise at higher values.
   *       Either way, seeing ~0.75g at rest is annoying me.
   *
   * @param num_samples Number of samples to average over
   *
   * @return AcclerometerReading object with cal adjustment values for each axis
   */
  AccelerometerReading calibrate(const uint16_t num_samples = 200);

  /**
   * Check the the accelerometer has new data.
   *
   * @return true if there is new data, false if not
   */
  bool isDataReady();

  /**
   * Read axis data from the accelerometer.
   *
   * @return AccelerometerReading object with axis data
   */
  AccelerometerReading read();

  /**
   * Write data to the accelerometer.
   * Used to write config registers.
   *
   * @param addr: Address to write to
   * @param data: Byte array containing config data
   * @param size: Size of the data array to write
   *
   * @return True if write was successful, false if not
   */
  bool write(uint32_t addr, uint8_t* data, uint8_t size);

  /**
   * Check the SPI connection to the accelerometer.
   *
   * This will attempt to read the WHOAMI register of the accelerometer and compare
   * it against the valid return value for the device.
   *
   * @return true if WHOAMI register read succeeds, false if not
   */
  bool checkConnection();

  /**
   * Set the power mode of the accelerometer.
   *
   * The chip boots up into a "power down".  Normal power mode will casue the chip
   * to update data at the Output Data Rate.  The other low power modes have their
   * own associated update rates.
   *
   * @param power_mode: Power mode to use for the accelerometer
   */
  void setPowerMode(const Config::PowerModes power_mode);

  /**
   * Set the full scale value of the accelerometer.
   * This is the maximum g-force that it can read.
   *
   * @param full_scale : max g value to use
   */
  void setFullScale(Config::FullScale full_scale);

  /**
   * Set the Output Data Rate (ODR) of the accelerometer.
   * This ODR is used when the accelerometer is in normal power mode.
   *
   * @param data_rate: Data rate to set the accelerometer to
   */
  void setOutputDataRate(const Config::DataRates data_rate);

  /**
   * Set whether or not each axis is enabled.
   *
   * @param x: Enabled value of the x axis
   * @param y: Enabled value of the y axis
   * @param z: Enabled value of the z axis
   */
  void setAxisEnables(const bool x, const bool y, const bool z);

private:

  /**
   * Registered as an ISR for when the accelerometer says that data is ready.
   */
  void serviceInterrupt();

  /**
   * Internal methods for communicating over different protocols.
   */
  bool configureSPI();
  bool configureI2C();

  void readSPI(AccelerometerReading &reading);
  void readI2C(AccelerometerReading &reading);

  uint8_t readSPIByte(uint8_t address);
  void writeSPIByte(uint8_t address, uint8_t data);

  Protocol m_protocol;
  std::atomic_bool m_data_ready;
  uint16_t m_max_accel;
  AccelerometerReading m_cal_adjustments;
  uint8_t m_chip_select_pin;
};
}; // namespace Accelerometer
}; // namespace Navigation
}; // namespace Melty
