#include "adxl345.h"
#include <rclcpp/rclcpp.hpp>

ADXL345::ADXL345(std::string device, int8_t addr)
{
    setup(device, addr);
}

ADXL345::~ADXL345()
{
    close(_f_dev);
}

void ADXL345::setup(std::string device, int8_t addr)
{
	int status;

    char device_str[20];
    sprintf(device_str, "/dev/%s", device.c_str());

    // Open the i2c device
	_f_dev = open(device_str, O_RDWR);
	if (_f_dev < 0) {
        RCLCPP_FATAL(rclcpp::get_logger("AMRSystemHardware"),"Failed to open: %s", device_str);
	}

    // Set the i2c addr
	status = ioctl(_f_dev, I2C_SLAVE, addr);
	if (status < 0) {
        RCLCPP_FATAL(rclcpp::get_logger("AMRSystemHardware"),"Failed to connect to addr: %d", addr);
	}

    // Set the data rate
    dataRate_t dataRate = ADXL345_DATARATE_100_HZ;
    i2c_smbus_write_byte_data(_f_dev, ADXL345_REG_BW_RATE, dataRate);

    // Set the range
    u_int8_t range = i2c_smbus_read_byte_data(_f_dev, ADXL345_REG_DATA_FORMAT);
    range |= ADXL345_RANGE_2_G;
    range |= 0x08; // Set FULL-RES bit to enable range scaling
    i2c_smbus_write_byte_data(_f_dev, ADXL345_REG_DATA_FORMAT, range);

    // Enable measurements
    i2c_smbus_write_byte_data(_f_dev, ADXL345_REG_POWER_CTL, 0x08);
}

void ADXL345::getAccel(float &x, float &y, float &z)
{
    // Read in acceleration data
    uint8_t bytes[6];
    i2c_smbus_read_i2c_block_data(_f_dev, ADXL345_REG_DATAX0, 6, bytes);

    x = (float)((int16_t)(bytes[1] << 8 | bytes[0])) * 0.004f;
    y = (float)((int16_t)(bytes[3] << 8 | bytes[2])) * 0.004f;
    z = (float)((int16_t)(bytes[5] << 8 | bytes[4])) * 0.004f;
}