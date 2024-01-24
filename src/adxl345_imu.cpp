#include "adxl345_imu.hpp"

namespace adxl345_imu_broadcaster
{

hardware_interface::CallbackReturn ADXL345Hardware::on_init(const hardware_interface::HardwareInfo & info)
{
	if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
	{
		return hardware_interface::CallbackReturn::ERROR;
	}

	_imu.setup(info_.hardware_parameters["i2c_device"], ADXL345_DEFAULT_ADDRESS);

  	RCLCPP_INFO(rclcpp::get_logger("ADXL345Hardware"), "Successfully initialized!");
	
	return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ADXL345Hardware::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> state_interfaces;

	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &_linear_accel_x));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &_linear_accel_y));
	state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &_linear_accel_z));

	return state_interfaces;
}

hardware_interface::CallbackReturn ADXL345Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	RCLCPP_INFO(rclcpp::get_logger("ADXL345Hardware"), "Starting controller ...");

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ADXL345Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	RCLCPP_INFO(rclcpp::get_logger("ADXL345Hardware"), "Stopping Controller...");
	
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ADXL345Hardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
	float accel[3];
	_imu.getAccel(accel[0], accel[1], accel[2]); 
	
	_linear_accel_x = (double)accel[1];
	_linear_accel_y = (double)accel[0];
	_linear_accel_z = (double)accel[2];

	return hardware_interface::return_type::OK;
}

} // namespace adxl345_imu_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
	adxl345_imu_broadcaster::ADXL345Hardware,
	hardware_interface::SensorInterface)
