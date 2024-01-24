#ifndef ADXL345_IMU_BROADCASTER__ADXL345_IMU_HPP_
#define ADXL345_IMU_BROADCASTER__ADXL345_IMU_HPP_

#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"

#include "adxl345.h"

namespace adxl345_imu_broadcaster
{

class ADXL345Hardware : public hardware_interface::SensorInterface
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ADXL345Hardware);

    ADXL345_IMU_BROADCASTER_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    ADXL345_IMU_BROADCASTER_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    ADXL345_IMU_BROADCASTER_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    ADXL345_IMU_BROADCASTER_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    ADXL345_IMU_BROADCASTER_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
    double _linear_accel_x;
    double _linear_accel_y;
    double _linear_accel_z;

    ADXL345 _imu;
};

} // namespace adxl345_imu_broadcaster

#endif  // ADXL345_IMU_BROADCASTER__ADXL345_IMU_HPP_