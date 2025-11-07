#pragma once

#include "openarm_hardware/v10_simple_hardware.hpp"
#include <safe_mrc_device/safe_mrc/safeguarder.hpp>

namespace openarm_hardware {

class OpenArm_v10SafeMRCHW : public OpenArm_v10HW {
public:
  OpenArm_v10SafeMRCHW();

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::unique_ptr<safe_mrc::Safeguarder> safeguarder_;
  std::string rs485_port_id_;
  std::vector<uint8_t> mrc_ids_;
  std::vector<safe_mrc::MRCType> mrc_types_;
  std::vector<std::string> joint_mappings_;

  // SafeMRC device states
  double mrc2_position_{0.0}, mrc2_velocity_{0.0}, mrc2_effort_{0.0};
  bool mrc2_collision_{false};
  double mrc4_position_{0.0}, mrc4_velocity_{0.0}, mrc4_effort_{0.0};
  bool mrc4_collision_{false};

  bool parse_safemrc_config();
  std::string get_device_path_by_id(const std::string& device_id);
  void initialize_safemrc();
  std::map<std::string, size_t> build_joint_index_map();

  static inline bool is_composed_joint(size_t idx) { return idx == 1 || idx == 3; }
};

} // namespace openarm_hardware
