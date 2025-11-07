#include "openarm_hardware/v10_safemrc_hardware.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <map>

namespace openarm_hardware {

OpenArm_v10SafeMRCHW::OpenArm_v10SafeMRCHW() {
  // Default SafeMRC device entries: joint2, joint4
  mrc_ids_ = {0x01, 0x02};
  mrc_types_ = {safe_mrc::MRCType::ROTARY96, safe_mrc::MRCType::ROTARY96};
  joint_mappings_ = {"joint2", "joint4"};
}

hardware_interface::CallbackReturn OpenArm_v10SafeMRCHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  auto base = OpenArm_v10HW::on_init(info);
  if (base != hardware_interface::CallbackReturn::SUCCESS) {
    return base;
  }
  if (!parse_safemrc_config()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  initialize_safemrc();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10SafeMRCHW::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  return OpenArm_v10HW::on_configure(previous_state);
}

bool OpenArm_v10SafeMRCHW::parse_safemrc_config() {
  try {
    const auto share = ament_index_cpp::get_package_share_directory("openarm_hardware");
    const auto cfg = share + std::string("/config/rs485_devices.yaml");
    if (!std::filesystem::exists(cfg)) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10SafeMRCHW"),
                   "RS485 config not found: %s", cfg.c_str());
      return false;
    }
    YAML::Node root = YAML::LoadFile(cfg);
    
    // Parse rs485_bus configuration
    if (!root["rs485_bus"]) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10SafeMRCHW"),
                   "rs485_bus section missing in %s", cfg.c_str());
      return false;
    }
    
    auto rs485_bus = root["rs485_bus"];
    rs485_port_id_ = rs485_bus["port_id"].as<std::string>("");
    if (rs485_port_id_.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10SafeMRCHW"),
                   "rs485_bus.port_id missing in %s", cfg.c_str());
      return false;
    }
    
    // Parse devices list
    if (!rs485_bus["devices"]) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10SafeMRCHW"),
                   "rs485_bus.devices missing in %s", cfg.c_str());
      return false;
    }
    
    std::vector<uint8_t> ids;
    std::vector<safe_mrc::MRCType> types;
    std::vector<std::string> joints;
    
    for (const auto& dev : rs485_bus["devices"]) {
      ids.push_back(dev["id"].as<uint8_t>());
      const auto type_str = dev["type"].as<std::string>("ROTARY96");
      types.push_back(type_str == "ROTARY52" ? safe_mrc::MRCType::ROTARY52
                                             : safe_mrc::MRCType::ROTARY96);
      joints.push_back(dev["joint"].as<std::string>(""));
    }
    
    if (ids.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10SafeMRCHW"),
                   "No devices found in rs485_bus.devices");
      return false;
    }
    
    mrc_ids_ = ids;
    mrc_types_ = types;
    joint_mappings_ = joints;
    
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10SafeMRCHW"),
                "Loaded RS485 config: port_id=%s, devices=%zu", 
                rs485_port_id_.c_str(), mrc_ids_.size());
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10SafeMRCHW"),
                 "Failed to parse RS485 config: %s", e.what());
    return false;
  }
}

std::string OpenArm_v10SafeMRCHW::get_device_path_by_id(const std::string& dev_id) {
  const std::string by_id = "/dev/serial/by-id/" + dev_id;
  if (std::filesystem::exists(by_id)) return by_id;
  RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10SafeMRCHW"),
              "Device %s not found under /dev/serial/by-id, attempting raw path",
              by_id.c_str());
  return by_id;
}

void OpenArm_v10SafeMRCHW::initialize_safemrc() {
  try {
    const auto path = get_device_path_by_id(rs485_port_id_);
    safeguarder_ = std::make_unique<safe_mrc::Safeguarder>(path);
    safeguarder_->init_mrcs(mrc_types_, mrc_ids_);
    safeguarder_->enable_all();
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10SafeMRCHW"),
                "Initialized SafeMRC on %s with %zu devices", path.c_str(), mrc_ids_.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10SafeMRCHW"),
                 "SafeMRC init failed: %s", e.what());
  }
}

std::map<std::string, size_t> OpenArm_v10SafeMRCHW::build_joint_index_map() {
  std::map<std::string, size_t> joint_map;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joint_map[joint_names_[i]] = i;
  }
  return joint_map;
}

hardware_interface::return_type OpenArm_v10SafeMRCHW::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  // Read OpenArm (DM motors) first
  auto base = OpenArm_v10HW::read(time, period);
  if (base != hardware_interface::return_type::OK) return base;

  if (safeguarder_) {
    safeguarder_->refresh_all_states();
    const auto& mrcs = safeguarder_->get_mrc_component().get_mrcs();
    
    // Update SafeMRC device states
    if (mrcs.size() >= 1) {
      mrc2_position_ = mrcs[0].get_position();
      mrc2_velocity_ = mrcs[0].get_velocity();
      mrc2_effort_ = mrcs[0].get_current();
      mrc2_collision_ = mrcs[0].get_collision_flag();
    }
    if (mrcs.size() >= 2) {
      mrc4_position_ = mrcs[1].get_position();
      mrc4_velocity_ = mrcs[1].get_velocity();
      mrc4_effort_ = mrcs[1].get_current();
      mrc4_collision_ = mrcs[1].get_collision_flag();
    }
    
    // Build joint name to index mapping
    auto joint_map = build_joint_index_map();
    
    // Compose joint angles: DM + SafeMRC for configured joints
    for (size_t i = 0; i < joint_mappings_.size() && i < mrcs.size(); ++i) {
      const std::string& joint_name = joint_mappings_[i];
      auto it = joint_map.find(joint_name);
      if (it != joint_map.end()) {
        size_t joint_idx = it->second;
        if (joint_idx < pos_states_.size()) {
          // Add SafeMRC position and velocity to DM motor values
          pos_states_[joint_idx] += mrcs[i].get_position();
          vel_states_[joint_idx] += mrcs[i].get_velocity();
        }
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10SafeMRCHW::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  // Send current commands to SafeMRC based on joint efforts for configured joints
  if (safeguarder_) {
    std::vector<safe_mrc::MRCCmd> cmds;
    cmds.reserve(mrc_ids_.size());
    
    // Build joint name to index mapping
    auto joint_map = build_joint_index_map();
    
    for (size_t i = 0; i < joint_mappings_.size() && i < mrc_ids_.size(); ++i) {
      const std::string& joint_name = joint_mappings_[i];
      auto it = joint_map.find(joint_name);
      if (it != joint_map.end()) {
        size_t joint_idx = it->second;
        const float current = (joint_idx < tau_commands_.size()) ? 
                              static_cast<float>(tau_commands_[joint_idx]) : 0.0f;
        cmds.push_back({safe_mrc::MRCMode::FIX_LIMIT, current});
      } else {
        // Default to zero current if joint not found
        cmds.push_back({safe_mrc::MRCMode::FIX_LIMIT, 0.0f});
      }
    }
    
    safeguarder_->get_mrc_component().mrc_control_all(cmds);
  }

  // Then write DM motor commands as usual
  return OpenArm_v10HW::write(time, period);
}

} // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10SafeMRCHW,
                       hardware_interface::SystemInterface)
