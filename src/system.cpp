#include "cubemars_hardware/system.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cubemars_hardware
{
CubeMarsSystemHardware::~CubeMarsSystemHardware()
{
  // If the controller manager is shutdown via Ctrl + C
  on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn
CubeMarsSystemHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("can_interface") != 0)
  {
    can_itf_ = info_.hardware_parameters.at("can_interface");
  }
  else
  {
    RCLCPP_FATAL(rclcpp::get_logger("CubeMarsSystemHardware"),
                 "No can_interface specified in URDF");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  last_pos_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_mode_.resize(info_.joints.size(), control_mode_t::UNDEFINED);

  for (const hardware_interface::ComponentInfo &joint : info_.joints)
  {
    if (joint.parameters.count("can_id") != 0 && joint.parameters.count("kt") != 0 &&
        joint.parameters.count("pole_pairs") != 0 && joint.parameters.count("gear_ratio") != 0)
    {
      can_ids_.emplace_back(std::stoul(joint.parameters.at("can_id")));
      torque_constants_.emplace_back(std::stod(joint.parameters.at("kt")));
      double erpm_conversion = std::stoi(joint.parameters.at("pole_pairs")) *
                               std::stoi(joint.parameters.at("gear_ratio")) * 60 / (2 * M_PI);
      erpm_conversions_.emplace_back(erpm_conversion);

      if (joint.parameters.count("acc_limit") != 0 && joint.parameters.count("vel_limit") != 0)
      {
        std::pair<std::int32_t, std::int32_t> limits;
        limits.first = std::stoi(joint.parameters.at("vel_limit")) / 10 * erpm_conversion;
        limits.second = std::stoi(joint.parameters.at("acc_limit")) / 10 * erpm_conversion;
        if (limits.first >= 32767 || limits.first <= 0)
        {
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                       "velocity limit is not in range 0-32767: %d", limits.first);
          return hardware_interface::CallbackReturn::ERROR;
        }
        if (limits.second >= 32767 || limits.second <= 0)
        {
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                       "acceleration limit is not in range 0-32767: %d", limits.second);
          return hardware_interface::CallbackReturn::ERROR;
        }
        limits_.emplace_back(limits);
      }
      else
      {
        limits_.emplace_back(std::make_pair(0, 0));
      }
    }
    else
    {
      RCLCPP_FATAL(rclcpp::get_logger("CubeMarsSystemHardware"),
                   "Missing parameters in URDF for %s", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.parameters.count("pos_limit_min") != 0 &&
        joint.parameters.count("pos_limit_max") != 0)
    {
      std::pair<double, double> limits;
      limits.first = std::stod(joint.parameters.at("pos_limit_min"));
      limits.second = std::stod(joint.parameters.at("pos_limit_max"));

      if (limits.first >= limits.second)
      {
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                     "position limit min is not less than max: %f, %f", limits.first,
                     limits.second);
        return hardware_interface::CallbackReturn::ERROR;
      }

      position_limits_.emplace_back(limits);
    }
    else
    {
      position_limits_.emplace_back(std::make_pair(0, 0));
    }

    // Optional hardware-side slew-rate cap on position commands [rad/s].
    if (joint.parameters.count("max_velocity") != 0 &&
        std::stod(joint.parameters.at("max_velocity")) > 0)
    {
      max_velocities_.emplace_back(std::stod(joint.parameters.at("max_velocity")));
    }
    else
    {
      max_velocities_.emplace_back(0);
    }

    if (joint.parameters.count("enc_off") != 0)
    {
      enc_offs_.emplace_back(std::stod(joint.parameters.at("enc_off")));
    }
    else
    {
      enc_offs_.emplace_back(0);
    }

    if (joint.parameters.count("trq_limit") != 0 && std::stod(joint.parameters.at("trq_limit")) > 0)
    {
      trq_limits_.emplace_back(std::stod(joint.parameters.at("trq_limit")));
    }
    else
    {
      trq_limits_.emplace_back(0);
    }

    if (joint.parameters.count("read_only") != 0 &&
        std::stoi(joint.parameters.at("read_only")) == 1)
    {
      read_only_.emplace_back(true);
    }
    else
    {
      read_only_.emplace_back(false);
    }

    // Impedance gains. If imp_kp is set, the position command interface is
    // realized as a host-side impedance law over the current loop instead of
    // the servo position loop:
    //   tau = imp_kp * (pos_cmd - pos) + imp_kd * (vel_cmd - vel) + effort_cmd
    if (joint.parameters.count("imp_kp") != 0)
    {
      double kp = std::stod(joint.parameters.at("imp_kp"));
      double kd = joint.parameters.count("imp_kd") != 0
                      ? std::stod(joint.parameters.at("imp_kd"))
                      : 0.0;
      if (kp < 0 || kd < 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                     "impedance gains must be non-negative: imp_kp=%f, imp_kd=%f", kp, kd);
        return hardware_interface::CallbackReturn::ERROR;
      }
      imp_kp_.emplace_back(kp);
      imp_kd_.emplace_back(kd);
      impedance_.emplace_back(true);
    }
    else
    {
      imp_kp_.emplace_back(0);
      imp_kd_.emplace_back(0);
      impedance_.emplace_back(false);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
CubeMarsSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  const bool connected = can_.connect(can_itf_, can_ids_, 0xFFU);
  comms_active_ = connected;

  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Communication active");

  return connected ? hardware_interface::CallbackReturn::SUCCESS
                   : hardware_interface::CallbackReturn::FAILURE;
}

hardware_interface::CallbackReturn
CubeMarsSystemHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Idempotent: the destructor also calls on_cleanup, and a hard shutdown may
  // skip on_deactivate, so guard against operating on an already-closed socket.
  if (!comms_active_)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Make sure no motor is left executing its last command while we tear down.
  stop_all_motors();

  const hardware_interface::CallbackReturn result =
      can_.disconnect() ? hardware_interface::CallbackReturn::SUCCESS
                        : hardware_interface::CallbackReturn::FAILURE;
  comms_active_ = false;

  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Communication closed");

  return result;
}

std::vector<hardware_interface::StateInterface> CubeMarsSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, "temperature", &hw_states_temperatures_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CubeMarsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION,
        &hw_commands_accelerations_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type CubeMarsSystemHardware::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces)
{
  stop_modes_.clear();
  start_modes_.clear();

  stop_modes_.resize(info_.joints.size(), false);

  // Define allowed combination of command interfaces
  std::unordered_set<std::string> eff{"effort"};
  std::unordered_set<std::string> vel{"velocity"};
  std::unordered_set<std::string> pos{"position"};

  std::unordered_set<std::string> joint_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    // find stop modes
    for (std::string key : stop_interfaces)
    {
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "stop interface: %s", key.c_str());
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        stop_modes_[i] = true;
        break;
      }
    }

    // find start modes
    joint_interfaces.clear();
    for (std::string key : start_interfaces)
    {
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "start interface: %s", key.c_str());
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        joint_interfaces.insert(key.substr(key.find("/") + 1));
      }
    }
    if (joint_interfaces == eff)
    {
      start_modes_.push_back(CURRENT_LOOP);
    }
    else if (joint_interfaces == vel)
    {
      start_modes_.push_back(SPEED_LOOP);
    }
    else if (joint_interfaces == pos)
    {
      if (impedance_[i])
      {
        start_modes_.push_back(IMPEDANCE);
      }
      else if (limits_[i].first == 0 || limits_[i].second == 0)
      {
        start_modes_.push_back(POSITION_LOOP);
      }
      else
      {
        start_modes_.push_back(POSITION_SPEED_LOOP);
      }
    }
    else if (joint_interfaces.empty())
    {
      if (stop_modes_[i])
      {
        start_modes_.push_back(UNDEFINED);
      }
      else
      {
        // don't change control mode
        start_modes_.push_back(control_mode_[i]);
      }
    }
    else
    {
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CubeMarsSystemHardware::perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (stop_modes_[i])
    {
      hw_commands_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_commands_velocities_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
      // The motor would otherwise keep executing its previous command until a
      // new controller claims it, so command an explicit stop on release.
      if (!read_only_[i])
      {
        stop_motor(i);
      }
    }
    // switch control mode
    control_mode_[i] = start_modes_[i];

    // Seed the slew limiter from the current measured position so the first
    // position command after a (re)claim ramps from where the joint actually
    // is, instead of from a stale target.
    if (start_modes_[i] == POSITION_LOOP || start_modes_[i] == POSITION_SPEED_LOOP ||
        start_modes_[i] == IMPEDANCE)
    {
      last_pos_commands_[i] = hw_states_positions_[i];
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn
CubeMarsSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
CubeMarsSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop every motor when the hardware is deactivated (e.g. controllers being
  // shut down) so nothing keeps spinning on its last commanded speed/current.
  stop_all_motors();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CubeMarsSystemHardware::read(const rclcpp::Time & /*time*/,
                                                             const rclcpp::Duration & /*period*/)
{
  bool all_ids[can_ids_.size()] = {false};
  std::uint32_t read_id;
  std::uint8_t read_data[8] = {0};
  std::uint8_t read_len;

  std::int16_t pos_int;

  // read all buffered CAN messages
  while (can_.read_nonblocking(read_id, read_data, read_len))
  {
    // Only trust frames coming from one of our motor CAN IDs. Bus noise or
    // frames from other devices that happen to pass the mask must not be
    // parsed as status, otherwise they corrupt the position/velocity state
    // that the limit filter relies on.
    auto it = std::find(can_ids_.begin(), can_ids_.end(), read_id);
    if (it == can_ids_.end())
    {
      continue;
    }

    // A servo status frame is always 8 bytes. A shorter frame would leave us
    // parsing stale/garbage bytes (fault, temperature, position), so drop it.
    if (read_len < 8)
    {
      RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "Ignoring malformed CAN frame (len %u) from CAN ID %u", read_len, read_id);
      continue;
    }

    if (read_data[7] != 0)
    {
      switch (read_data[7])
      {
      case 1:
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Motor over-temperature fault.");
        break;
      case 2:
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Over-current fault.");
        break;
      case 3:
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Over-voltage fault.");
        break;
      case 4:
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Under-voltage fault.");
        break;
      case 5:
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Encoder fault.");
        break;
      case 6:
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                     "MOSFET over-temperature fault.");
        break;
      case 7:
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Motor stall.");
        break;
      }
    }

    int i = std::distance(can_ids_.begin(), it);
    all_ids[i] = true;
    pos_int = read_data[0] << 8 | read_data[1];
    if (std::abs(pos_int) >= 32000)
    {
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "Position has reached maximum possible value.");
    }
    hw_states_positions_[i] = pos_int;
    hw_states_velocities_[i] = std::int16_t(read_data[2] << 8 | read_data[3]);
    hw_states_efforts_[i] = std::int16_t(read_data[4] << 8 | read_data[5]);
  }

  // check if all CAN IDs have received a message
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (!all_ids[i])
    {
      RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "No CAN message received from CAN ID: %u. ", can_ids_[i]);
    }
    else
    {
      // Unit conversions
      hw_states_positions_[i] = hw_states_positions_[i] * 0.1 * M_PI / 180 - enc_offs_[i];
      hw_states_velocities_[i] = hw_states_velocities_[i] * 10 / erpm_conversions_[i];
      hw_states_efforts_[i] = hw_states_efforts_[i] * 0.01 * torque_constants_[i] *
                              std::stoi(info_.joints[i].parameters.at("gear_ratio"));
      hw_states_temperatures_[i] = read_data[6];
      if (position_limits_[i].first != 0 || position_limits_[i].second != 0)
      {
        if (hw_states_positions_[i] < position_limits_[i].first ||
            hw_states_positions_[i] > position_limits_[i].second)
        {
          // Throw warn, but don't disable motor
          RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                      "Joint %lu went out of position limits.", i);
        }
      }
      if (trq_limits_[i] != 0 && hw_states_efforts_[i] > trq_limits_[i])
      {
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                     "Joint %lu went over torque limit.", i);

        // disable motor
        std::uint8_t data[4] = {0, 0, 0, 0};
        can_.write_message(can_ids_[i] | CURRENT_LOOP << 8, data, 4);

        return hardware_interface::return_type::ERROR;
      }
      if (read_only_[i])
      {
        // RCLCPP_INFO(
        //   rclcpp::get_logger("CubeMarsSystemHardware"),
        //   "read states joint %lu: pos %f, spd %f, eff %f, temp %f",
        //   i, hw_states_positions_[i], hw_states_velocities_[i], hw_states_efforts_[i],
        //   hw_states_temperatures_[i]);
        RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Joint %lu: pos: %f", i,
                    hw_states_positions_[i]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CubeMarsSystemHardware::write(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration &period)
{
  const double dt = period.seconds();
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (!read_only_[i])
    {
      switch (control_mode_[i])
      {
      case UNDEFINED:
      {
        // RCLCPP_INFO(
        //   rclcpp::get_logger("CubeMarsSystemHardware"),
        //   "Nothing is using the hardware interface!");
        break;
      }
      case CURRENT_LOOP:
      {
        if (std::isfinite(hw_commands_efforts_[i]))
        {
          std::int32_t current = hw_commands_efforts_[i] * 1000 / torque_constants_[i];
          if (std::abs(current) >= 60000)
          {
            RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                         "current command is over maximal allowed value of 60000: %d", current);
            return hardware_interface::return_type::ERROR;
          }
          // RCLCPP_INFO(
          //   rclcpp::get_logger("CubeMarsSystemHardware"),
          //   "current command for joint %lu: %d", i, current);

          // filter command to be within limits to avoid out of range commands
          if (accept_command_direction(current, position_limits_[i], hw_states_positions_[i],
                                       enc_offs_[i], control_mode_[i]))
          {
            std::uint8_t data[4];
            data[0] = current >> 24;
            data[1] = current >> 16;
            data[2] = current >> 8;
            data[3] = current;

            can_.write_message(can_ids_[i] | CURRENT_LOOP << 8, data, 4);
          }
          else
          {
            // Violate limits, stop motor
            stop_motor(i);
          }
        }
        break;
      }
      case IMPEDANCE:
      {
        if (std::isfinite(hw_commands_positions_[i]))
        {
          // Host-side impedance law, realized over the current loop. The
          // velocity reference is zero unless a velocity command is claimed,
          // and the effort command (if claimed) acts as a torque feedforward.
          double vel_cmd = std::isnan(hw_commands_velocities_[i]) ? 0.0 : hw_commands_velocities_[i];
          double tau_ff = std::isnan(hw_commands_efforts_[i]) ? 0.0 : hw_commands_efforts_[i];
          double pos_cmd = sanitize_position_command(i, hw_commands_positions_[i], dt);
          double tau = imp_kp_[i] * (pos_cmd - hw_states_positions_[i]) +
                       imp_kd_[i] * (vel_cmd - hw_states_velocities_[i]) + tau_ff;

          std::int32_t current = tau * 1000 / torque_constants_[i];
          if (std::abs(current) >= 60000)
          {
            RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                         "impedance current command is over maximal allowed value of 60000: %d",
                         current);
            return hardware_interface::return_type::ERROR;
          }

          // filter command to be within limits to avoid out of range commands
          if (accept_command_direction(current, position_limits_[i], hw_states_positions_[i],
                                       enc_offs_[i], control_mode_[i]))
          {
            std::uint8_t data[4];
            data[0] = current >> 24;
            data[1] = current >> 16;
            data[2] = current >> 8;
            data[3] = current;

            can_.write_message(can_ids_[i] | CURRENT_LOOP << 8, data, 4);
          }
          else
          {
            // Violate limits, stop motor
            stop_motor(i);
          }
        }
        break;
      }
      case SPEED_LOOP:
      {
        if (std::isfinite(hw_commands_velocities_[i]))
        {
          std::int32_t speed = hw_commands_velocities_[i] * erpm_conversions_[i];
          if (std::abs(speed) >= 100000)
          {
            RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                         "speed command is over maximal allowed value of 100000: %d", speed);
            return hardware_interface::return_type::ERROR;
          }
          // RCLCPP_INFO(
          //   rclcpp::get_logger("CubeMarsSystemHardware"),
          //   "speed command for joint %lu: %d", i, speed);

          // filter command to be within limits to avoid out of range commands
          if (accept_command_direction(speed, position_limits_[i], hw_states_positions_[i],
                                       enc_offs_[i], control_mode_[i]))
          {
            std::uint8_t data[4];
            data[0] = speed >> 24;
            data[1] = speed >> 16;
            data[2] = speed >> 8;
            data[3] = speed;

            can_.write_message(can_ids_[i] | SPEED_LOOP << 8, data, 4);
          }
          else
          {
            // Violate limits, stop motor
            stop_motor(i);
          }
        }
        break;
      }
      case POSITION_LOOP:
      {
        if (std::isfinite(hw_commands_positions_[i]))
        {
          double pos_cmd = sanitize_position_command(i, hw_commands_positions_[i], dt);
          std::int32_t position = (pos_cmd + enc_offs_[i]) * 10000 * 180 / M_PI;
          if (std::abs(position) >= 360000000)
          {
            RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                         "position command is over maximal allowed value of 360000000: %d",
                         position);
            return hardware_interface::return_type::ERROR;
          }
          // RCLCPP_INFO(
          //   rclcpp::get_logger("CubeMarsSystemHardware"),
          //   "position command for joint %lu: %d", i, position);

          // filter command to be within limits to avoid out of range commands
          if (accept_command_direction(position, position_limits_[i], hw_states_positions_[i],
                                       enc_offs_[i], control_mode_[i]))
          {
            std::uint8_t data[4];
            data[0] = position >> 24;
            data[1] = position >> 16;
            data[2] = position >> 8;
            data[3] = position;

            can_.write_message(can_ids_[i] | POSITION_LOOP << 8, data, 4);
          }
          else
          {
            // Violate limits, stop motor
            stop_motor(i);
          }
        }
        break;
      case POSITION_SPEED_LOOP:
      {
        if (std::isfinite(hw_commands_positions_[i]))
        {
          double pos_cmd = sanitize_position_command(i, hw_commands_positions_[i], dt);
          std::int32_t position = (pos_cmd + enc_offs_[i]) * 10000 * 180 / M_PI;
          std::int16_t vel = limits_[i].first;
          std::int16_t acc = limits_[i].second;
          if (std::abs(position) >= 360000000)
          {
            RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                         "position command is over maximal allowed value of 360000000: %d",
                         position);
            return hardware_interface::return_type::ERROR;
          }
          // RCLCPP_INFO(
          //   rclcpp::get_logger("CubeMarsSystemHardware"),
          //   "command for joint %lu: pos %d, vel %d, acc %d",
          //   i, position, vel, acc);

          // filter command to be within limits to avoid out of range commands
          if (accept_command_direction(position, position_limits_[i], hw_states_positions_[i],
                                       enc_offs_[i], control_mode_[i]))
          {
            std::uint8_t data[8];
            data[0] = position >> 24;
            data[1] = position >> 16;
            data[2] = position >> 8;
            data[3] = position;
            data[4] = vel >> 8;
            data[5] = vel;
            data[6] = acc >> 8;
            data[7] = acc;

            can_.write_message(can_ids_[i] | POSITION_SPEED_LOOP << 8, data, 8);
          }
        }
        else
        {
          // Violate limits, stop motor
          stop_motor(i);
        }
        break;
      }
      }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

bool CubeMarsSystemHardware::accept_command_direction(std::int32_t command,
                                                      std::pair<double, double> pos_limit,
                                                      double current_pos, double enc_offset,
                                                      const control_mode_t mode)
{
  // skip if limit is not set
  if (pos_limit.first == 0 && pos_limit.second == 0)
  {
    return true;
  }

  switch (mode)
  {
  case CURRENT_LOOP:
  case SPEED_LOOP:
  case IMPEDANCE:
  {
    // These modes have no firmware position bound: the live position reading is
    // the only thing keeping the joint inside its limits. If we have no valid
    // reading (no CAN status yet, or a dropped/garbage frame) we cannot enforce
    // the limit, so refuse to drive rather than command blindly.
    if (std::isnan(current_pos))
    {
      RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "Refusing command %d: no valid position feedback to enforce limits", command);
      return false;
    }

    // Restrict command to be within limits to avoid out of range commands
    // violate minimum limit
    if (current_pos < pos_limit.first && command < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "Ignoring command %d, joint %f is below min limit %f", command, current_pos,
                  pos_limit.first);
      return false;
    }

    // violate maximum limit
    if (current_pos > pos_limit.second && command > 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "Ignoring command %d, joint %f is above max limit %f", command, current_pos,
                  pos_limit.second);
      return false;
    }

    break;
  }

  case POSITION_LOOP:
  case POSITION_SPEED_LOOP:
  { // Restrict command to be within limits to avoid out of range commands
    // Scale radian to int32_t
    const double scale = 10000.0 * 180.0 / M_PI;
    std::int32_t limit_min = static_cast<std::int32_t>((pos_limit.first + enc_offset) * scale);
    std::int32_t limit_max = static_cast<std::int32_t>((pos_limit.second + enc_offset) * scale);

    // Check if the command is within the limits
    if (command < limit_min)
    {
      RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "Ignoring command %d, joint %f is below min limit %d", command, current_pos,
                  limit_min);
      return false;
    }
    else if (command > limit_max)
    {
      RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "Ignoring command %d, joint %f is above max limit %d", command, current_pos,
                  limit_max);
      return false;
    }

    break;
  }

  default:
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
                 "Control mode %d not supported for command filtering", mode);
    return false;
  }

  // no limit violation or command is in the direction of the limit
  return true;
}

bool CubeMarsSystemHardware::stop_motor(std::size_t joint_index)
{
  if (joint_index >= can_ids_.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Joint index %lu is out of range",
                 joint_index);
    return false;
  }

  std::int32_t speed = 0;
  std::uint8_t data[4];
  data[0] = speed >> 24;
  data[1] = speed >> 16;
  data[2] = speed >> 8;
  data[3] = speed;

  can_.write_message(can_ids_[joint_index] | SPEED_LOOP << 8, data, 4);

  return true;
}

double CubeMarsSystemHardware::sanitize_position_command(std::size_t joint_index, double command,
                                                         double dt)
{
  const std::pair<double, double> &lim = position_limits_[joint_index];

  // 1) Hard-clamp to the configured joint position limits. This is the last
  //    line of defense: even a command that slipped past every upstream check
  //    cannot be sent out of range.
  if (lim.first != 0.0 || lim.second != 0.0)
  {
    const double clamped = std::clamp(command, lim.first, lim.second);
    if (clamped != command)
    {
      RCLCPP_WARN(rclcpp::get_logger("CubeMarsSystemHardware"),
                  "Clamped joint %lu position command %f to limit [%f, %f]", joint_index, command,
                  lim.first, lim.second);
      command = clamped;
    }
  }

  // 2) Rate-limit the change relative to the previous command so a sudden jump
  //    (garbage setpoint, controller glitch) cannot become a fast, large move.
  if (max_velocities_[joint_index] > 0.0 && std::isfinite(last_pos_commands_[joint_index]) &&
      dt > 0.0)
  {
    const double max_step = max_velocities_[joint_index] * dt;
    const double delta =
        std::clamp(command - last_pos_commands_[joint_index], -max_step, max_step);
    command = last_pos_commands_[joint_index] + delta;
  }

  last_pos_commands_[joint_index] = command;
  return command;
}

void CubeMarsSystemHardware::stop_all_motors()
{
  if (!comms_active_)
  {
    return;
  }
  for (std::size_t i = 0; i < can_ids_.size(); i++)
  {
    if (!read_only_[i])
    {
      stop_motor(i);
      control_mode_[i] = UNDEFINED;
    }
  }
}

} // namespace cubemars_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cubemars_hardware::CubeMarsSystemHardware,
                       hardware_interface::SystemInterface)
