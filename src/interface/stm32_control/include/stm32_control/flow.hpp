#ifndef STM32_CONTROL__FLOW_HPP_
#define STM32_CONTROL__FLOW_HPP_

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include "stm32_control/action_codes.hpp"  // need action codes for StepConfig

namespace stm32_control {

// slot definitions moved out of ActionHandler so that flows can reference them
enum class TargetSlot {
  None,
  Move1,
  Move2,
  Move3
};

// forward declaration of Flow (abstract base class for a sequence of steps)
class Flow {
public:
  enum class MoveMode {
    Absolute,
    RelativeOffset
  };

  struct StepConfig {
    std::string name;
    ActionCode action;
    // Used only when action == MOVE and move_mode == Absolute.
    TargetSlot move_target_slot = TargetSlot::None;
    MoveMode move_mode = MoveMode::Absolute;
    // Used only when move_mode == RelativeOffset (unit: meter).
    // The target sent to STM32 will be (current_x + move_dx_m, current_y + move_dy_m).
    double move_dx_m = 0.0;
    double move_dy_m = 0.0;
    // Used only for ROTATE action; 0:前, 1:左, 2:后, 3:右.
    std::optional<int16_t> rotate_direction;
    // Used only for STAIR action; 0:上台阶, 1:下台阶.
    std::optional<int16_t> stair_direction;
    bool finish_sequence = false;
  };

  virtual ~Flow() = default;

  // return configuration for a given step index (0-based)
  virtual const StepConfig& get_step(size_t idx) const = 0;

  // determine next index when ack arrives; flows may inspect whether move2/move3 targets exist
  virtual size_t next_on_ack(size_t current_step,
                              bool has_move2,
                              bool has_move3) const = 0;

  // index of the first active step; usually 0 (idle)
  virtual size_t initial_step() const { return 0; }

  // If true, ActionHandler should activate sequence immediately after flow is set.
  virtual bool auto_activate_on_set() const { return false; }

  // If true, runtime should load and loop the fixed arm debug trajectory.
  virtual bool uses_fixed_arm_debug_trajectory() const { return false; }

  virtual const std::string& name() const = 0;

  virtual size_t step_count() const = 0;
};

// registry holding available flows by name
class FlowRegistry {
public:
  using Factory = std::function<std::shared_ptr<Flow>()>;

  static FlowRegistry& instance();

  void register_flow(const std::string& name, Factory creator);
  std::shared_ptr<Flow> create(const std::string& name) const;
  std::vector<std::string> available() const;

private:
  std::map<std::string, Factory> registry_;
};

// register the built-in flows (grab_tip, climb, etc.)
void register_builtin_flows();

} // namespace stm32_control

#endif // STM32_CONTROL__FLOW_HPP_
