#include "stm32_control/flow.hpp"
#include <vector>

namespace stm32_control {

// --- default grab-tip flow --------------------------------------------------
class DefaultFlow : public Flow {
public:
  DefaultFlow() {
    auto make_step = [](const std::string& name,
                        ActionCode action,
                        TargetSlot move_target_slot) {
      Flow::StepConfig step;
      step.name = name;
      step.action = action;
      step.move_target_slot = move_target_slot;
      return step;
    };

    // build step list in the same order as old SequenceStep enum
    steps_.push_back(make_step("Idle", ActionCode::IDLE, TargetSlot::None));
    steps_.push_back(make_step("Move1", ActionCode::MOVE, TargetSlot::Move1));
    steps_.push_back(make_step("WaitMove2", ActionCode::MOVE, TargetSlot::Move1));
    steps_.push_back(make_step("Move2", ActionCode::MOVE, TargetSlot::Move2));
    steps_.push_back(make_step("Align", ActionCode::ALIGN, TargetSlot::None));
    steps_.push_back(make_step("Grip", ActionCode::GRIP, TargetSlot::None));
    steps_.push_back(make_step("WaitMove3", ActionCode::MOVE, TargetSlot::Move2));
    steps_.push_back(make_step("Move3", ActionCode::MOVE, TargetSlot::Move3));
    Flow::StepConfig done = make_step("Done", ActionCode::IDLE, TargetSlot::None);
    done.finish_sequence = true;
    steps_.push_back(done);
  }

  const StepConfig& get_step(size_t idx) const override {
    return steps_.at(idx);
  }

  size_t next_on_ack(size_t current_step,
                      bool has_move2,
                      bool has_move3) const override {
    switch (current_step) {
      case 1: // Move1
        return 4; // Align
      case 4: // Align
        return has_move2 ? 3 : 2; // Move2 or WaitMove2
      case 3: // Move2
        return 5; // Grip
      case 5: // Grip
        return has_move3 ? 7 : 6; // Move3 or WaitMove3
      case 7: // Move3
        return 8; // Done
      default:
        return current_step;
    }
  }

  const std::string& name() const override { static const std::string n = "grab_tip"; return n; }
  size_t step_count() const override { return steps_.size(); }

private:
  std::vector<StepConfig> steps_;
};

// --- climb flow skeleton ----------------------------------------------------
class ClimbFlow : public Flow {
public:
  ClimbFlow() {
    auto make_step = [](const std::string& name,
                        ActionCode action,
                        TargetSlot move_target_slot) {
      Flow::StepConfig step;
      step.name = name;
      step.action = action;
      step.move_target_slot = move_target_slot;
      return step;
    };

    steps_.push_back(make_step("Idle", ActionCode::IDLE, TargetSlot::None));

    Flow::StepConfig turn = make_step("Turn", ActionCode::ROTATE, TargetSlot::None);
    turn.rotate_direction = 1;  // default: face forward
    steps_.push_back(turn);

    Flow::StepConfig move = make_step("Move", ActionCode::MOVE, TargetSlot::Move1);
    // Choose MOVE behavior per step:
    // 1) Absolute (default): use move_target_slot.
    // 2) RelativeOffset: set move_mode + move_dx_m/move_dy_m.
    move.move_mode = Flow::MoveMode::Absolute;
    move.move_dx_m = 0.0;
    move.move_dy_m = 0.0;
    steps_.push_back(move);

    Flow::StepConfig stair = make_step("Stair", ActionCode::STAIR, TargetSlot::None);
    stair.stair_direction = 0;  // default: up stair
    steps_.push_back(stair);

    Flow::StepConfig done = make_step("Done", ActionCode::IDLE, TargetSlot::None);
    done.finish_sequence = true;
    steps_.push_back(done);
  }

  const StepConfig& get_step(size_t idx) const override {
    return steps_.at(idx);
  }

  size_t next_on_ack(size_t current_step,
                      bool /*has_move2*/, bool /*has_move3*/) const override {
    // linear sequence for now
    if (current_step + 1 < steps_.size()) {
      return current_step + 1;
    }
    return current_step;
  }

  const std::string& name() const override { static const std::string n = "climb"; return n; }
  size_t step_count() const override { return steps_.size(); }

private:
  std::vector<StepConfig> steps_;
};

// --- arm grab flow ----------------------------------------------------------
class ArmGrabFlow : public Flow {
public:
  ArmGrabFlow() {
    auto make_step = [](const std::string& name,
                        ActionCode action,
                        TargetSlot move_target_slot) {
      Flow::StepConfig step;
      step.name = name;
      step.action = action;
      step.move_target_slot = move_target_slot;
      return step;
    };

    // Minimal arm-grab sequence requested by user.
    steps_.push_back(make_step("Idle", ActionCode::IDLE, TargetSlot::None));
    steps_.push_back(make_step("Move1", ActionCode::MOVE, TargetSlot::Move1));
    steps_.push_back(make_step("ArmGrip", ActionCode::ARM_GRIP, TargetSlot::None));
    Flow::StepConfig idle_end = make_step("IdleEnd", ActionCode::IDLE, TargetSlot::None);
    idle_end.finish_sequence = true;
    steps_.push_back(idle_end);
  }

  const StepConfig& get_step(size_t idx) const override {
    return steps_.at(idx);
  }

  size_t next_on_ack(size_t current_step,
                      bool /*has_move2*/,
                      bool /*has_move3*/) const override {
    switch (current_step) {
      case 1: // Move1
        return 2; // ArmGrip
      case 2: // ArmGrip
        return 3; // IdleEnd
      default:
        return current_step;
    }
  }

  const std::string& name() const override { static const std::string n = "arm_grab"; return n; }
  size_t step_count() const override { return steps_.size(); }

private:
  std::vector<StepConfig> steps_;
};

// --- move then turn-left flow ----------------------------------------------
class MoveTurnLeftFlow : public Flow {
public:
  MoveTurnLeftFlow() {
    auto make_step = [](const std::string& name,
                        ActionCode action,
                        TargetSlot move_target_slot) {
      Flow::StepConfig step;
      step.name = name;
      step.action = action;
      step.move_target_slot = move_target_slot;
      return step;
    };

    steps_.push_back(make_step("Idle", ActionCode::IDLE, TargetSlot::None));
    steps_.push_back(make_step("Move", ActionCode::MOVE, TargetSlot::Move1));

    Flow::StepConfig rotate_left = make_step("RotateLeft", ActionCode::ROTATE, TargetSlot::None);
    rotate_left.rotate_direction = 1;  // 1: left
    steps_.push_back(rotate_left);

    Flow::StepConfig idle_end = make_step("IdleEnd", ActionCode::IDLE, TargetSlot::None);
    idle_end.finish_sequence = true;
    steps_.push_back(idle_end);
  }

  const StepConfig& get_step(size_t idx) const override {
    return steps_.at(idx);
  }

  size_t next_on_ack(size_t current_step,
                      bool /*has_move2*/,
                      bool /*has_move3*/) const override {
    switch (current_step) {
      case 1: // Move
        return 2; // RotateLeft
      case 2: // RotateLeft
        return 3; // IdleEnd
      default:
        return current_step;
    }
  }

  const std::string& name() const override { static const std::string n = "move_turn_left"; return n; }
  size_t step_count() const override { return steps_.size(); }

private:
  std::vector<StepConfig> steps_;
};

// --- stag insert flow ------------------------------------------------------
class StagInsertFlow : public Flow {
public:
  StagInsertFlow() {
    auto make_step = [](const std::string& name,
                        ActionCode action,
                        TargetSlot move_target_slot) {
      Flow::StepConfig step;
      step.name = name;
      step.action = action;
      step.move_target_slot = move_target_slot;
      return step;
    };

    // Sequence: IDLE -> WaitMove1 -> Move1 -> STICK_IN -> IDLE
    steps_.push_back(make_step("Idle", ActionCode::IDLE, TargetSlot::None));
    steps_.push_back(make_step("WaitMove1", ActionCode::IDLE, TargetSlot::None));
    steps_.push_back(make_step("Move1", ActionCode::MOVE, TargetSlot::Move1));
    steps_.push_back(make_step("StickIn", ActionCode::STICK_IN, TargetSlot::None));

    Flow::StepConfig idle_end = make_step("IdleEnd", ActionCode::IDLE, TargetSlot::None);
    idle_end.finish_sequence = true;
    steps_.push_back(idle_end);
  }

  const StepConfig& get_step(size_t idx) const override {
    return steps_.at(idx);
  }

  size_t next_on_ack(size_t current_step,
                      bool /*has_move2*/,
                      bool /*has_move3*/) const override {
    switch (current_step) {
      case 2: // Move1
        return 3; // StickIn
      case 3: // StickIn
        return 4; // IdleEnd
      default:
        return current_step;
    }
  }

  const std::string& name() const override { static const std::string n = "stag_insert"; return n; }
  size_t step_count() const override { return steps_.size(); }

private:
  std::vector<StepConfig> steps_;
};

// --- arm grip only debug flow ----------------------------------------------
class ArmGripOnlyDebugFlow : public Flow {
public:
  ArmGripOnlyDebugFlow() {
    auto make_step = [](const std::string& name,
                        ActionCode action,
                        TargetSlot move_target_slot) {
      Flow::StepConfig step;
      step.name = name;
      step.action = action;
      step.move_target_slot = move_target_slot;
      return step;
    };

    steps_.push_back(make_step("ArmGrip", ActionCode::ARM_GRIP, TargetSlot::None));

    Flow::StepConfig idle_end = make_step("IdleEnd", ActionCode::IDLE, TargetSlot::None);
    idle_end.finish_sequence = true;
    steps_.push_back(idle_end);
  }

  const StepConfig& get_step(size_t idx) const override {
    return steps_.at(idx);
  }

  size_t next_on_ack(size_t current_step,
                      bool /*has_move2*/,
                      bool /*has_move3*/) const override {
    switch (current_step) {
      case 0: // ArmGrip
        return 1; // IdleEnd
      default:
        return current_step;
    }
  }

  const std::string& name() const override { static const std::string n = "arm_grip_only_debug"; return n; }
  bool auto_activate_on_set() const override { return true; }
  size_t step_count() const override { return steps_.size(); }

private:
  std::vector<StepConfig> steps_;
};

// --- arm grip fixed trajectory debug flow ----------------------------------
class ArmGripFixedDebugFlow : public Flow {
public:
  ArmGripFixedDebugFlow() {
    auto make_step = [](const std::string& name,
                        ActionCode action,
                        TargetSlot move_target_slot) {
      Flow::StepConfig step;
      step.name = name;
      step.action = action;
      step.move_target_slot = move_target_slot;
      return step;
    };

    // Keep a single ARM_GRIP step so runtime can repeatedly send
    // fixed arm trajectory points to STM32 for integration testing.
    steps_.push_back(make_step("ArmGripFixed", ActionCode::ARM_GRIP, TargetSlot::None));

    Flow::StepConfig idle_end = make_step("IdleEnd", ActionCode::IDLE, TargetSlot::None);
    idle_end.finish_sequence = true;
    steps_.push_back(idle_end);
  }

  const StepConfig& get_step(size_t idx) const override {
    return steps_.at(idx);
  }

  size_t next_on_ack(size_t current_step,
                      bool /*has_move2*/,
                      bool /*has_move3*/) const override {
    switch (current_step) {
      case 0: // ArmGripFixed
        return 1; // IdleEnd
      default:
        return current_step;
    }
  }

  const std::string& name() const override { static const std::string n = "arm_grip_fixed_debug"; return n; }
  bool auto_activate_on_set() const override { return true; }
  bool uses_fixed_arm_debug_trajectory() const override { return true; }
  size_t step_count() const override { return steps_.size(); }

private:
  std::vector<StepConfig> steps_;
};

// helper to register built-in flows
void register_builtin_flows() {
  auto &reg = FlowRegistry::instance();
  reg.register_flow("grab_tip", [](){ return std::make_shared<DefaultFlow>(); });
  reg.register_flow("climb", [](){ return std::make_shared<ClimbFlow>(); });
  reg.register_flow("arm_grab", [](){ return std::make_shared<ArmGrabFlow>(); });
  reg.register_flow("move_turn_left", [](){ return std::make_shared<MoveTurnLeftFlow>(); });
  reg.register_flow("stag_insert", [](){ return std::make_shared<StagInsertFlow>(); });
  reg.register_flow("arm_grip_only_debug", [](){ return std::make_shared<ArmGripOnlyDebugFlow>(); });
  reg.register_flow("arm_grip_fixed_debug", [](){ return std::make_shared<ArmGripFixedDebugFlow>(); });
}

} // namespace stm32_control
