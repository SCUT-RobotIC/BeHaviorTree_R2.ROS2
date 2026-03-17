#include "stm32_control/flow.hpp"

namespace stm32_control {

FlowRegistry& FlowRegistry::instance() {
  static FlowRegistry registry;
  return registry;
}

void FlowRegistry::register_flow(const std::string& name, Factory creator) {
  registry_[name] = std::move(creator);
}

std::shared_ptr<Flow> FlowRegistry::create(const std::string& name) const {
  auto it = registry_.find(name);
  if (it == registry_.end()) {
    return nullptr;
  }
  return it->second();
}

std::vector<std::string> FlowRegistry::available() const {
  std::vector<std::string> names;
  for (const auto& kv : registry_) {
    names.push_back(kv.first);
  }
  return names;
}

} // namespace stm32_control
