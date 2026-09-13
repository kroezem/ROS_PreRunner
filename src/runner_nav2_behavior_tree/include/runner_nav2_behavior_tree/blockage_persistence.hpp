// Copyright 2026 matti
// Licensed under the Apache License, Version 2.0

#ifndef RUNNER_NAV2_BEHAVIOR_TREE__BLOCKAGE_PERSISTENCE_HPP_
#define RUNNER_NAV2_BEHAVIOR_TREE__BLOCKAGE_PERSISTENCE_HPP_

#include <algorithm>
#include <cstddef>

namespace runner_nav2_behavior_tree
{

class BlockagePersistence
{
public:
  bool observe(const bool path_is_valid, const unsigned int required_observations)
  {
    if (path_is_valid) {
      blocked_observations_ = 0;
      return false;
    }
    blocked_observations_ = std::min(
      blocked_observations_ + 1, std::max(1u, required_observations));
    return blocked_observations_ >= std::max(1u, required_observations);
  }

  void reset() {blocked_observations_ = 0;}
  unsigned int blockedObservations() const {return blocked_observations_;}

private:
  unsigned int blocked_observations_{0};
};

}  // namespace runner_nav2_behavior_tree

#endif  // RUNNER_NAV2_BEHAVIOR_TREE__BLOCKAGE_PERSISTENCE_HPP_
