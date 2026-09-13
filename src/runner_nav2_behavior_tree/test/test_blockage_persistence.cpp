// Copyright 2026 matti
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include "runner_nav2_behavior_tree/blockage_persistence.hpp"

using runner_nav2_behavior_tree::BlockagePersistence;

TEST(BlockagePersistence, current_and_transient_evidence_retain_path)
{
  BlockagePersistence persistence;
  EXPECT_FALSE(persistence.observe(true, 3));
  EXPECT_FALSE(persistence.observe(false, 3));
  EXPECT_EQ(persistence.blockedObservations(), 1u);
  EXPECT_FALSE(persistence.observe(true, 3));
  EXPECT_EQ(persistence.blockedObservations(), 0u);
}

TEST(BlockagePersistence, persistent_evidence_requests_replan_once_threshold_is_met)
{
  BlockagePersistence persistence;
  EXPECT_FALSE(persistence.observe(false, 3));
  EXPECT_FALSE(persistence.observe(false, 3));
  EXPECT_TRUE(persistence.observe(false, 3));
  EXPECT_EQ(persistence.blockedObservations(), 3u);
}

TEST(BlockagePersistence, reset_allows_a_new_committed_path)
{
  BlockagePersistence persistence;
  EXPECT_TRUE(persistence.observe(false, 1));
  persistence.reset();
  EXPECT_EQ(persistence.blockedObservations(), 0u);
  EXPECT_FALSE(persistence.observe(false, 2));
}
