/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "third_party/eigenmath_tf2/buffer_core.h"

#include <cmath>

#include "absl/status/status.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace eigenmath {
namespace tf2 {
namespace {

TEST(BufferCoreTest, SetTransform_FailCases) {
  BufferCore tfc;
  EXPECT_EQ(
      tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(0), "", "", "authority1")
          .code(),
      absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(0), "", "child",
                             "authority1")
                .code(),
            absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(0), "foo", "",
                             "authority1")
                .code(),
            absl::StatusCode::kInvalidArgument);

  Pose3d nan_pose;
  nan_pose.translation().z() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(tfc.SetTransform(nan_pose, absl::FromUnixSeconds(0), "foo", "child",
                             "authority1")
                .code(),
            absl::StatusCode::kInvalidArgument);
}

TEST(BufferCoreTest, SetTransform_Valid) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo",
                               "child", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.CanTransform("foo", "child", absl::FromUnixSeconds(1)).ok());
  EXPECT_TRUE(
      tfc.LookupTransform("foo", "child", absl::FromUnixSeconds(1)).ok());
  EXPECT_TRUE(
      tfc.LookupTransform("child", "foo", absl::FromUnixSeconds(1)).ok());
  absl::Time transform_time;
  EXPECT_TRUE(tfc.LookupTransform("child", "child", absl::InfiniteFuture(),
                                  &transform_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), transform_time);
  EXPECT_TRUE(tfc.LookupTransform("foo", "child", absl::InfiniteFuture(),
                                  &transform_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), transform_time);
  EXPECT_TRUE(tfc.LookupTransform("child", "foo", absl::InfiniteFuture(),
                                  &transform_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), transform_time);

  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(2), "foo",
                               "child", "authority1")
                  .ok());
  absl::Time oldest_time;
  absl::Time latest_time;
  EXPECT_TRUE(
      tfc.GetTransformTimeInterval("foo", "child", &oldest_time, &latest_time)
          .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), oldest_time);
  EXPECT_EQ(absl::FromUnixSeconds(2), latest_time);
}

TEST(BufferCoreTest, RemoveTransformFrameValid) {
  BufferCore tfc;
  ASSERT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo",
                               "child", "authority1")
                  .ok());

  ASSERT_TRUE(tfc.RemoveTransformFrame("child").ok());
  EXPECT_EQ(tfc.CanTransform("foo", "child", absl::FromUnixSeconds(1)).code(),
            absl::StatusCode::kUnavailable);
}

TEST(BufferCoreTest, RemoveTransformFrameNonExisting) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo",
                               "child", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.CanTransform("foo", "child", absl::FromUnixSeconds(1)).ok());
  EXPECT_TRUE(
      tfc.LookupTransform("foo", "child", absl::FromUnixSeconds(1)).ok());
  EXPECT_TRUE(
      tfc.LookupTransform("child", "foo", absl::FromUnixSeconds(1)).ok());

  EXPECT_EQ(tfc.RemoveTransformFrame("grand_child").code(),
            absl::StatusCode::kUnavailable);
}

TEST(BufferCoreTest, RemoveTransformFrameParent) {
  BufferCore tfc;
  ASSERT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo",
                               "child", "authority1")
                  .ok());
  ASSERT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "child",
                               "grand_child", "authority1")
                  .ok());

  EXPECT_EQ(tfc.RemoveTransformFrame("child").code(),
            absl::StatusCode::kInvalidArgument);
}

TEST(BufferCoreTest, RemoveTransformFrameAddRemoveAdd) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo",
                               "child", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "child",
                               "grand_child", "authority1")
                  .ok());
  EXPECT_TRUE(
      tfc.CanTransform("foo", "grand_child", absl::FromUnixSeconds(1)).ok());
  EXPECT_TRUE(
      tfc.LookupTransform("foo", "grand_child", absl::FromUnixSeconds(1)).ok());
  EXPECT_TRUE(
      tfc.LookupTransform("grand_child", "foo", absl::FromUnixSeconds(1)).ok());

  EXPECT_TRUE(tfc.RemoveTransformFrame("grand_child").ok());
  EXPECT_TRUE(tfc.CanTransform("foo", "child", absl::FromUnixSeconds(1)).ok());
  EXPECT_EQ(
      tfc.CanTransform("foo", "grand_child", absl::FromUnixSeconds(1)).code(),
      absl::StatusCode::kUnavailable);

  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "child",
                               "grand_child", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.CanTransform("foo", "child", absl::FromUnixSeconds(1)).ok());
  EXPECT_TRUE(
      tfc.CanTransform("foo", "grand_child", absl::FromUnixSeconds(1)).ok());
}

TEST(BufferCoreTest, LookupTransform_InvalidInterpolation) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixMillis(1000), "foo",
                               "child", "authority1", false,
                               absl::Milliseconds(500))
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixMillis(1500), "foo",
                               "child", "authority1", false,
                               absl::Milliseconds(500))
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixMillis(2010), "foo",
                               "child", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixMillis(2500), "foo",
                               "child", "authority1", false,
                               absl::Milliseconds(0))
                  .ok());

  EXPECT_TRUE(
      tfc.CanTransform("foo", "child", absl::FromUnixMillis(1200)).ok());
  EXPECT_EQ(tfc.CanTransform("foo", "child", absl::FromUnixMillis(1700)).code(),
            absl::StatusCode::kOutOfRange);
  EXPECT_TRUE(
      tfc.CanTransform("foo", "child", absl::FromUnixMillis(2200)).ok());

  EXPECT_TRUE(
      tfc.LookupTransform("foo", "child", absl::FromUnixMillis(1200)).ok());
  EXPECT_EQ(tfc.LookupTransform("child", "foo", absl::FromUnixMillis(1700))
                .status()
                .code(),
            absl::StatusCode::kOutOfRange);
  EXPECT_TRUE(
      tfc.LookupTransform("foo", "child", absl::FromUnixMillis(2200)).ok());
}

TEST(BufferCoreTest, SetTransform_Static) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo",
                               "child", "authority1", true)
                  .ok())
      << tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo", "child",
                          "authority1", true)
             .message();
  EXPECT_TRUE(tfc.CanTransform("foo", "child", absl::FromUnixSeconds(1)).ok());
  EXPECT_TRUE(
      tfc.LookupTransform("foo", "child", absl::FromUnixSeconds(1)).ok());
  EXPECT_TRUE(
      tfc.LookupTransform("child", "foo", absl::FromUnixSeconds(1)).ok());
  absl::Time transform_time;
  EXPECT_TRUE(tfc.LookupTransform("child", "child", absl::FromUnixSeconds(0),
                                  &transform_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(0), transform_time);
  EXPECT_TRUE(tfc.LookupTransform("foo", "child", absl::FromUnixSeconds(0),
                                  &transform_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(0), transform_time);
  EXPECT_TRUE(tfc.LookupTransform("child", "foo", absl::FromUnixSeconds(0),
                                  &transform_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(0), transform_time);
}

TEST(BufferCoreTest, LookupTransform_Nothing_Exists) {
  BufferCore tfc;
  EXPECT_EQ(
      tfc.LookupTransform("a", "b", absl::FromUnixSeconds(1)).status().code(),
      absl::StatusCode::kUnavailable);
}

TEST(BufferCoreTest, CanTransform_Nothing_Exists) {
  BufferCore tfc;
  EXPECT_EQ(tfc.CanTransform("a", "b", absl::FromUnixSeconds(1)).code(),
            absl::StatusCode::kUnavailable);
}

TEST(BufferCoreTest, LookupTransform_One_Exists) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo",
                               "child", "authority1")
                  .ok());
  EXPECT_EQ(tfc.LookupTransform("foo", "bar", absl::FromUnixSeconds(1))
                .status()
                .code(),
            absl::StatusCode::kUnavailable);
}

TEST(BufferCoreTest, CanTransform_One_Exists) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo",
                               "child", "authority1")
                  .ok());
  EXPECT_EQ(tfc.CanTransform("foo", "bar", absl::FromUnixSeconds(1)).code(),
            absl::StatusCode::kUnavailable);
}

TEST(BufferCoreTest, CanTransform_EmptyFrameId) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo",
                               "child", "authority1")
                  .ok());
  EXPECT_EQ(tfc.CanTransform("foo", "", absl::FromUnixSeconds(1)).code(),
            absl::StatusCode::kInvalidArgument);
}

TEST(BufferCoreTest, CanTransform_StartWithSlash) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo",
                               "child", "authority1")
                  .ok());
  EXPECT_EQ(tfc.CanTransform("foo", "/child", absl::FromUnixSeconds(1)).code(),
            absl::StatusCode::kInvalidArgument);
}

TEST(BufferCoreTest, SetTransform_TooOldData) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1000), "foo",
                               "child", "authority1")
                  .ok());
  EXPECT_EQ(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "foo", "child",
                             "authority1")
                .code(),
            absl::StatusCode::kInvalidArgument);
}

TEST(BufferCoreTest, ViaFixedFrame) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "parent",
                               "child", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(2), "parent",
                               "uncle", "authority1", true)
                  .ok());
  EXPECT_TRUE(tfc.CanTransform("child", absl::FromUnixSeconds(1), "uncle",
                               absl::FromUnixSeconds(2), "parent")
                  .ok());
  absl::Time child_time;
  absl::Time uncle_time;
  absl::Time parent_time;
  EXPECT_TRUE(tfc.LookupTransform("child", absl::InfiniteFuture(), "uncle",
                                  absl::FromUnixSeconds(2), "parent",
                                  &child_time, &uncle_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), child_time);
  EXPECT_EQ(absl::FromUnixSeconds(2), uncle_time);
  EXPECT_TRUE(tfc.LookupTransform("child", absl::InfiniteFuture(), "uncle",
                                  absl::FromUnixSeconds(0), "parent",
                                  &child_time, &uncle_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), child_time);
  EXPECT_EQ(absl::FromUnixSeconds(0), uncle_time);
  EXPECT_TRUE(tfc.LookupTransform("parent", absl::InfiniteFuture(), "uncle",
                                  absl::FromUnixSeconds(1), "child",
                                  &parent_time, &uncle_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), parent_time);
  EXPECT_EQ(absl::FromUnixSeconds(1), uncle_time);
  EXPECT_TRUE(tfc.LookupTransform("parent", absl::InfiniteFuture(), "uncle",
                                  absl::InfiniteFuture(), "child", &parent_time,
                                  &uncle_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), parent_time);
  EXPECT_EQ(absl::FromUnixSeconds(1), uncle_time);
}

TEST(BufferCoreTest, DisjointTrees) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "dad",
                               "child", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "mom", "dad",
                               "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "neighbor",
                               "friend", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.CanTransform("mom", "child", absl::InfiniteFuture()).ok());
  EXPECT_EQ(tfc.CanTransform("friend", "child", absl::InfiniteFuture()).code(),
            absl::StatusCode::kInternal);
  absl::Time transform_time;
  EXPECT_TRUE(tfc.LookupTransform("mom", "child", absl::InfiniteFuture(),
                                  &transform_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), transform_time);
  EXPECT_EQ(tfc.LookupTransform("friend", "child", absl::InfiniteFuture(),
                                &transform_time)
                .status()
                .code(),
            absl::StatusCode::kInternal);
}

TEST(BufferCoreTest, LoopDetection) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "dad",
                               "child", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "child",
                               "mom", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "mom", "dad",
                               "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "mom",
                               "aunt", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixSeconds(1), "child",
                               "son_in_law", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.CanTransform("dad", "child", absl::InfiniteFuture()).ok());
  EXPECT_TRUE(
      tfc.CanTransform("dad", "son_in_law", absl::InfiniteFuture()).ok());
  EXPECT_EQ(
      tfc.CanTransform("aunt", "son_in_law", absl::InfiniteFuture()).code(),
      absl::StatusCode::kInternal);
  absl::Time transform_time;
  EXPECT_TRUE(tfc.LookupTransform("dad", "child", absl::InfiniteFuture(),
                                  &transform_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), transform_time);
  EXPECT_TRUE(tfc.LookupTransform("dad", "son_in_law", absl::InfiniteFuture(),
                                  &transform_time)
                  .ok());
  EXPECT_EQ(absl::FromUnixSeconds(1), transform_time);
  EXPECT_EQ(tfc.LookupTransform("aunt", "son_in_law", absl::InfiniteFuture(),
                                &transform_time)
                .status()
                .code(),
            absl::StatusCode::kInternal);

  const std::string yaml_dump = tfc.AllFramesAsYAML(absl::FromUnixSeconds(1));
  EXPECT_EQ(std::count(yaml_dump.begin(), yaml_dump.end(), '\n'), 40)
      << yaml_dump;
  EXPECT_THAT(yaml_dump, ::testing::HasSubstr(R"""(child:
  parent: 'dad'
  broadcaster: 'authority1'
  rate: 10000.000
  most_recent_transform: 1970-01-01T00:00:01+00:00
  oldest_transform: 1970-01-01T00:00:01+00:00
  transform_delay: 0
  buffer_length: 0)"""));
  EXPECT_THAT(yaml_dump, ::testing::HasSubstr(R"""(dad:
  parent: 'mom'
  broadcaster: 'authority1'
  rate: 10000.000
  most_recent_transform: 1970-01-01T00:00:01+00:00
  oldest_transform: 1970-01-01T00:00:01+00:00
  transform_delay: 0
  buffer_length: 0)"""));
  EXPECT_THAT(yaml_dump, ::testing::HasSubstr(R"""(mom:
  parent: 'child'
  broadcaster: 'authority1'
  rate: 10000.000
  most_recent_transform: 1970-01-01T00:00:01+00:00
  oldest_transform: 1970-01-01T00:00:01+00:00
  transform_delay: 0
  buffer_length: 0)"""));
  EXPECT_THAT(yaml_dump, ::testing::HasSubstr(R"""(aunt:
  parent: 'mom'
  broadcaster: 'authority1'
  rate: 10000.000
  most_recent_transform: 1970-01-01T00:00:01+00:00
  oldest_transform: 1970-01-01T00:00:01+00:00
  transform_delay: 0
  buffer_length: 0)"""));
  EXPECT_THAT(yaml_dump, ::testing::HasSubstr(R"""(son_in_law:
  parent: 'child'
  broadcaster: 'authority1'
  rate: 10000.000
  most_recent_transform: 1970-01-01T00:00:01+00:00
  oldest_transform: 1970-01-01T00:00:01+00:00
  transform_delay: 0
  buffer_length: 0)"""));
}

TEST(BufferCoreTest, CollectAllTransforms) {
  BufferCore tfc;
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixMillis(1100), "parent",
                               "child", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixMillis(2100), "parent",
                               "child", "authority1")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixMillis(3100), "parent",
                               "child", "authority4")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixMillis(1200),
                               "grandparent", "parent", "authority2")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixMillis(2200),
                               "grandparent", "parent", "authority2")
                  .ok());
  EXPECT_TRUE(tfc.SetTransform(Pose3d(), absl::FromUnixMillis(1300), "neighbor",
                               "friend", "authority3")
                  .ok());

  std::vector<TransformStorage> all_transforms = tfc.CollectAllTransforms();
  ASSERT_EQ(all_transforms.size(), 6);
  CompactFrameID child_c_id = all_transforms.front().child_frame_id_;
  std::string child_id = tfc.GetFrameId(child_c_id);
  std::string child_authority = tfc.GetFrameAuthority(child_c_id);
  CompactFrameID parent_c_id = all_transforms.front().parent_frame_id_;
  std::string parent_id = tfc.GetFrameId(parent_c_id);
  for (auto& transform : all_transforms) {
    if (transform.child_frame_id_ != child_c_id) {
      child_c_id = transform.child_frame_id_;
      child_id = tfc.GetFrameId(child_c_id);
      child_authority = tfc.GetFrameAuthority(child_c_id);
    }
    if (transform.parent_frame_id_ != parent_c_id) {
      parent_c_id = transform.parent_frame_id_;
      parent_id = tfc.GetFrameId(parent_c_id);
    }
    if (child_id == "child") {
      EXPECT_EQ(parent_id, "parent");
      EXPECT_EQ(child_authority, "authority4");
      EXPECT_EQ(absl::ToUnixMillis(transform.stamp_) % 1000, 100);
    }
    if (child_id == "parent") {
      EXPECT_EQ(parent_id, "grandparent");
      EXPECT_EQ(child_authority, "authority2");
      EXPECT_EQ(absl::ToUnixMillis(transform.stamp_) % 1000, 200);
    }
    if (child_id == "friend") {
      EXPECT_EQ(parent_id, "neighbor");
      EXPECT_EQ(child_authority, "authority3");
      EXPECT_EQ(absl::ToUnixMillis(transform.stamp_) % 1000, 300);
    }
  }
}

}  // namespace
}  // namespace tf2
}  // namespace eigenmath
