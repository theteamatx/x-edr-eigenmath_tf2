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

#include "eigenmath_tf2/time_cache.h"

#include <cmath>

#include "absl/time/time.h"
#include "eigenmath/interpolation.h"
#include "eigenmath/matchers.h"
#include "eigenmath/rotation_utils.h"
#include "eigenmath_tf2/transform_storage.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::eigenmath::testing::IsApprox;
using ::testing::Not;

namespace eigenmath::tf2 {
namespace {

class TimeCacheTest : public ::testing::Test {
 public:
  TimeCacheTest() {
    values_.clear();
    for (unsigned int i = 0; i < 1000; i++) {
      int pseudo_rand = std::floor(i * M_PI);
      values_.push_back((pseudo_rand % 100) / 50.0 - 1.0);
    }
  }

  double GetRand() { return values_[step_ = (step_ + 1) % values_.size()]; }

 private:
  std::vector<double> values_;
  int step_ = -1;
};

TEST_F(TimeCacheTest, Repeatability) {
  constexpr int kRuns = 100;

  TimeCache cache;

  TransformStorage stor;
  stor.pose_ = Pose3d();

  for (int i = 1; i < kRuns; i++) {
    stor.parent_frame_id_ = i;
    stor.stamp_ = absl::FromUnixMillis(i);
    EXPECT_TRUE(cache.InsertData(stor));
  }

  for (int i = 1; i < kRuns; i++) {
    auto stor_or_status = cache.GetData(absl::FromUnixMillis(i));
    ASSERT_TRUE(stor_or_status.ok());
    stor = *stor_or_status;
    EXPECT_EQ(stor.parent_frame_id_, i);
    EXPECT_EQ(stor.stamp_, absl::FromUnixMillis(i));
  }

  auto stored_frames = cache.GetAllData();
  EXPECT_EQ(stored_frames.size(), kRuns - 1);
  for (int i = 1; i < kRuns; i++) {
    EXPECT_EQ(stored_frames[i - 1].parent_frame_id_, i);
    EXPECT_EQ(stored_frames[i - 1].stamp_, absl::FromUnixMillis(i));
  }
}

TEST_F(TimeCacheTest, RepeatabilityReverseInsertOrder) {
  constexpr int kRuns = 100;

  TimeCache cache;

  TransformStorage stor;
  stor.pose_ = Pose3d();

  for (int i = kRuns - 1; i >= 0; i--) {
    stor.parent_frame_id_ = i;
    stor.stamp_ = absl::FromUnixMillis(i);

    EXPECT_TRUE(cache.InsertData(stor));
  }
  for (int i = 1; i < kRuns; i++) {
    auto stor_or_status = cache.GetData(absl::FromUnixMillis(i));
    ASSERT_TRUE(stor_or_status.ok());
    stor = *stor_or_status;
    EXPECT_EQ(stor.parent_frame_id_, i);
    EXPECT_EQ(stor.stamp_, absl::FromUnixMillis(i));
  }
}

TEST_F(TimeCacheTest, ZeroAtFront) {
  constexpr int kRuns = 100;

  TimeCache cache;

  TransformStorage stor;
  stor.pose_ = Pose3d();

  for (int i = 1; i < kRuns; i++) {
    stor.parent_frame_id_ = i;
    stor.stamp_ = absl::FromUnixMillis(i);
    EXPECT_TRUE(cache.InsertData(stor));
  }

  stor.parent_frame_id_ = kRuns;
  stor.stamp_ = absl::FromUnixMillis(kRuns);
  EXPECT_TRUE(cache.InsertData(stor));

  for (int i = 1; i < kRuns; i++) {
    auto stor_or_status_0 = cache.GetData(absl::FromUnixMillis(i));
    ASSERT_TRUE(stor_or_status_0.ok());
    stor = *stor_or_status_0;
    EXPECT_EQ(stor.parent_frame_id_, i);
    EXPECT_EQ(stor.stamp_, absl::FromUnixMillis(i));
  }

  auto stor_or_status_1 = cache.GetData(absl::InfiniteFuture());
  ASSERT_TRUE(stor_or_status_1.ok());
  stor = *stor_or_status_1;
  EXPECT_EQ(stor.parent_frame_id_, kRuns);
  EXPECT_EQ(stor.stamp_, absl::FromUnixMillis(kRuns));

  stor.parent_frame_id_ = kRuns;
  stor.stamp_ = absl::FromUnixMillis(kRuns + 1);
  EXPECT_TRUE(cache.InsertData(stor));

  // Make sure we get a different value now that a new values is added
  auto stor_or_status_2 = cache.GetData(absl::InfiniteFuture());
  ASSERT_TRUE(stor_or_status_2.ok());
  stor = *stor_or_status_2;
  EXPECT_EQ(stor.parent_frame_id_, kRuns);
  EXPECT_EQ(stor.stamp_, absl::FromUnixMillis(kRuns + 1));
}

TEST_F(TimeCacheTest, CartesianInterpolation) {
  constexpr int kRuns = 100;
  constexpr double kEpsilon = 2e-6;

  TimeCache cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  int offset = 200;

  TransformStorage stor;
  stor.pose_ = Pose3d();

  for (int i = 1; i < kRuns; i++) {
    for (int step = 0; step < 2; step++) {
      xvalues[step] = 10.0 * GetRand();
      yvalues[step] = 10.0 * GetRand();
      zvalues[step] = 10.0 * GetRand();

      stor.pose_.translation() =
          Vector3d(xvalues[step], yvalues[step], zvalues[step]);
      stor.parent_frame_id_ = 2;
      stor.stamp_ = absl::FromUnixMillis(step * 100 + offset);
      EXPECT_TRUE(cache.InsertData(stor));
    }

    for (int pos = 0; pos < 100; pos++) {
      auto stor_or_status = cache.GetData(absl::FromUnixMillis(offset + pos));
      ASSERT_TRUE(stor_or_status.ok());
      stor = *stor_or_status;
      double x_out = stor.pose_.translation().x();
      double y_out = stor.pose_.translation().y();
      double z_out = stor.pose_.translation().z();
      EXPECT_NEAR(xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos / 100.0,
                  x_out, kEpsilon);
      EXPECT_NEAR(yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos / 100.0,
                  y_out, kEpsilon);
      EXPECT_NEAR(zvalues[0] + (zvalues[1] - zvalues[0]) * (double)pos / 100.0,
                  z_out, kEpsilon);
    }

    cache.Clear();
  }
}

/** \brief Make sure we dont' interpolate across reparented data */
TEST_F(TimeCacheTest, ReparentingInterpolationProtection) {
  constexpr double kEpsilon = 1e-6;
  constexpr int kOffset = 555;

  TimeCache cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  TransformStorage stor;
  stor.pose_ = Pose3d();

  for (int step = 0; step < 2; step++) {
    xvalues[step] = 10.0 * GetRand();
    yvalues[step] = 10.0 * GetRand();
    zvalues[step] = 10.0 * GetRand();

    stor.pose_.translation() =
        Vector3d(xvalues[step], yvalues[step], zvalues[step]);
    stor.parent_frame_id_ = step + 4;
    stor.stamp_ = absl::FromUnixMillis(step * 100 + kOffset);
    EXPECT_TRUE(cache.InsertData(stor));
  }

  for (int pos = 0; pos < 100; pos++) {
    auto stor_or_status = cache.GetData(absl::FromUnixMillis(kOffset + pos));
    ASSERT_TRUE(stor_or_status.ok());
    stor = *stor_or_status;
    double x_out = stor.pose_.translation().x();
    double y_out = stor.pose_.translation().y();
    double z_out = stor.pose_.translation().z();
    EXPECT_NEAR(xvalues[0], x_out, kEpsilon);
    EXPECT_NEAR(yvalues[0], y_out, kEpsilon);
    EXPECT_NEAR(zvalues[0], z_out, kEpsilon);
  }
}

TEST_F(TimeCacheTest, AngularInterpolation) {
  constexpr int kRuns = 100;
  constexpr int kOffset = 200;
  constexpr double kEpsilon = 1e-6;

  TimeCache cache;
  std::vector<Quaterniond> quats(2);

  TransformStorage stor;
  stor.pose_ = Pose3d();

  for (int i = 1; i < kRuns; i++) {
    for (int step = 0; step < 2; step++) {
      const double yawvalue = 10.0 * GetRand() / 100.0;
      quats[step] = QuaternionFromRPY(0.0, 0.0, yawvalue);
      stor.pose_.setQuaternion(quats[step]);
      stor.parent_frame_id_ = 3;
      stor.stamp_ =
          absl::FromUnixMillis(kOffset + (step * 100));  // step = 0 or 1
      EXPECT_TRUE(cache.InsertData(stor));
    }

    for (int pos = 0; pos < 100; pos++) {
      auto stor_or_status = cache.GetData(absl::FromUnixMillis(kOffset + pos));
      ASSERT_TRUE(stor_or_status.ok());
      stor = *stor_or_status;  // get the transform for the position

      // Generate a ground truth quaternion directly calling slerp
      SO3d ground_truth(Interpolate(pos / 100.0, quats[0], quats[1]));

      // Make sure the transformed one and the direct call match
      EXPECT_TRUE(stor.pose_.so3().isApprox(ground_truth, kEpsilon));
    }

    cache.Clear();
  }
}

TEST_F(TimeCacheTest, GetDataOnEmptyCache) {
  TimeCache cache;

  EXPECT_FALSE(cache.HasData(absl::FromUnixMillis(1)));
  EXPECT_EQ(cache.GetData(absl::FromUnixMillis(1)).status().code(),
            absl::StatusCode::kOutOfRange);
  auto ts_and_fr_id = cache.GetTimeIntervalAndParent();
  EXPECT_EQ(ts_and_fr_id.first.oldest, absl::InfiniteFuture());
  EXPECT_EQ(ts_and_fr_id.first.latest, absl::InfiniteFuture());
  EXPECT_EQ(ts_and_fr_id.second, 0);
  EXPECT_EQ(cache.GetLatestTimestamp(), absl::InfinitePast());
  EXPECT_EQ(cache.GetOldestTimestamp(), absl::InfiniteFuture());
}

TEST_F(TimeCacheTest, GetDataOnSingleElementCache) {
  TimeCache cache;

  TransformStorage stor;
  stor.pose_ = Pose3d();
  stor.parent_frame_id_ = 3;
  stor.stamp_ = absl::FromUnixMillis(2);

  EXPECT_TRUE(cache.InsertData(stor));

  EXPECT_FALSE(cache.HasData(absl::FromUnixMillis(3)));
  EXPECT_EQ(cache.GetData(absl::FromUnixMillis(3)).status().code(),
            absl::StatusCode::kOutOfRange);

  EXPECT_FALSE(cache.HasData(absl::FromUnixMillis(1)));
  EXPECT_EQ(cache.GetData(absl::FromUnixMillis(1)).status().code(),
            absl::StatusCode::kOutOfRange);

  EXPECT_TRUE(cache.GetData(absl::FromUnixMillis(2)).ok());
}

TEST_F(TimeCacheTest, GetDataInFutureAndPast) {
  TimeCache cache;

  TransformStorage stor;
  stor.pose_ = Pose3d();
  stor.parent_frame_id_ = 3;
  stor.stamp_ = absl::FromUnixMillis(2);

  EXPECT_TRUE(cache.InsertData(stor));
  stor.stamp_ = absl::FromUnixMillis(3);
  EXPECT_TRUE(cache.InsertData(stor));

  EXPECT_EQ(cache.GetData(absl::FromUnixMillis(4)).status().code(),
            absl::StatusCode::kUnavailable);

  EXPECT_EQ(cache.GetData(absl::FromUnixMillis(1)).status().code(),
            absl::StatusCode::kOutOfRange);

  EXPECT_TRUE(cache.GetData(absl::FromUnixMillis(2)).ok());
}

TEST_F(TimeCacheTest, GetDataWithMaxInterpolationDuration) {
  TimeCache cache;
  TransformStorage stor;
  stor.pose_ = Pose3d();
  stor.parent_frame_id_ = 3;
  stor.child_frame_id_ = 4;
  stor.stamp_ = absl::FromUnixMillis(1000);
  stor.max_interpolation_duration_ = absl::Seconds(0.5);
  EXPECT_TRUE(cache.InsertData(stor));
  stor.stamp_ = absl::FromUnixMillis(1500);
  EXPECT_TRUE(cache.InsertData(stor));
  stor.stamp_ = absl::FromUnixMillis(2010);
  EXPECT_TRUE(cache.InsertData(stor));
  stor.max_interpolation_duration_ = absl::Seconds(0.0);
  stor.stamp_ = absl::FromUnixMillis(2500);
  EXPECT_TRUE(cache.InsertData(stor));

  EXPECT_TRUE(cache.GetData(absl::FromUnixMillis(1200)).ok());
  EXPECT_EQ(cache.GetData(absl::FromUnixMillis(1700)).status().code(),
            absl::StatusCode::kOutOfRange);
  EXPECT_TRUE(cache.GetData(absl::FromUnixMillis(2200)).ok());

  EXPECT_TRUE(cache.GetParent(absl::FromUnixMillis(1200)).ok());
  EXPECT_EQ(cache.GetParent(absl::FromUnixMillis(1700)).status().code(),
            absl::StatusCode::kOutOfRange);
  EXPECT_TRUE(cache.GetParent(absl::FromUnixMillis(2200)).ok());

  EXPECT_TRUE(cache.GetChild(absl::FromUnixMillis(1200)).ok());
  EXPECT_EQ(cache.GetChild(absl::FromUnixMillis(1700)).status().code(),
            absl::StatusCode::kOutOfRange);
  EXPECT_TRUE(cache.GetChild(absl::FromUnixMillis(2200)).ok());
}

TEST_F(TimeCacheTest, DuplicateEntries) {
  TimeCache cache;

  TransformStorage stor;
  stor.pose_ = Pose3d();
  stor.parent_frame_id_ = 3;
  stor.stamp_ = absl::FromUnixMillis(1);

  EXPECT_TRUE(cache.InsertData(stor));
  EXPECT_TRUE(cache.InsertData(stor));

  auto stor_or_status = cache.GetData(absl::FromUnixMillis(1));
  ASSERT_TRUE(stor_or_status.ok());
  stor = *stor_or_status;

  EXPECT_TRUE(!std::isnan(stor.pose_.translation().x()));
  EXPECT_TRUE(!std::isnan(stor.pose_.translation().y()));
  EXPECT_TRUE(!std::isnan(stor.pose_.translation().z()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().x()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().y()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().z()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().w()));
}

TEST_F(TimeCacheTest, NearDuplicateEntries) {
  TimeCache cache;

  TransformStorage stor;
  stor.pose_ = Pose3d();
  stor.parent_frame_id_ = 3;
  stor.stamp_ = absl::FromUnixNanos(1);

  EXPECT_TRUE(cache.InsertData(stor));
  stor.stamp_ = absl::FromUnixNanos(3);
  EXPECT_TRUE(cache.InsertData(stor));

  auto stor_or_status = cache.GetData(absl::FromUnixNanos(2));
  ASSERT_TRUE(stor_or_status.ok());
  stor = *stor_or_status;

  EXPECT_TRUE(!std::isnan(stor.pose_.translation().x()));
  EXPECT_TRUE(!std::isnan(stor.pose_.translation().y()));
  EXPECT_TRUE(!std::isnan(stor.pose_.translation().z()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().x()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().y()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().z()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().w()));
}

TEST(StaticCache, Repeatability) {
  constexpr int kRuns = 100;

  StaticCache cache;

  TransformStorage stor;
  stor.pose_ = Pose3d();

  EXPECT_EQ(cache.GetBufferSize(), 1);
  cache.Clear();
  EXPECT_EQ(cache.GetBufferSize(), 1);

  for (int i = 1; i < kRuns; i++) {
    stor.parent_frame_id_ = CompactFrameID(i);
    stor.stamp_ = absl::FromUnixMillis(i);

    EXPECT_TRUE(cache.InsertData(stor));
    EXPECT_EQ(cache.GetBufferSize(), 1);
    EXPECT_EQ(cache.GetLatestTimestamp(), absl::InfiniteFuture());
    EXPECT_EQ(cache.GetOldestTimestamp(), absl::InfinitePast());

    auto stor_or_status = cache.GetData(absl::FromUnixMillis(i));
    ASSERT_TRUE(stor_or_status.ok());
    stor = *stor_or_status;
    EXPECT_EQ(stor.parent_frame_id_, i);
    EXPECT_EQ(stor.stamp_, absl::FromUnixMillis(i));
  }
}

TEST(StaticCache, DuplicateEntries) {
  StaticCache cache;

  TransformStorage stor;
  stor.pose_ = Pose3d();
  stor.parent_frame_id_ = CompactFrameID(3);
  stor.stamp_ = absl::FromUnixMillis(1);

  EXPECT_TRUE(cache.InsertData(stor));
  EXPECT_TRUE(cache.InsertData(stor));

  auto stor_or_status = cache.GetData(absl::FromUnixMillis(1));
  ASSERT_TRUE(stor_or_status.ok());
  stor = *stor_or_status;

  EXPECT_TRUE(!std::isnan(stor.pose_.translation().x()));
  EXPECT_TRUE(!std::isnan(stor.pose_.translation().y()));
  EXPECT_TRUE(!std::isnan(stor.pose_.translation().z()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().x()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().y()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().z()));
  EXPECT_TRUE(!std::isnan(stor.pose_.quaternion().w()));
}

TEST_F(TimeCacheTest, PruneStorage) {
  TimeCache cache;

  TransformStorage stor;
  stor.pose_ = Pose3d();
  stor.parent_frame_id_ = 3;

  stor.stamp_ = absl::FromUnixMillis(1);
  EXPECT_TRUE(cache.InsertData(stor));
  stor.stamp_ = absl::FromUnixMillis(2);
  EXPECT_TRUE(cache.InsertData(stor));
  stor.stamp_ = TimeCache::kDefaultCacheTime + absl::FromUnixMillis(3);
  EXPECT_TRUE(cache.InsertData(stor));

  // Check pruning of old data:
  EXPECT_EQ(cache.GetData(absl::FromUnixMillis(1)).status().code(),
            absl::StatusCode::kOutOfRange);
  // Check pruning conserved last data point older than cache time:
  EXPECT_TRUE(cache.GetData(absl::FromUnixMillis(2)).ok());

  stor.stamp_ = 2 * TimeCache::kDefaultCacheTime + absl::FromUnixMillis(10);
  EXPECT_TRUE(cache.InsertData(stor));

  // Check pruning conserved last data point older than cache time:
  EXPECT_TRUE(
      cache.GetData(TimeCache::kDefaultCacheTime + absl::FromUnixMillis(3))
          .ok());
}

class InvertWhenInterpolatingTest : public TimeCacheTest,
                                    public ::testing::WithParamInterface<bool> {
};
TEST_P(InvertWhenInterpolatingTest, InvertWhenInterpolating) {
  TransformStorage stor_a;
  stor_a.stamp_ = absl::FromUnixSeconds(100);
  stor_a.parent_frame_id_ = 1;
  stor_a.child_frame_id_ = 0;
  stor_a.max_interpolation_duration_ = absl::InfiniteDuration();
  // Pick transforms where pose_.translation() and the relative orientation
  // is large.
  stor_a.pose_ = Pose3d(Vector3d{1000, 1000, 1000}).inverse();
  TransformStorage stor_b = stor_a;
  stor_b.stamp_ = absl::FromUnixSeconds(101);
  stor_b.pose_ = Pose3d(SO3d{M_PI / 4, M_PI / 4, M_PI / 4},
                        Vector3d{1000.1, 1000.1, 1000.1})
                     .inverse();

  const bool invert_when_interpolating = GetParam();

  TimeCache cache;
  stor_a.invert_when_interpolating_ = stor_b.invert_when_interpolating_ =
      invert_when_interpolating;
  ASSERT_TRUE(cache.InsertData(stor_a));
  ASSERT_TRUE(cache.InsertData(stor_b));

  TimeCache cache_inverted;
  stor_a.parent_frame_id_ = stor_b.parent_frame_id_ = 0;
  stor_a.child_frame_id_ = stor_b.child_frame_id_ = 0;
  stor_a.invert_when_interpolating_ = stor_b.invert_when_interpolating_ = false;
  stor_a.pose_ = stor_a.pose_.inverse();
  stor_b.pose_ = stor_b.pose_.inverse();
  ASSERT_TRUE(cache_inverted.InsertData(stor_a));
  ASSERT_TRUE(cache_inverted.InsertData(stor_b));

  const auto dt = absl::Seconds(0.01);
  // We do not test at stor_a.stamp_ and stor_b.stamp_ since it will be correct
  // regardless of invert_when_interpolating.
  for (absl::Time stamp = stor_a.stamp_ + dt; stamp < stor_b.stamp_;
       stamp = stamp + dt) {
    TransformStorage stor_inverted;
    auto stor_inverted_or_status = cache_inverted.GetData(stamp);
    ASSERT_TRUE(stor_inverted_or_status.ok());
    stor_inverted = *stor_inverted_or_status;
    TransformStorage stor;
    auto stor_or_status = cache.GetData(stamp);
    ASSERT_TRUE(stor_or_status.ok());
    stor = *stor_or_status;
    if (invert_when_interpolating) {
      EXPECT_THAT(stor.pose_, IsApprox(stor_inverted.pose_.inverse()));
    } else {
      EXPECT_THAT(stor.pose_, Not(IsApprox(stor_inverted.pose_.inverse())));
    }
  }
}
INSTANTIATE_TEST_SUITE_P(InvertWhenInterpolatingFlag,
                         InvertWhenInterpolatingTest, ::testing::Bool());

}  // namespace
}  // namespace eigenmath::tf2
