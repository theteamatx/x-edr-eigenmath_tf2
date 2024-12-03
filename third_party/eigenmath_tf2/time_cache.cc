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

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "eigenmath/interpolation.h"

namespace eigenmath::tf2 {

namespace {

bool IsApproxEqual(absl::Time t0, absl::Time t1, absl::Duration tolerance) {
  return (t0 < t1 + tolerance) && (t0 + tolerance > t1);
}

}  // namespace

TimeCacheInterface::~TimeCacheInterface() = default;

const absl::Duration TimeCache::kMinInterpolationDistance =
    absl::Nanoseconds(5);
const absl::Duration TimeCache::kDefaultCacheTime = absl::Seconds(10);

TimeCache::TimeCache(absl::Duration max_storage_time)
    : storage_(8), max_storage_time_(max_storage_time) {}

absl::Status TimeCache::FindClosest(absl::Time target_time,
                                    TransformStorage const** one,
                                    TransformStorage const** two) const {
  using std::abs;
  *one = nullptr;
  *two = nullptr;

  // No values stored
  if (storage_.empty()) {
    return absl::OutOfRangeError("Lookup cannot work on an empty buffer");
  }

  // If time == 0 return the latest
  if (target_time == absl::InfiniteFuture()) {
    *one = &storage_.front();
    return absl::OkStatus();
  }

  // One value stored
  if (storage_.size() == 1) {
    if (IsApproxEqual(storage_.front().stamp_, target_time,
                      kMinInterpolationDistance)) {
      *one = &storage_.front();
      return absl::OkStatus();
    } else {
      return absl::OutOfRangeError(absl::StrCat(
          "Lookup would require extrapolation at time ",
          absl::FormatTime(target_time), ", but only time ",
          absl::FormatTime(storage_.front().stamp_), " is in the buffer"));
    }
  }

  const absl::Time latest_time = storage_.front().stamp_;
  const absl::Time earliest_time = storage_.back().stamp_;

  if (IsApproxEqual(target_time, latest_time, kMinInterpolationDistance)) {
    *one = &storage_.front();
    return absl::OkStatus();
  } else if (IsApproxEqual(target_time, earliest_time,
                           kMinInterpolationDistance)) {
    *one = &storage_.back();
    return absl::OkStatus();
  } else if (target_time > latest_time) {
    return absl::UnavailableError(absl::StrCat(
        "Lookup would require extrapolation into the future. Requested time ",
        absl::FormatTime(target_time), " but latest time ",
        absl::FormatTime(latest_time)));
  } else if (target_time < earliest_time) {
    return absl::OutOfRangeError(absl::StrCat(
        "Lookup would require extrapolation into the past. Requested time ",
        absl::FormatTime(target_time), " but earliest time ",
        absl::FormatTime(earliest_time)));
  }

  // At least 2 values stored
  // Find the first value less than the target value
  auto storage_it = std::lower_bound(
      storage_.begin(), storage_.end(), target_time,
      [](const TransformStorage& stored_val, const absl::Time& target_time) {
        return stored_val.stamp_ > target_time;
      });

  // Finally the case were somewhere in the middle
  *one = &*(storage_it);    // Older
  *two = &*(--storage_it);  // Newer

  const absl::Duration interpolation_duration = (*two)->stamp_ - (*one)->stamp_;
  // Test if the duration between the two candidate transforms is within the
  // valid maximum interpolation duration bound.
  if (interpolation_duration > (*one)->max_interpolation_duration_) {
    absl::Status status = absl::OutOfRangeError(absl::StrCat(
        "Lookup would require interpolation between transforms that are ",
        absl::FormatDuration(interpolation_duration),
        "s apart, but the maximum allowed interpolation duration is ",
        absl::FormatDuration((*one)->max_interpolation_duration_), "s."));
    *one = nullptr;
    *two = nullptr;
    return status;
  }

  return absl::OkStatus();
}

void TimeCache::Interpolate(const TransformStorage& one,
                            const TransformStorage& two, absl::Time time,
                            TransformStorage* output) const {
  // Check for zero distance case
  if (IsApproxEqual(two.stamp_, one.stamp_, kMinInterpolationDistance)) {
    *output = two;
    return;
  }
  // Calculate the ratio
  const double ratio = absl::ToDoubleSeconds(time - one.stamp_) /
                       absl::ToDoubleSeconds(two.stamp_ - one.stamp_);

  // Interpolate pose (uses slerp for rotation)
  if (one.invert_when_interpolating_) {
    output->pose_ = ::eigenmath::Interpolate(ratio, one.pose_.inverse(),
                                             two.pose_.inverse())
                        .inverse();
  } else {
    output->pose_ = ::eigenmath::Interpolate(ratio, one.pose_, two.pose_);
  }

  output->stamp_ = one.stamp_;
  output->parent_frame_id_ = one.parent_frame_id_;
  output->child_frame_id_ = one.child_frame_id_;
}

bool TimeCache::HasData(absl::Time time) const {
  if (storage_.empty()) {
    return false;
  }
  if (time == absl::InfiniteFuture()) {
    return true;
  }
  const absl::Time latest_time = storage_.front().stamp_;
  const absl::Time earliest_time = storage_.back().stamp_;
  if ((time > latest_time + kMinInterpolationDistance) ||
      (time + kMinInterpolationDistance < earliest_time)) {
    return false;
  }
  return true;
}

absl::StatusOr<TransformStorage> TimeCache::GetData(absl::Time time) const {
  TransformStorage const* p_temp_1;
  TransformStorage const* p_temp_2;
  absl::Status status = FindClosest(time, &p_temp_1, &p_temp_2);
  if (!status.ok()) {
    return status;
  }
  TransformStorage data_out;
  if (p_temp_2 == nullptr) {
    data_out = *p_temp_1;
  } else {
    if (p_temp_1->parent_frame_id_ == p_temp_2->parent_frame_id_) {
      if (p_temp_1->invert_when_interpolating_ !=
          p_temp_2->invert_when_interpolating_) {
        return absl::FailedPreconditionError(absl::StrCat(
            "Cannot interpolate between transforms with different "
            "value of invert_when_interpolating:\n",
            p_temp_1->parent_frame_id_, "\n", p_temp_1->child_frame_id_));
      }
      Interpolate(*p_temp_1, *p_temp_2, time, &data_out);
    } else {
      data_out = *p_temp_1;
    }
  }

  return data_out;
}

absl::StatusOr<CompactFrameID> TimeCache::GetParent(absl::Time time) const {
  TransformStorage const* p_temp_1;
  TransformStorage const* p_temp_2;
  absl::Status status = FindClosest(time, &p_temp_1, &p_temp_2);
  if (!status.ok()) {
    return status;
  }
  return p_temp_1->parent_frame_id_;
}

absl::StatusOr<CompactFrameID> TimeCache::GetChild(absl::Time time) const {
  TransformStorage const* p_temp_1;
  TransformStorage const* p_temp_2;
  absl::Status status = FindClosest(time, &p_temp_1, &p_temp_2);
  if (!status.ok()) {
    return status;
  }
  return p_temp_1->child_frame_id_;
}

bool TimeCache::InsertData(const TransformStorage& new_data) {
  // Grow capacity by a factor of 2 if we ran out:
  if (storage_.size() == storage_.capacity()) {
    storage_.ChangeCapacity(storage_.capacity() * 2);
  }

  if (storage_.empty()) {
    storage_.push_front(new_data);
    return true;
  }
  const auto storage_it = storage_.begin();

  if ((storage_it != storage_.end()) &&
      (storage_it->stamp_ > new_data.stamp_ + max_storage_time_)) {
    return false;
  }

  // This is a workaround the lack of insert() function in CircularBuffer,
  // it may look inefficient, but it is optimal when new data is mostly
  // coming in in order (rarely need to do much data rotation).
  storage_.push_front(new_data);
  const auto insert_pos = std::lower_bound(
      std::next(storage_.begin()), storage_.end(), new_data.stamp_,
      [](const TransformStorage& stored_val, const absl::Time& new_time) {
        return stored_val.stamp_ > new_time;
      });
  // rotate puts first element (new_data) just before insert_pos and moves
  // everything in-between one position to the left.
  std::rotate(storage_.begin(), std::next(storage_.begin()), insert_pos);

  PruneStorage();
  return true;
}

std::vector<TransformStorage> TimeCache::GetAllData() const {
  return {storage_.rbegin(), storage_.rend()};
}

void TimeCache::Clear() { storage_.clear(); }

int TimeCache::GetBufferSize() const { return storage_.size(); }

TimeIntervalAndFrameID TimeCache::GetTimeIntervalAndParent() const {
  if (storage_.empty()) {
    return std::make_pair(
        TimeInterval{absl::InfiniteFuture(), absl::InfiniteFuture()}, 0);
  }

  const auto& ts = storage_.front();
  return std::make_pair(TimeInterval{storage_.back().stamp_, ts.stamp_},
                        ts.parent_frame_id_);
}

absl::Time TimeCache::GetLatestTimestamp() const {
  if (storage_.empty()) {
    return absl::InfinitePast();  // empty list case
  }
  return storage_.front().stamp_;
}

absl::Time TimeCache::GetOldestTimestamp() const {
  if (storage_.empty()) {
    return absl::InfiniteFuture();  // empty list case
  }
  return storage_.back().stamp_;
}

void TimeCache::PruneStorage() {
  const absl::Time latest_time = storage_.front().stamp_;

  while ((storage_.size() > 2) &&
         (std::next(storage_.rbegin())->stamp_ + max_storage_time_ <
          latest_time)) {
    storage_.pop_back();
  }
}

bool StaticCache::HasData(absl::Time time) const { return true; }

absl::StatusOr<TransformStorage> StaticCache::GetData(absl::Time time) const {
  TransformStorage data_out = storage_;
  data_out.stamp_ = time;
  return data_out;
}

bool StaticCache::InsertData(const TransformStorage& new_data) {
  storage_ = new_data;
  return true;
}

std::vector<TransformStorage> StaticCache::GetAllData() const {
  return {storage_};
}

void StaticCache::Clear() {}

int StaticCache::GetBufferSize() const { return 1; }

absl::StatusOr<CompactFrameID> StaticCache::GetParent(absl::Time time) const {
  return storage_.parent_frame_id_;
}

absl::StatusOr<CompactFrameID> StaticCache::GetChild(absl::Time time) const {
  return storage_.child_frame_id_;
}

TimeIntervalAndFrameID StaticCache::GetTimeIntervalAndParent() const {
  return std::make_pair(
      TimeInterval{absl::InfinitePast(), absl::InfiniteFuture()},
      storage_.parent_frame_id_);
}

absl::Time StaticCache::GetLatestTimestamp() const {
  return absl::InfiniteFuture();
}

absl::Time StaticCache::GetOldestTimestamp() const {
  return absl::InfinitePast();
}

}  // namespace eigenmath::tf2
