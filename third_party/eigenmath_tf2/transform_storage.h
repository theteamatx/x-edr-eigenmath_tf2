/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef EIGENMATH_TF2_EIGENMATH_TF2_TRANSFORM_STORAGE_H_
#define EIGENMATH_TF2_EIGENMATH_TF2_TRANSFORM_STORAGE_H_

#include "absl/time/time.h"
#include "eigenmath/pose3.h"

namespace eigenmath {
namespace tf2 {

using CompactFrameID = uint32_t;
struct TimeInterval {
  absl::Time oldest;
  absl::Time latest;
};
typedef std::pair<TimeInterval, CompactFrameID> TimeIntervalAndFrameID;

// Storage for transforms and their parent
class TransformStorage {
 public:
  TransformStorage() = default;
  TransformStorage(const Pose3d& pose, absl::Time stamp,
                   CompactFrameID parent_frame_id,
                   CompactFrameID child_frame_id,
                   absl::Duration max_interpolation_duration,
                   bool invert_when_interpolating)
      : pose_(pose),
        stamp_(stamp),
        parent_frame_id_(parent_frame_id),
        child_frame_id_(child_frame_id),
        max_interpolation_duration_(max_interpolation_duration),
        invert_when_interpolating_(invert_when_interpolating) {}

  Pose3d pose_;
  absl::Time stamp_;
  CompactFrameID parent_frame_id_;
  CompactFrameID child_frame_id_;
  absl::Duration max_interpolation_duration_ = absl::InfiniteDuration();
  bool invert_when_interpolating_ = false;
};

}  // namespace tf2
}  // namespace eigenmath

#endif  // EIGENMATH_TF2_EIGENMATH_TF2_TRANSFORM_STORAGE_H_
