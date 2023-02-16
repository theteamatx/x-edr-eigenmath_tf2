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

#include "third_party/eigenmath_tf2/buffer_core.h"

#include <sstream>

#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "eigenmath/pose3.h"
#include "eigenmath/pose3_utils.h"
#include "third_party/eigenmath_tf2/time_cache.h"
#include "third_party/eigenmath_tf2/transform_storage.h"

namespace eigenmath {
namespace tf2 {

namespace {

bool StartsWithSlash(std::string_view frame_id) {
  if (!frame_id.empty()) {
    if (frame_id[0] == '/') {
      return true;
    }
  }
  return false;
}

void StripSlash(std::string_view* in) {
  if (StartsWithSlash(*in)) {
    in->remove_prefix(1);
  }
}

}  // namespace

#define EIGENMATH_TF2_RETURN_IF_ERROR(expr, msg)                               \
  {                                                                            \
    absl::Status status = expr;                                                \
    if (!status.ok()) {                                                        \
      return absl::Status(status.code(), absl::StrCat(status.message(), msg)); \
    }                                                                          \
  }

#define EIGENMATH_TF2_ASSIGN_OR_RETURN_IMPL(statusor, lhs, expr, msg)    \
  auto statusor = (expr);                                                \
  if (!statusor.ok()) {                                                  \
    return absl::Status(statusor.status().code(),                        \
                        absl::StrCat(statusor.status().message(), msg)); \
  }                                                                      \
  lhs = (*std::move(statusor))

#define EIGENMATH_TF2_ASSIGN_OR_RETURN_STR_CONCAT_INNER(x, y) x##y
#define EIGENMATH_TF2_ASSIGN_OR_RETURN_STR_CONCAT(x, y) \
  EIGENMATH_TF2_ASSIGN_OR_RETURN_STR_CONCAT_INNER(x, y)

#define EIGENMATH_TF2_ASSIGN_OR_RETURN(lhs, expr, msg)                      \
  EIGENMATH_TF2_ASSIGN_OR_RETURN_IMPL(                                      \
      EIGENMATH_TF2_ASSIGN_OR_RETURN_STR_CONCAT(status_or_value, __LINE__), \
      lhs, expr, msg)

const absl::Duration BufferCore::kDefaultCacheTime = absl::Seconds(10);

absl::Status BufferCore::CheckFrameId(std::string_view frame_id) const {
  if (frame_id.empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid argument \"", frame_id,
                     "\", in tf2 frame_ids cannot be empty"));
  }

  if (StartsWithSlash(frame_id)) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid argument \"", frame_id,
                     "\", in tf2 frame_ids cannot start with a '/' character"));
  }

  return absl::OkStatus();
}

absl::StatusOr<CompactFrameID> BufferCore::ValidatedFrameId(
    std::string_view frame_id) const {
  EIGENMATH_TF2_RETURN_IF_ERROR(CheckFrameId(frame_id), "");

  const CompactFrameID id = LookupFrameNumber(frame_id);
  if (id == kRootFrameId) {
    return absl::UnavailableError(
        absl::StrCat("Frame-id \"", frame_id, "\" does not exist."));
  }

  return id;
}

BufferCore::BufferCore(absl::Duration cache_time) : cache_time_(cache_time) {
  frame_ids_["NO_PARENT"] = kRootFrameId;
  frames_[kRootFrameId] = {};
  frame_authority_.emplace(kRootFrameId, "");
  frame_ids_reverse_.emplace(kRootFrameId, "NO_PARENT");
}

void BufferCore::Clear() {
  absl::WriterMutexLock lock(&frame_mutex_);
  if (frames_.size() < 2) {
    return;
  }
  for (auto& [frame_id, cache] : frames_) {
    if (frame_id != kRootFrameId && cache != nullptr) {
      cache->Clear();
    }
  }
}

absl::Status BufferCore::SetTransform(
    const Pose3d& pose, absl::Time stamp, std::string_view parent_frame_id,
    std::string_view child_frame_id, std::string_view authority, bool is_static,
    absl::Duration max_interpolation_duration, bool invert_when_interpolating) {
  StripSlash(&parent_frame_id);
  StripSlash(&child_frame_id);

  if (child_frame_id == parent_frame_id) {
    return absl::InvalidArgumentError(
        absl::StrCat("Ignoring transform from authority \"", authority,
                     "\" with parent_frame_id and child_frame_id \"",
                     child_frame_id, "\" because they are the same"));
  }

  if (child_frame_id.empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Ignoring transform from authority \"", authority,
                     "\" with parent_frame_id \"", parent_frame_id,
                     "\" because child_frame_id not set"));
  }

  if (parent_frame_id.empty()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Ignoring transform from authority \"", authority,
                     "\" with child_frame_id \"", child_frame_id,
                     "\" because parent_frame_id not set"));
  }

  if (std::isnan(pose.translation().x()) ||
      std::isnan(pose.translation().y()) ||
      std::isnan(pose.translation().z()) || std::isnan(pose.quaternion().x()) ||
      std::isnan(pose.quaternion().y()) || std::isnan(pose.quaternion().z()) ||
      std::isnan(pose.quaternion().w())) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Ignoring transform from authority \"", authority,
        "\" with child_frame_id \"", child_frame_id,
        "\" and parent_frame_id \"", parent_frame_id,
        "\" because of a nan value in the transform (", pose.translation().x(),
        ", ", pose.translation().y(), ", ", pose.translation().z(), ") (",
        pose.quaternion().x(), ", ", pose.quaternion().y(), ", ",
        pose.quaternion().z(), ", ", pose.quaternion().w(), ")"));
  }

  CompactFrameID child_frame_num, parent_frame_num;
  bool new_child_frame, new_parent_frame;
  {
    absl::WriterMutexLock lock(&frame_mutex_);
    child_frame_num =
        LookupOrInsertFrameNumber(child_frame_id, &new_child_frame);
    parent_frame_num =
        LookupOrInsertFrameNumber(parent_frame_id, &new_parent_frame);
    TimeCacheInterface* frame = GetOrAllocateFrame(child_frame_num, is_static);

    const auto time_frame_id_pair = frame->GetTimeIntervalAndParent();
    if (time_frame_id_pair.first.latest != absl::InfiniteFuture() &&
        time_frame_id_pair.second != parent_frame_num) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Given parent_frame_id \"", parent_frame_id,
          "\" with child_frame_id \"", child_frame_id,
          "\" does not match the previously used parent_frame_id \"",
          *LookupFrameString(time_frame_id_pair.second),
          "\". A given child frame can always only have one unique parent "
          "frame to preserve the tree structure!"));
    }

    if (frame->InsertData(TransformStorage(
            pose, stamp, parent_frame_num, child_frame_num,
            max_interpolation_duration, invert_when_interpolating))) {
      frame_authority_[child_frame_num].assign(authority.begin(),
                                               authority.end());
    } else {
      return absl::InvalidArgumentError(absl::StrCat(
          "Ignoring transform from authority \"", authority,
          "\" with child_frame_id \"", child_frame_id,
          "\" and parent_frame_id \"", parent_frame_id,
          "\" because it is too old with time ", absl::FormatTime(stamp)));
    }
  }

  return absl::OkStatus();
}

TimeCacheInterface* BufferCore::GetOrAllocateFrame(CompactFrameID cfid,
                                                   bool is_static) {
  auto& frame = frames_[cfid];
  if (frame != nullptr) {
    return frame.get();
  }
  if (is_static) {
    frame = std::make_unique<StaticCache>();
  } else {
    frame = std::make_unique<TimeCache>(cache_time_);
  }
  return frame.get();
}

enum WalkEnding {
  Identity,
  TargetParentOfSource,
  SourceParentOfTarget,
  FullPath,
};

template <typename F>
absl::Status BufferCore::WalkToTopParent(F* f, absl::Time time,
                                         CompactFrameID target_id,
                                         CompactFrameID source_id) const {
  // Short circuit if zero length transform to allow lookups on non existent
  // links
  if (source_id == target_id) {
    f->Finalize(Identity, time);
    return absl::OkStatus();
  }

  // If getting the latest get the latest common time
  if (time == absl::InfiniteFuture()) {
    absl::Time ignore_oldest;
    EIGENMATH_TF2_RETURN_IF_ERROR(
        GetCommonTimeInterval(target_id, source_id, &ignore_oldest, &time), "");
  }

  // Walk the tree to its root from the source frame, accumulating transforms
  CompactFrameID frame = source_id;
  CompactFrameID top_parent = frame;
  int depth = 0;

  absl::Status extrapolation_might_have_occurred_status;
  while (frame != kRootFrameId) {
    const TimeCacheInterface* cache = GetFrame(frame);

    if (!cache) {
      // There will be no cache for the very root of the tree
      top_parent = frame;
      break;
    }

    absl::StatusOr<CompactFrameID> status_or_parent = f->Gather(cache, time);
    if (!status_or_parent.ok()) {
      // Just break out here... there may still be a path from source -> target
      top_parent = frame;
      extrapolation_might_have_occurred_status = status_or_parent.status();
      break;
    }

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id) {
      f->Finalize(TargetParentOfSource, time);
      return absl::OkStatus();
    }

    f->Accum(true);

    top_parent = frame;
    frame = *status_or_parent;

    ++depth;
    if (depth > kMaxGraphDepth) {
      return absl::InternalError(
          absl::StrCat("The tf tree is invalid because it contains a loop.\n",
                       AllFramesAsStringNoLock(), "\n"));
    }
  }

  // Now walk to the top parent from the target frame, accumulating transforms
  frame = target_id;
  depth = 0;

  while (frame != top_parent) {
    const TimeCacheInterface* cache = GetFrame(frame);

    if (!cache) {
      break;
    }

    EIGENMATH_TF2_ASSIGN_OR_RETURN(
        CompactFrameID parent, f->Gather(cache, time),
        absl::StrCat("while looking up transform between frames '",
                     *LookupFrameString(target_id), "' and '",
                     *LookupFrameString(source_id), "'"));

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id) {
      f->Finalize(SourceParentOfTarget, time);
      return absl::OkStatus();
    }

    f->Accum(false);

    frame = parent;

    ++depth;
    if (depth > kMaxGraphDepth) {
      return absl::InternalError(
          absl::StrCat("The tf tree is invalid because it contains a loop.\n",
                       AllFramesAsStringNoLock(), "\n"));
    }
  }

  if (frame != top_parent) {
    EIGENMATH_TF2_RETURN_IF_ERROR(
        extrapolation_might_have_occurred_status,
        absl::StrCat("while looking up transform between frames '",
                     *LookupFrameString(target_id), "' and '",
                     *LookupFrameString(source_id), "'"));

    return absl::InternalError(absl::StrCat(
        "Could not find a connection between '", *LookupFrameString(target_id),
        "' and '", *LookupFrameString(source_id),
        "' because they are not part of the same tree. Tf has two "
        "or more unconnected trees."));
  }

  f->Finalize(FullPath, time);

  return absl::OkStatus();
}

struct TransformAccum {
  TransformAccum() : source_to_top(), target_to_top(), result() {}

  absl::StatusOr<CompactFrameID> Gather(const TimeCacheInterface* cache,
                                        absl::Time time) {
    EIGENMATH_TF2_ASSIGN_OR_RETURN(st, cache->GetData(time), "");
    return st.parent_frame_id_;
  }

  void Accum(bool source) {
    if (source) {
      source_to_top = st.pose_ * source_to_top;
    } else {
      target_to_top = st.pose_ * target_to_top;
    }
  }

  void Finalize(WalkEnding end, absl::Time _time) {
    switch (end) {
      case Identity:
        break;
      case TargetParentOfSource:
        result = source_to_top;
        break;
      case SourceParentOfTarget:
        result = target_to_top.inverse();
        break;
      case FullPath:
        result = target_to_top.inverse() * source_to_top;
        break;
    }

    time = _time;
  }

  TransformStorage st;
  absl::Time time;
  Pose3d source_to_top;
  Pose3d target_to_top;

  Pose3d result;
};

absl::StatusOr<Pose3d> BufferCore::LookupTransform(
    std::string_view target_frame, std::string_view source_frame,
    absl::Time query_time, absl::Time* transform_time) const {
  absl::ReaderMutexLock lock(&frame_mutex_);
  return LookupTransformNoLock(target_frame, source_frame, query_time,
                               transform_time);
}

absl::StatusOr<Pose3d> BufferCore::LookupTransform(
    std::string_view target_frame, const absl::Time& target_time,
    std::string_view source_frame, const absl::Time& source_time,
    std::string_view fixed_frame, absl::Time* target_transform_time,
    absl::Time* source_transform_time) const {
  absl::ReaderMutexLock lock(&frame_mutex_);

  EIGENMATH_TF2_RETURN_IF_ERROR(
      ValidatedFrameId(target_frame).status(),
      absl::StrCat("LookupTransform: Invalid target frame-id ('", target_frame,
                   "')"));
  EIGENMATH_TF2_RETURN_IF_ERROR(
      ValidatedFrameId(source_frame).status(),
      absl::StrCat("LookupTransform: Invalid source frame-id ('", source_frame,
                   "')"));
  EIGENMATH_TF2_RETURN_IF_ERROR(
      ValidatedFrameId(fixed_frame).status(),
      absl::StrCat("LookupTransform: Invalid fixed frame-id ('", fixed_frame,
                   "')"));

  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      Pose3d temp1,
      LookupTransformNoLock(fixed_frame, source_frame, source_time,
                            source_transform_time),
      absl::StrCat("LookupTransform: Could not find transform fixed ('",
                   fixed_frame, "') -> source ('", source_frame, "') at time ",
                   absl::FormatTime(source_time)));
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      Pose3d temp2,
      LookupTransformNoLock(target_frame, fixed_frame, target_time,
                            target_transform_time),
      absl::StrCat("LookupTransform: Could not find transform target ('",
                   target_frame, "') -> fixed ('", fixed_frame, "') at time ",
                   absl::FormatTime(target_time)));

  return Pose3d(temp2 * temp1);
}

absl::StatusOr<Pose2d> BufferCore::LookupTransform2d(
    std::string_view target_frame, std::string_view source_frame,
    absl::Time query_time, absl::string_view horizontal_frame,
    absl::Time* transform_time) const {
  absl::ReaderMutexLock lock(&frame_mutex_);

  EIGENMATH_TF2_RETURN_IF_ERROR(
      ValidatedFrameId(target_frame).status(),
      absl::StrCat("LookupTransform2d: Invalid target frame-id ('",
                   target_frame, "')"));
  EIGENMATH_TF2_RETURN_IF_ERROR(
      ValidatedFrameId(source_frame).status(),
      absl::StrCat("LookupTransform2d: Invalid source frame-id ('",
                   source_frame, "')"));
  EIGENMATH_TF2_RETURN_IF_ERROR(
      ValidatedFrameId(horizontal_frame).status(),
      absl::StrCat("LookupTransform2d: Invalid horizontal frame-id ('",
                   horizontal_frame, "')"));

  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      Pose3d source_pose3_horiz,
      LookupTransformNoLock(source_frame, horizontal_frame, query_time,
                            transform_time),
      absl::StrCat("LookupTransform2d: Could not find transform horizontal ('",
                   horizontal_frame, "') -> source ('", source_frame,
                   "') at time ", absl::FormatTime(query_time)));
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      Pose3d target_pose3_horiz,
      LookupTransformNoLock(target_frame, horizontal_frame, query_time,
                            transform_time),
      absl::StrCat("LookupTransform2d: Could not find transform target ('",
                   target_frame, "') -> horizontal ('", horizontal_frame,
                   "') at time ", absl::FormatTime(query_time)));

  const Pose2d source_pose2_horiz = ToPose2XY(source_pose3_horiz);
  const Pose2d target_pose2_horiz = ToPose2XY(target_pose3_horiz);

  return target_pose2_horiz * source_pose2_horiz.inverse();
}

absl::StatusOr<Pose2d> BufferCore::LookupTransform2d(
    std::string_view target_frame, const absl::Time& target_time,
    std::string_view source_frame, const absl::Time& source_time,
    std::string_view fixed_frame, absl::string_view horizontal_frame,
    absl::Time* target_transform_time,
    absl::Time* source_transform_time) const {
  absl::ReaderMutexLock lock(&frame_mutex_);

  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      CompactFrameID target_id, ValidatedFrameId(target_frame),
      absl::StrCat("LookupTransform2d: Invalid target frame-id ('",
                   target_frame, "')"));
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      CompactFrameID source_id, ValidatedFrameId(source_frame),
      absl::StrCat("LookupTransform2d: Invalid source frame-id ('",
                   source_frame, "')"));
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      CompactFrameID fixed_id, ValidatedFrameId(fixed_frame),
      absl::StrCat("LookupTransform2d: Invalid fixed frame-id ('", fixed_frame,
                   "')"));
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      CompactFrameID horizontal_id, ValidatedFrameId(horizontal_frame),
      absl::StrCat("LookupTransform2d: Invalid horizontal frame-id ('",
                   horizontal_frame, "')"));

  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      int horiz_count_in_source_path,
      CountFrameNoLock(fixed_id, source_id, horizontal_id, source_time), "");
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      int horiz_count_in_target_path,
      CountFrameNoLock(target_id, fixed_id, horizontal_id, target_time), "");

  // Special cases for when the horizontal frame can be found between one leg
  // or the other of the query.
  if (horiz_count_in_source_path == 1) {
    absl::Time actual_source_time = absl::InfinitePast();
    EIGENMATH_TF2_ASSIGN_OR_RETURN(
        Pose3d source_pose3_horiz,
        LookupTransformNoLock(source_frame, horizontal_frame, source_time,
                              &actual_source_time),
        absl::StrCat("LookupTransform2d: Could not find transform source ('",
                     source_frame, "') -> horizontal ('", horizontal_frame,
                     "') at time ", absl::FormatTime(source_time)));
    EIGENMATH_TF2_ASSIGN_OR_RETURN(
        Pose3d fixed_pose3_horiz,
        LookupTransformNoLock(fixed_frame, horizontal_frame, actual_source_time,
                              source_transform_time),
        absl::StrCat("LookupTransform2d: Could not find transform fixed ('",
                     fixed_frame, "') -> horizontal ('", horizontal_frame,
                     "') at time ", absl::FormatTime(actual_source_time)));
    EIGENMATH_TF2_ASSIGN_OR_RETURN(
        Pose3d target_pose3_fixed,
        LookupTransformNoLock(target_frame, fixed_frame, target_time,
                              target_transform_time),
        absl::StrCat("LookupTransform2d: Could not find transform target ('",
                     target_frame, "') -> fixed ('", fixed_frame, "') at time ",
                     absl::FormatTime(target_time)));

    const Pose2d source_pose2_horiz = ToPose2XY(source_pose3_horiz);
    const Pose2d target_pose2_horiz =
        ToPose2XY(target_pose3_fixed * fixed_pose3_horiz);

    return target_pose2_horiz * source_pose2_horiz.inverse();
  } else if (horiz_count_in_target_path == 1) {
    absl::Time actual_target_time = absl::InfinitePast();
    EIGENMATH_TF2_ASSIGN_OR_RETURN(
        Pose3d target_pose3_horiz,
        LookupTransformNoLock(target_frame, horizontal_frame, target_time,
                              &actual_target_time),
        absl::StrCat("LookupTransform2d: Could not find transform target ('",
                     target_frame, "') -> horizontal ('", horizontal_frame,
                     "') at time ", absl::FormatTime(target_time)));
    EIGENMATH_TF2_ASSIGN_OR_RETURN(
        Pose3d fixed_pose3_horiz,
        LookupTransformNoLock(fixed_frame, horizontal_frame, actual_target_time,
                              target_transform_time),
        absl::StrCat("LookupTransform2d: Could not find transform fixed ('",
                     fixed_frame, "') -> horizontal ('", horizontal_frame,
                     "') at time ", absl::FormatTime(actual_target_time)));
    EIGENMATH_TF2_ASSIGN_OR_RETURN(
        Pose3d source_pose3_fixed,
        LookupTransformNoLock(source_frame, fixed_frame, source_time,
                              source_transform_time),
        absl::StrCat("LookupTransform2d: Could not find transform source ('",
                     source_frame, "') -> fixed ('", fixed_frame, "') at time ",
                     absl::FormatTime(source_time)));

    const Pose2d source_pose2_horiz =
        ToPose2XY(source_pose3_fixed * fixed_pose3_horiz);
    const Pose2d target_pose2_horiz = ToPose2XY(target_pose3_horiz);

    return target_pose2_horiz * source_pose2_horiz.inverse();
  }

  absl::Time actual_source_time = absl::InfinitePast();
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      Pose3d source_pose3_horiz,
      LookupTransformNoLock(source_frame, horizontal_frame, source_time,
                            &actual_source_time),
      absl::StrCat("LookupTransform2d: Could not find transform source ('",
                   source_frame, "') -> horizontal ('", horizontal_frame,
                   "') at time ", absl::FormatTime(source_time)));
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      Pose3d fixed_pose3_horiz_at_source,
      LookupTransformNoLock(fixed_frame, horizontal_frame, actual_source_time,
                            source_transform_time),
      absl::StrCat("LookupTransform2d: Could not find transform fixed ('",
                   fixed_frame, "') -> horizontal ('", horizontal_frame,
                   "') at time ", absl::FormatTime(actual_source_time)));

  absl::Time actual_target_time = absl::InfinitePast();
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      Pose3d target_pose3_horiz,
      LookupTransformNoLock(target_frame, horizontal_frame, target_time,
                            &actual_target_time),
      absl::StrCat("LookupTransform2d: Could not find transform target ('",
                   target_frame, "') -> horizontal ('", horizontal_frame,
                   "') at time ", absl::FormatTime(target_time)));
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      Pose3d fixed_pose3_horiz_at_target,
      LookupTransformNoLock(fixed_frame, horizontal_frame, actual_target_time,
                            target_transform_time),
      absl::StrCat("LookupTransform2d: Could not find transform fixed ('",
                   fixed_frame, "') -> horizontal ('", horizontal_frame,
                   "') at time ", absl::FormatTime(actual_target_time)));

  const Pose2d source_pose2_horiz = ToPose2XY(source_pose3_horiz);
  const Pose2d horiz_at_target_pose2_horiz_at_source = ToPose2XY(
      fixed_pose3_horiz_at_target.inverse() * fixed_pose3_horiz_at_source);
  const Pose2d target_pose2_horiz = ToPose2XY(target_pose3_horiz);

  return target_pose2_horiz * horiz_at_target_pose2_horiz_at_source *
         source_pose2_horiz.inverse();
}

absl::StatusOr<Pose3d> BufferCore::LookupTransformNoLock(
    std::string_view target_frame, std::string_view source_frame,
    absl::Time query_time, absl::Time* transform_time) const {
  if (target_frame == source_frame) {
    if (transform_time) {
      if (query_time == absl::InfiniteFuture()) {
        const CompactFrameID target_id = LookupFrameNumber(target_frame);
        const TimeCacheInterface* cache = GetFrame(target_id);
        if (cache) {
          *transform_time = cache->GetLatestTimestamp();
        } else {
          *transform_time = query_time;
        }
      } else {
        *transform_time = query_time;
      }
    }

    // Return identity transform
    return Pose3d();
  }

  // Identity case does not need to be validated above
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      CompactFrameID target_id, ValidatedFrameId(target_frame),
      absl::StrCat("LookupTransform: Invalid target frame-id ('", target_frame,
                   "')"));
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      CompactFrameID source_id, ValidatedFrameId(source_frame),
      absl::StrCat("LookupTransform: Invalid source frame-id ('", source_frame,
                   "')"));

  TransformAccum accum;
  EIGENMATH_TF2_RETURN_IF_ERROR(
      WalkToTopParent(&accum, query_time, target_id, source_id), "");

  if (transform_time) {
    *transform_time = accum.time;
  }
  return accum.result;
}

struct CountFrameAccum {
  explicit CountFrameAccum(CompactFrameID sought_frame_)
      : sought_frame(sought_frame_), count(0) {}

  absl::StatusOr<CompactFrameID> Gather(const TimeCacheInterface* cache,
                                        absl::Time time) {
    EIGENMATH_TF2_ASSIGN_OR_RETURN(CompactFrameID child_id,
                                   cache->GetChild(time), "");
    if (child_id == sought_frame) {
      ++count;
    }
    return cache->GetParent(time);
  }
  void Accum(bool source) {}
  void Finalize(WalkEnding end, absl::Time _time) {}

  CompactFrameID sought_frame;
  int count;
};

absl::StatusOr<int> BufferCore::CountFrameNoLock(CompactFrameID target_id,
                                                 CompactFrameID source_id,
                                                 CompactFrameID sought_id,
                                                 const absl::Time& time) const {
  if (target_id == kRootFrameId || source_id == kRootFrameId) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid arguments: target_id = ", target_id,
                     " source_id = ", source_id));
  }

  if (target_id == source_id) {
    return (target_id == sought_id ? 1 : 0);
  }

  CountFrameAccum accum(sought_id);
  EIGENMATH_TF2_RETURN_IF_ERROR(
      WalkToTopParent(&accum, time, target_id, source_id), "");

  return accum.count;
}

struct CanTransformAccum {
  absl::StatusOr<CompactFrameID> Gather(const TimeCacheInterface* cache,
                                        absl::Time time) {
    return cache->GetParent(time);
  }
  void Accum(bool source) {}
  void Finalize(WalkEnding end, absl::Time _time) {}
};

absl::Status BufferCore::CanTransformNoLock(CompactFrameID target_id,
                                            CompactFrameID source_id,
                                            const absl::Time& time) const {
  if (target_id == kRootFrameId || source_id == kRootFrameId) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid arguments: target_id = ", target_id,
                     " source_id = ", source_id));
  }

  if (target_id == source_id) {
    return absl::OkStatus();
  }

  CanTransformAccum accum;
  return WalkToTopParent(&accum, time, target_id, source_id);
}

absl::Status BufferCore::CanTransform(std::string_view target_frame,
                                      std::string_view source_frame,
                                      const absl::Time& time) const {
  // Short circuit if target_frame == source_frame
  if (target_frame == source_frame) {
    return absl::OkStatus();
  }

  absl::ReaderMutexLock lock(&frame_mutex_);

  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      CompactFrameID target_id, ValidatedFrameId(target_frame),
      absl::StrCat("CanTransform: Invalid target frame-id ('", target_frame,
                   "')"));
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      CompactFrameID source_id, ValidatedFrameId(source_frame),
      absl::StrCat("CanTransform: Invalid source frame-id ('", source_frame,
                   "')"));

  return CanTransformNoLock(target_id, source_id, time);
}

absl::Status BufferCore::CanTransform(std::string_view target_frame,
                                      const absl::Time& target_time,
                                      std::string_view source_frame,
                                      const absl::Time& source_time,
                                      std::string_view fixed_frame) const {
  EIGENMATH_TF2_RETURN_IF_ERROR(
      CheckFrameId(target_frame),
      absl::StrCat("CanTransform: Invalid target frame-id ('", target_frame,
                   "')"));
  EIGENMATH_TF2_RETURN_IF_ERROR(
      CheckFrameId(source_frame),
      absl::StrCat("CanTransform: Invalid source frame-id ('", source_frame,
                   "')"));
  EIGENMATH_TF2_RETURN_IF_ERROR(
      CheckFrameId(fixed_frame),
      absl::StrCat("CanTransform: Invalid fixed frame-id ('", fixed_frame,
                   "')"));

  EIGENMATH_TF2_RETURN_IF_ERROR(
      CanTransform(target_frame, fixed_frame, target_time), "");
  EIGENMATH_TF2_RETURN_IF_ERROR(
      CanTransform(fixed_frame, source_frame, source_time), "");
  return absl::OkStatus();
}

absl::Status BufferCore::GetTransformTimeInterval(
    std::string_view target_frame, std::string_view source_frame,
    absl::Time* oldest_time, absl::Time* latest_time) const {
  absl::ReaderMutexLock lock(&frame_mutex_);

  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      CompactFrameID target_id, ValidatedFrameId(target_frame),
      absl::StrCat("GetTransformTimeInterval: Invalid target frame-id ('",
                   target_frame, "')"));
  EIGENMATH_TF2_ASSIGN_OR_RETURN(
      CompactFrameID source_id, ValidatedFrameId(source_frame),
      absl::StrCat("GetTransformTimeInterval: Invalid source frame-id ('",
                   source_frame, "')"));

  return GetCommonTimeInterval(target_id, source_id, oldest_time, latest_time);
}

const TimeCacheInterface* BufferCore::GetFrame(CompactFrameID frame_id) const {
  auto it = frames_.find(frame_id);
  return it == frames_.end() ? nullptr : it->second.get();
}

CompactFrameID BufferCore::LookupFrameNumber(
    std::string_view frameid_str) const {
  const auto map_it = frame_ids_.find(frameid_str);
  return map_it == frame_ids_.end() ? kRootFrameId : map_it->second;
}

CompactFrameID BufferCore::LookupOrInsertFrameNumber(
    std::string_view frameid_str, bool* inserted) {
  CompactFrameID retval = kRootFrameId;
  const auto map_it = frame_ids_.find(frameid_str);
  if (map_it == frame_ids_.end()) {
    const size_t full_hash = absl::HashOf(frameid_str);
    retval =
        static_cast<CompactFrameID>(absl::HashOf(frameid_str) & 0xFFFFFFFF);
    while (frames_.contains(retval)) {
      // Rotate / mix the hash with itself.
      retval = static_cast<CompactFrameID>(absl::HashOf(full_hash, retval));
    }
    frames_[retval] = {};
    frame_authority_.emplace(retval, "");
    frame_ids_.emplace(frameid_str, retval);
    frame_ids_reverse_.emplace(retval, frameid_str);
    *inserted = true;
  } else {
    retval = map_it->second;
    *inserted = false;
  }

  return retval;
}

absl::StatusOr<absl::string_view> BufferCore::LookupFrameString(
    CompactFrameID frame_id_num) const {
  auto it = frame_ids_reverse_.find(frame_id_num);
  if (it == frame_ids_reverse_.end()) {
    return absl::InternalError(
        absl::StrCat("Reverse lookup of frame id ", frame_id_num, " failed!"));
  }
  return absl::string_view(it->second);
}

absl::Status BufferCore::RemoveTransformFrame(std::string_view child_frame_id) {
  absl::WriterMutexLock lock(&frame_mutex_);
  // Get the compact id.
  const CompactFrameID cfid = LookupFrameNumber(child_frame_id);
  if (cfid == kRootFrameId) {
    return absl::UnavailableError(absl::StrCat(
        "Trying to remove non-existing frame '", child_frame_id, "'!"));
  }

  // Make sure there are no children to this frame.
  // This is kind of expensive, but not much of a choice.
  for (const auto& [cid, frame] : frames_) {
    if (frame == nullptr) {
      continue;
    }
    absl::StatusOr<CompactFrameID> parent_id =
        frame->GetParent(absl::InfiniteFuture());
    if (!parent_id.ok()) {
      continue;
    }
    if (*parent_id == cfid) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Invalid argument: '", child_frame_id,
          "' is not a leaf node, cannot remove a parent frame from the tree!"));
    }
  }

  // Remove the frame from the containers.
  frames_.erase(cfid);
  frame_authority_.erase(cfid);
  frame_ids_.erase(child_frame_id);
  frame_ids_reverse_.erase(cfid);

  return absl::OkStatus();
}

std::string BufferCore::AllFramesAsString() const {
  absl::ReaderMutexLock lock(&frame_mutex_);
  return this->AllFramesAsStringNoLock();
}

std::string BufferCore::AllFramesAsStringNoLock() const {
  std::stringstream mstream;

  // Regular transforms
  for (const auto& [cid, frame] : frames_) {
    if (!frame) {
      continue;
    }
    const CompactFrameID parent_id =
        frame->HasData(absl::InfiniteFuture())
            ? frame->GetData(absl::InfiniteFuture())->parent_frame_id_
            : kRootFrameId;
    mstream << "Frame " << frame_ids_reverse_.find(cid)->second
            << " exists with parent "
            << frame_ids_reverse_.find(parent_id)->second << "." << std::endl;
  }

  return mstream.str();
}

struct BufferCore::CommonTimeIntervalAccum {
  absl::Time oldest = absl::InfinitePast();
  absl::Time latest = absl::InfiniteFuture();
  const TimeCacheInterface* oldest_limiter = nullptr;
  const TimeCacheInterface* latest_limiter = nullptr;

  void Update(const TimeCacheInterface* cache) {
    if (oldest < cache->GetOldestTimestamp()) {
      oldest = cache->GetOldestTimestamp();
      oldest_limiter = cache;
    }
    if (latest > cache->GetLatestTimestamp()) {
      latest = cache->GetLatestTimestamp();
      latest_limiter = cache;
    }
  }
};

absl::Status BufferCore::CheckCommonTimeInterval(
    const CommonTimeIntervalAccum& common_interval) const {
  if (common_interval.latest < common_interval.oldest) {
    auto& lc = *common_interval.latest_limiter;
    auto& oc = *common_interval.oldest_limiter;
    return absl::UnavailableError(absl::StrCat(
        "Disjoint time intervals! Link between '",
        *LookupFrameString(*lc.GetParent(absl::InfiniteFuture())), "' and '",
        *LookupFrameString(*lc.GetChild(absl::InfiniteFuture())),
        "' with time-interval [", absl::FormatTime(lc.GetOldestTimestamp()),
        ", ", absl::FormatTime(lc.GetLatestTimestamp()),
        "] is disjoint from link between '",
        *LookupFrameString(*oc.GetParent(absl::InfiniteFuture())), "' and '",
        *LookupFrameString(*oc.GetChild(absl::InfiniteFuture())),
        "' with time-interval [", absl::FormatTime(oc.GetOldestTimestamp()),
        ", ", absl::FormatTime(oc.GetLatestTimestamp()), "]."));
  }
  return absl::OkStatus();
}

absl::Status BufferCore::GetCommonTimeInterval(CompactFrameID target_id,
                                               CompactFrameID source_id,
                                               absl::Time* oldest_time,
                                               absl::Time* latest_time) const {
  // Error if one of the frames don't exist.
  if (source_id == kRootFrameId || target_id == kRootFrameId) {
    return absl::InternalError(
        "Source or target compact frame id is zero. This should never happen.");
  }

  if (source_id == target_id) {
    const TimeCacheInterface* cache = GetFrame(source_id);
    // Set time to latest absl::Time of frameid in case of target and source
    // frame id are the same
    if (cache) {
      *oldest_time = cache->GetOldestTimestamp();
      *latest_time = cache->GetLatestTimestamp();
    } else {
      *oldest_time = absl::InfiniteFuture();
      *latest_time = absl::InfinitePast();
    }
    return absl::OkStatus();
  }

  std::vector<const TimeCacheInterface*> source_to_root_links;

  // Walk the tree to its root from the source frame, accumulating the list of
  // parent/time as well as the latest time
  // in the target is a direct parent
  CompactFrameID frame = source_id;
  int depth = 0;
  CommonTimeIntervalAccum common_interval;
  while (frame != kRootFrameId) {
    const TimeCacheInterface* cache = GetFrame(frame);

    if (!cache) {
      // There will be no cache for the very root of the tree
      break;
    }

    auto status_or_parent = cache->GetParent(absl::InfiniteFuture());
    if (!status_or_parent.ok() || *status_or_parent == kRootFrameId) {
      // Just break out here... there may still be a path from source -> target
      break;
    }

    if (cache->GetLatestTimestamp() != absl::InfiniteFuture()) {
      common_interval.Update(cache);
    }

    source_to_root_links.push_back(cache);

    frame = *status_or_parent;

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id) {
      EIGENMATH_TF2_RETURN_IF_ERROR(
          CheckCommonTimeInterval(common_interval),
          absl::StrCat("GetCommonTimeInterval between '",
                       *LookupFrameString(target_id), "' and '",
                       *LookupFrameString(source_id), "'."));
      *oldest_time = common_interval.oldest;
      *latest_time = common_interval.latest;
      return absl::OkStatus();
    }

    ++depth;
    if (depth > kMaxGraphDepth) {
      return absl::InternalError(
          absl::StrCat("The tf tree is invalid because it contains a loop.\n",
                       AllFramesAsStringNoLock(), "\n"));
    }
  }

  // Now walk to the top parent from the target frame, accumulating the
  // latest time and looking for a common parent
  frame = target_id;
  depth = 0;
  common_interval = CommonTimeIntervalAccum();
  CompactFrameID common_parent = kRootFrameId;
  while (true) {
    const TimeCacheInterface* cache = GetFrame(frame);

    if (!cache) {
      break;
    }

    auto status_or_parent = cache->GetParent(absl::InfiniteFuture());
    if (!status_or_parent.ok() || *status_or_parent == kRootFrameId) {
      break;
    }

    if (cache->GetLatestTimestamp() != absl::InfiniteFuture()) {
      common_interval.Update(cache);
    }

    frame = *status_or_parent;

    auto it =
        std::find_if(source_to_root_links.begin(), source_to_root_links.end(),
                     [&frame](const TimeCacheInterface* rhs) {
                       return *rhs->GetParent(absl::InfiniteFuture()) == frame;
                     });
    if (it != source_to_root_links.end()) {  // found a common parent
      common_parent = *(*it)->GetParent(absl::InfiniteFuture());
      break;
    }

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id) {
      EIGENMATH_TF2_RETURN_IF_ERROR(
          CheckCommonTimeInterval(common_interval),
          absl::StrCat("GetCommonTimeInterval between '",
                       *LookupFrameString(target_id), "' and '",
                       *LookupFrameString(source_id), "'."));
      *oldest_time = common_interval.oldest;
      *latest_time = common_interval.latest;
      return absl::OkStatus();
    }

    ++depth;
    if (depth > kMaxGraphDepth) {
      return absl::InternalError(
          absl::StrCat("The tf tree is invalid because it contains a loop.\n",
                       AllFramesAsStringNoLock(), "\n"));
    }
  }

  if (common_parent == kRootFrameId) {
    return absl::InternalError(absl::StrCat(
        "Could not find a connection between '", *LookupFrameString(target_id),
        "' and '", *LookupFrameString(source_id),
        "' because they are not part of the same tree.",
        " Tf has two or more unconnected trees."));
  }

  // Loop through the source -> root list until we hit the common parent
  for (const auto& link_cache : source_to_root_links) {
    if (link_cache->GetLatestTimestamp() != absl::InfiniteFuture()) {
      common_interval.Update(link_cache);
    }

    if (*link_cache->GetParent(absl::InfiniteFuture()) == common_parent) {
      break;
    }
  }

  EIGENMATH_TF2_RETURN_IF_ERROR(
      CheckCommonTimeInterval(common_interval),
      absl::StrCat("GetCommonTimeInterval between '",
                   *LookupFrameString(target_id), "' and '",
                   *LookupFrameString(source_id), "'."));
  *oldest_time = common_interval.oldest;
  *latest_time = common_interval.latest;
  return absl::OkStatus();
}

std::string BufferCore::AllFramesAsYAML(absl::Time query_time) const {
  std::stringstream mstream;
  absl::ReaderMutexLock lock(&frame_mutex_);

  TransformStorage temp;

  if (frames_.size() == 1) {
    mstream << "[]";
  }

  mstream.precision(3);
  mstream.setf(std::ios::fixed, std::ios::floatfield);

  for (const auto& [cfid, frame] : frames_) {
    if (!frame || !frame->HasData(absl::InfiniteFuture())) {
      continue;
    }

    const CompactFrameID parent_id =
        frame->GetData(absl::InfiniteFuture())->parent_frame_id_;

    const double rate =
        frame->GetBufferSize() /
        std::max((absl::ToDoubleSeconds(frame->GetLatestTimestamp() -
                                        frame->GetOldestTimestamp())),
                 0.0001);

    mstream << std::fixed;  // fixed point notation
    mstream.precision(3);   // 3 decimal places
    mstream << frame_ids_reverse_.find(cfid)->second << ":" << std::endl;
    mstream << "  parent: '" << frame_ids_reverse_.find(parent_id)->second
            << "'\n";
    mstream << "  broadcaster: '" << frame_authority_.find(cfid)->second
            << "'\n";
    mstream << "  rate: " << rate << std::endl;
    mstream << "  most_recent_transform: "
            << absl::FormatTime(frame->GetLatestTimestamp()) << std::endl;
    mstream << "  oldest_transform: "
            << absl::FormatTime(frame->GetOldestTimestamp()) << std::endl;
    if (query_time != absl::InfiniteFuture()) {
      mstream << "  transform_delay: "
              << absl::FormatDuration(query_time - frame->GetLatestTimestamp())
              << std::endl;
    }
    mstream << "  buffer_length: "
            << FormatDuration(frame->GetLatestTimestamp() -
                              frame->GetOldestTimestamp())
            << std::endl;
  }

  return mstream.str();
}

std::string BufferCore::AllFramesAsYAML() const {
  return this->AllFramesAsYAML(absl::InfiniteFuture());
}

std::vector<TransformStorage> BufferCore::CollectAllTransforms() const {
  absl::ReaderMutexLock lock(&frame_mutex_);
  std::vector<TransformStorage> result;
  for (const auto& [c_frame_id, cache] : frames_) {
    if (cache == nullptr) {
      continue;
    }
    auto cache_dump = cache->GetAllData();
    result.insert(result.end(), cache_dump.begin(), cache_dump.end());
  }
  return result;  // NRVO
}

std::string BufferCore::GetFrameId(CompactFrameID c_frame_id) const {
  absl::MutexLock lock(&frame_mutex_);
  auto it = frame_ids_reverse_.find(c_frame_id);
  if (it == frame_ids_reverse_.end()) {
    return "";
  }
  return it->second;
}

std::string BufferCore::GetFrameAuthority(CompactFrameID c_frame_id) const {
  absl::MutexLock lock(&frame_mutex_);
  auto it = frame_authority_.find(c_frame_id);
  if (it == frame_authority_.end()) {
    return "";
  }
  return it->second;
}

}  // namespace tf2
}  // namespace eigenmath
