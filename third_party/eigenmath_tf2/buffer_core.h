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

#ifndef EIGENMATH_TF2_EIGENMATH_TF2_BUFFER_CORE_H_
#define EIGENMATH_TF2_EIGENMATH_TF2_BUFFER_CORE_H_

#include <memory>
#include <string>
#include <vector>

#include "eigenmath/pose2.h"
#include "eigenmath/pose3.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "third_party/eigenmath_tf2/time_cache.h"
#include "third_party/eigenmath_tf2/transform_storage.h"

namespace eigenmath {
namespace tf2 {

// A Class which provides coordinate transforms between any two frames
// in a system.
//
// This class provides a simple interface to allow recording and lookup of
// relationships between arbitrary frames of the system.
//
// TF2 assumes that there is a tree of coordinate frame transforms which
// define the relationship between all coordinate frames. For example your
// typical robot would have a transform from global to real world.  And then
// from base to hand, and from base to head. But Base to Hand really is composed
// of base to shoulder to elbow to wrist to hand. TF2 is designed to take care
// of all the intermediate steps for you.
//
// Internal Representation
// TF2 will store frames with the parameters necessary for generating the
// transform into that frame from it's parent and a reference to the parent
// frame. Frames are designated using a string. 0 is a frame without a parent
// (the top of a tree) The positions of frames over time must be pushed in.
class BufferCore {
 public:
  /************* Constants ***********************/
  // The default amount of time to cache data in seconds.
  static const absl::Duration kDefaultCacheTime;
  // The maximum depth of the tree.
  static constexpr int kMaxGraphDepth = 1000;

  // Constructor
  // \param cache_time How long to keep a history of transforms
  explicit BufferCore(absl::Duration cache_time);
  BufferCore() : BufferCore(kDefaultCacheTime) {}

  // Clear all data
  void Clear() ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Get how far into the past this buffer will keep transforms
  absl::Duration GetCacheLength() const { return cache_time_; }

  // Add transform information to the tf data structure
  // \param pose The transform to store
  // \param stamp The absl::Time of the transform to store
  // \param parent_frame_id The frame id of the parent frame
  // \param child_frame_id The frame id of the child frame
  // \param authority The source of the information for this transform
  // \param is_static Record this transform as a static transform.  It will be
  // good across all time.  (This cannot be changed after the first call.)
  // \param max_interpolation_duration Determines the maximum duration between
  // this and the following transform that still shall allow interpolation. If
  // the temporal gap is larger than this duration, a lookup error is yielded.
  // Accordingly, zero or negative values disallow inerpolation in between this
  // and the next transform.
  // \param invert_when_interpolating If true, transforms will be inverted
  // before interpolation and the inverse interpolant is used to compute the
  // desired transform.
  // \return true unless an error occurred
  absl::Status SetTransform(
      const Pose3d& pose, absl::Time stamp,
      std::string_view parent_frame_id, std::string_view child_frame_id,
      std::string_view authority, bool is_static = false,
      absl::Duration max_interpolation_duration = absl::InfiniteDuration(),
      bool invert_when_interpolating = false) ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Remove a transform frame from the tf data structure.
  // The removed frame must be a leaf node of the tree, i.e., no children.
  // All the cached transform history of the link between this child frame
  // and its parent will be removed permanently from the tree.
  // Returns an UNAVAILABLE failure if the frame does not exist, or an
  // INVALID_ARGUMENT failure if the frame has children.
  absl::Status RemoveTransformFrame(std::string_view child_frame_id)
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Get the transform between two frames by frame ID.
  // \param target_frame The frame to which data should be transformed.
  // \param source_frame The frame where the data originated.
  // \param query_time The time at which the value of the transform is desired.
  // (0 will get the latest)
  // \param transform_time (optional) Outputs the time of transform found.
  // \return The transform between the frames.
  //
  // Possible error statuses NOT_FOUND, INVALID_ARGUMENT, INTERNAL_ERROR
  absl::StatusOr<Pose3d> LookupTransform(
      std::string_view target_frame, std::string_view source_frame,
      absl::Time query_time, absl::Time* transform_time = nullptr) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Same as LookupTransform above, but the given `horizontal_frame` is
  // treated as approximately horizontal, meaning that transform chains on
  // either side of it can be projected to 2D to from a target-to-source
  // transform in 2d.
  absl::StatusOr<eigenmath::Pose2d> LookupTransform2d(
      std::string_view target_frame, std::string_view source_frame,
      absl::Time query_time, std::string_view horizontal_frame,
      absl::Time* transform_time = nullptr) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Get the transform between two frames by frame ID assuming fixed frame.
  // The function outputs the transform target_pose_source by composing it with
  // the transform target_pose_fixed and fixed_pose_source. The purpose of this
  // is to allow mismatching time-stamps in those two transforms, which is
  // reasonable only if the intermediate frame is known to be fixed.
  // \param target_frame The frame to which data should be transformed.
  // \param target_time The time to which the data should be transformed.
  // (0 will get the latest)
  // \param source_frame The frame where the data originated.
  // \param source_time The time at which the source_frame should be evaluated.
  // (0 will get the latest)
  // \param fixed_frame The frame in which to assume the transform is constant.
  // \param target_transform_time (optional) Outputs the time of transform
  //                              found between target -> fixed.
  // \param source_transform_time (optional) Outputs the time of transform
  //                              found between fixed -> source.
  // \return The transform between the frames.
  //
  // Possible error statuses NOT_FOUND, INVALID_ARGUMENT, INTERNAL_ERROR
  absl::StatusOr<Pose3d> LookupTransform(
      std::string_view target_frame, const absl::Time& target_time,
      std::string_view source_frame, const absl::Time& source_time,
      std::string_view fixed_frame,
      absl::Time* target_transform_time = nullptr,
      absl::Time* source_transform_time = nullptr) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Same as LookupTransform above, but the given `horizontal_frame` is
  // treated as approximately horizontal, meaning that transform chains on
  // either side of it can be projected to 2D to from a target-to-source
  // transform in 2d.
  absl::StatusOr<eigenmath::Pose2d> LookupTransform2d(
      std::string_view target_frame, const absl::Time& target_time,
      std::string_view source_frame, const absl::Time& source_time,
      std::string_view fixed_frame, absl::string_view horizontal_frame,
      absl::Time* target_transform_time = nullptr,
      absl::Time* source_transform_time = nullptr) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Test if a transform is possible.
  // \param target_frame The frame into which to transform.
  // \param source_frame The frame from which to transform.
  // \param time The time at which to transform.
  // \return True if the transform is possible, false otherwise.
  absl::Status CanTransform(std::string_view target_frame,
                            std::string_view source_frame,
                            const absl::Time& time) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Test if a transform is possible.
  // \param target_frame The frame into which to transform.
  // \param target_time The time into which to transform.
  // \param source_frame The frame from which to transform.
  // \param source_time The time from which to transform.
  // \param fixed_frame The frame in which to treat the transform as constant.
  // \return True if the transform is possible, false otherwise
  absl::Status CanTransform(std::string_view target_frame,
                            const absl::Time& target_time,
                            std::string_view source_frame,
                            const absl::Time& source_time,
                            std::string_view fixed_frame) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Test if a transform is possible and output the oldest and latest time
  // stamp for this there is a transform from target to source.
  // \param target_frame The frame into which to transform.
  // \param source_frame The frame from which to transform.
  // \param oldest_time The oldest time at which the transform exists.
  // \param latest_time The latest time at which the transform exists.
  // \return True if the transform is possible, false otherwise.
  absl::Status GetTransformTimeInterval(std::string_view target_frame,
                                        std::string_view source_frame,
                                        absl::Time* oldest_time,
                                        absl::Time* latest_time) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // A way to see what frames have been cached in yaml format
  // Useful for debugging tools
  std::string AllFramesAsYAML(absl::Time query_time) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Backwards compatibility for #84
  std::string AllFramesAsYAML() const ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // A way to see what frames have been cached
  std::string AllFramesAsString() const ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Collect a vector of all transforms stored in all caches, in no particular
  // order. This can be used to dump all transforms (e.g., to restore state).
  std::vector<TransformStorage> CollectAllTransforms() const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

  // Gets the string frame id corresponding to a compact frame id.
  std::string GetFrameId(CompactFrameID c_frame_id) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);
  // Gets the latest authority corresponding to a compact frame id.
  std::string GetFrameAuthority(CompactFrameID c_frame_id) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);

 private:
  // A way to see what frames have been cached
  // Useful for debugging. Use this call internally.
  std::string AllFramesAsStringNoLock() const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);

  /******************** Internal Storage ****************/

  static constexpr CompactFrameID kRootFrameId = 0;

  // The pointers to potential frames that the tree can be made of.
  // The frames will be allocated at run time when set the first time.
  absl::flat_hash_map<CompactFrameID, std::unique_ptr<TimeCacheInterface>>
      frames_ ABSL_GUARDED_BY(frame_mutex_);

  // A mutex to protect testing and allocating new frames on the above vector.
  mutable absl::Mutex frame_mutex_;

  // A map from string frame ids to CompactFrameID
  absl::flat_hash_map<std::string, CompactFrameID> frame_ids_
      ABSL_GUARDED_BY(frame_mutex_);
  // A map from CompactFrameID frame_id_numbers to string for debugging
  absl::flat_hash_map<CompactFrameID, std::string> frame_ids_reverse_
      ABSL_GUARDED_BY(frame_mutex_);
  // A map to lookup the most recent authority for a given frame
  absl::flat_hash_map<CompactFrameID, std::string> frame_authority_
      ABSL_GUARDED_BY(frame_mutex_);

  /// How long to cache transform history
  absl::Duration cache_time_;

  /************************* Internal Functions ****************************/

  // Get a frame, which will return nullptr if the frame does not exist.
  // \param frame_number The frameID of the desired reference frame.
  const TimeCacheInterface* GetFrame(CompactFrameID c_frame_id) const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);

  TimeCacheInterface* GetOrAllocateFrame(CompactFrameID cfid, bool is_static)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(frame_mutex_);

  absl::Status CheckFrameId(
      std::string_view frame_id) const;
  absl::StatusOr<CompactFrameID> ValidatedFrameId(
      std::string_view frame_id) const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);

  // String to number for frame lookup with dynamic allocation of new frames
  CompactFrameID LookupFrameNumber(std::string_view frameid_str) const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);

  // String to number for frame lookup with dynamic allocation of new frames
  CompactFrameID LookupOrInsertFrameNumber(std::string_view frameid_str,
                                           bool* inserted)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(frame_mutex_);

  // Number to string frame lookup may throw LookupException if number invalid
  absl::StatusOr<std::string_view> LookupFrameString(
      CompactFrameID frame_id_num) const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);

  absl::StatusOr<eigenmath::Pose3d> LookupTransformNoLock(
      std::string_view target_frame, std::string_view source_frame,
      absl::Time query_time, absl::Time* transform_time) const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);

  struct CommonTimeIntervalAccum;  // forward-decl

  absl::Status CheckCommonTimeInterval(
      const CommonTimeIntervalAccum& common_interval) const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);

  // Return the latest time which is common across the spanning set
  // zero if fails to cross.
  absl::Status GetCommonTimeInterval(CompactFrameID target_id,
                                     CompactFrameID source_id,
                                     absl::Time* oldest_time,
                                     absl::Time* latest_time) const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);

  // Traverse the transform tree, with visitor f.
  template <typename F>
  absl::Status WalkToTopParent(F* f, absl::Time time, CompactFrameID target_id,
                               CompactFrameID source_id) const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);
  absl::Status CanTransformInternal(CompactFrameID target_id,
                                    CompactFrameID source_id,
                                    const absl::Time& time) const
      ABSL_LOCKS_EXCLUDED(frame_mutex_);
  absl::Status CanTransformNoLock(CompactFrameID target_id,
                                  CompactFrameID source_id,
                                  const absl::Time& time) const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);
  absl::StatusOr<int> CountFrameNoLock(CompactFrameID target_id,
                                       CompactFrameID source_id,
                                       CompactFrameID sought_id,
                                       const absl::Time& time) const
      ABSL_SHARED_LOCKS_REQUIRED(frame_mutex_);
};

}  // namespace tf2
}  // namespace eigenmath

#endif  // EIGENMATH_TF2_EIGENMATH_TF2_BUFFER_CORE_H_
