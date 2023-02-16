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

#ifndef EIGENMATH_TF2_EIGENMATH_TF2_TIME_CACHE_H_
#define EIGENMATH_TF2_EIGENMATH_TF2_TIME_CACHE_H_

#include <memory>

#include "eigenmath_tf2/circular_buffer.h"
#include "third_party/eigenmath_tf2/transform_storage.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace eigenmath {
namespace tf2 {

class TimeCacheInterface {
 public:
  virtual ~TimeCacheInterface();

  // Check if data can be provided by the cache
  virtual bool HasData(absl::Time time) const = 0;

  // Access data from the cache
  virtual absl::StatusOr<TransformStorage> GetData(absl::Time time) const = 0;

  // Access all data from the cache as a vector of transforms.
  virtual std::vector<TransformStorage> GetAllData() const = 0;

  // Insert data into the cache
  virtual bool InsertData(const TransformStorage& new_data) = 0;

  // Clear the list of stored values
  virtual void Clear() = 0;

  // Retrieve the parent at a specific time
  virtual absl::StatusOr<CompactFrameID> GetParent(absl::Time time) const = 0;

  // Retrieve the parent at a specific time
  virtual absl::StatusOr<CompactFrameID> GetChild(absl::Time time) const = 0;

  // Get the time interval stored in this cache, and the parent associated
  // with it.  Returns parent = 0 if no data.
  virtual TimeIntervalAndFrameID GetTimeIntervalAndParent() const = 0;

  // Debugging information methods
  // Get the length of the stored list
  virtual int GetBufferSize() const = 0;

  // Get the latest timestamp cached
  virtual absl::Time GetLatestTimestamp() const = 0;

  // Get the oldest timestamp cached
  virtual absl::Time GetOldestTimestamp() const = 0;
};

// A class to keep a circular buffer of transforms, sorted in time.
// This builds and maintains a buffer of timestamped
// data. And provides lookup functions to get
// data out as a function of time.
class TimeCache : public TimeCacheInterface {
 public:
  // Duration of time to not interpolate below (i.e., time tolerance).
  static const absl::Duration kMinInterpolationDistance;
  // The default amount of time to cache data.
  static const absl::Duration kDefaultCacheTime;

  TimeCache(absl::Duration max_storage_time = kDefaultCacheTime);

  // Virtual methods

  bool HasData(absl::Time time) const override;
  absl::StatusOr<TransformStorage> GetData(absl::Time time) const override;
  bool InsertData(const TransformStorage& new_data) override;
  std::vector<TransformStorage> GetAllData() const override;
  void Clear() override;
  absl::StatusOr<CompactFrameID> GetParent(absl::Time time) const override;
  absl::StatusOr<CompactFrameID> GetChild(absl::Time time) const override;
  TimeIntervalAndFrameID GetTimeIntervalAndParent() const override;

  // Debugging information methods
  int GetBufferSize() const override;
  absl::Time GetLatestTimestamp() const override;
  absl::Time GetOldestTimestamp() const override;

 private:
  CircularBuffer<TransformStorage> storage_;

  absl::Duration max_storage_time_;

  // A helper function for getData
  absl::Status FindClosest(absl::Time target_time, TransformStorage const** one,
                           TransformStorage const** two) const;

  void Interpolate(const TransformStorage& one, const TransformStorage& two,
                   absl::Time time, TransformStorage* output) const;

  void PruneStorage();
};

// A class that trivially keeps track of a single transform that is assumed
// to be constant for all time (past and future).
class StaticCache : public TimeCacheInterface {
 public:
  // Virtual methods

  bool HasData(absl::Time time) const override;
  absl::StatusOr<TransformStorage> GetData(absl::Time time) const override;
  bool InsertData(const TransformStorage& new_data) override;
  std::vector<TransformStorage> GetAllData() const override;
  void Clear() override;
  absl::StatusOr<CompactFrameID> GetParent(absl::Time time) const override;
  absl::StatusOr<CompactFrameID> GetChild(absl::Time time) const override;
  TimeIntervalAndFrameID GetTimeIntervalAndParent() const override;

  // Debugging information methods
  int GetBufferSize() const override;
  absl::Time GetLatestTimestamp() const override;
  absl::Time GetOldestTimestamp() const override;

 private:
  TransformStorage  storage_;
};

}  // namespace tf2
}  // namespace eigenmath

#endif  // EIGENMATH_TF2_EIGENMATH_TF2_TIME_CACHE_H_
