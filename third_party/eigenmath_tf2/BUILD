# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

licenses(["notice"])

exports_files(["LICENSE"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "eigenmath_tf2",
    srcs = [
        "buffer_core.cc",
        "time_cache.cc",
    ],
    hdrs = [
        "buffer_core.h",
        "time_cache.h",
        "transform_storage.h",
    ],
    deps = [
        "//eigenmath_tf2:circular_buffer",
        "@abseil-cpp//absl/container:flat_hash_map",
        "@abseil-cpp//absl/hash",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/synchronization",
        "@abseil-cpp//absl/time",
        "@x_edr_eigenmath//eigenmath",
    ],
    strip_include_prefix = "//third_party",
)

cc_test(
    name = "buffer_core_test",
    srcs = ["buffer_core_test.cc"],
    deps = [
        ":eigenmath_tf2",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "time_cache_test",
    srcs = ["time_cache_test.cc"],
    deps = [
        ":eigenmath_tf2",
        "@abseil-cpp//absl/time",
        "@googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:matchers",
    ],
)
