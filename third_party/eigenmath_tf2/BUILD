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

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

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
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/hash",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@x_edr_eigenmath//eigenmath",
    ],
)

cc_test(
    name = "buffer_core_test",
    srcs = ["buffer_core_test.cc"],
    deps = [
        ":eigenmath_tf2",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "time_cache_test",
    srcs = ["time_cache_test.cc"],
    deps = [
        ":eigenmath_tf2",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
        "@x_edr_eigenmath//eigenmath",
        "@x_edr_eigenmath//eigenmath:matchers",
    ],
)
