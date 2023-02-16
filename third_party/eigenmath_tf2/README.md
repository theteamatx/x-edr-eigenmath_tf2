# eigenmath_tf2 library

tf2 is the second generation of the tf library. This is a library originally
shipped with ROS (Robot Operating System), and its original version can be found
in //third_party/ros/tf2. This new version ports the main functionality into a
library that does not depend on ROS nor on Boost, and instead uses code from
Abseil, Eigenmath and Genit.

This library implements the interface defined by eigenmath::tf2::BufferCore.

## Code API

The main interface is through the eigenmath::tf2::BufferCore interface.

It uses the statuses from //absl/status, the time-stamps from
//absl/time, and the transforms types from //eigenmath.
