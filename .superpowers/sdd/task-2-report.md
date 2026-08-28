# Task 2 — Restore original mesh geometry with compatible frame names

## Scope

Restored only the ROS 2 description package’s upstream Mecanum visual and
collision meshes, transforms, and package installation metadata. The Gazebo
SDF model was intentionally not changed.

## RED evidence

Command:

```text
python3 ros2_ws/src/mentorpi_description/test/test_original_model.py
```

Before the implementation, the test failed at line 20 because no mesh
filenames were present:

```text
AssertionError: 'package://mentorpi_description/meshes/mecanum/base_link.STL' not found in []
```

## Implementation

- Extracted the seven specified binary STL files directly from the preserved
  upstream archive using the task’s `tar --strip-components=4` command.
- Replaced the base, wheel, lidar, and camera primitive geometry with upstream
  mesh geometry in both visual and collision elements.
- Restored source wheel joint names, origins, and axes; restored the source
  base-to-footprint transform.
- Kept public consumer frame names `base_laser` and `depth_camera_link`, with
  joint names `laser_joint` and `depth_camera_joint`.
- Added `meshes` to the install rule and updated the package description.

## GREEN evidence for Task 2 scope

The same focused test now passes every URDF and source-asset assertion. Its
sole remaining failure is Task 3’s intentionally untouched Gazebo SDF pose:

```text
AssertionError: '<link name="base_laser"><pose>-0.012242 -0.00008533 0.162501 0 0 0</pose>' not found in ...model.sdf.xacro...
```

Additional verification succeeded:

```text
URDF structure, all 14 mesh references, transforms, axes, and seven asset hashes verified
Xacro expansion verified
```

## Files changed

- `ros2_ws/src/mentorpi_description/meshes/mecanum/base_link.STL`
- `ros2_ws/src/mentorpi_description/meshes/mecanum/wheel_lf_Link.STL`
- `ros2_ws/src/mentorpi_description/meshes/mecanum/wheel_rf_Link.STL`
- `ros2_ws/src/mentorpi_description/meshes/mecanum/wheel_lb_Link.STL`
- `ros2_ws/src/mentorpi_description/meshes/mecanum/wheel_rb_Link.STL`
- `ros2_ws/src/mentorpi_description/meshes/mecanum/lidar_Link.STL`
- `ros2_ws/src/mentorpi_description/meshes/mecanum/cam_Link.STL`
- `ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro`
- `ros2_ws/src/mentorpi_description/CMakeLists.txt`
- `ros2_ws/src/mentorpi_description/package.xml`

## Self-review

- Confirmed all seven extracted files byte-match the archive SHA-256 digests.
- Confirmed each mesh appears once in a visual and once in a collision element.
- Confirmed `base_laser`, `depth_camera_link`, `laser_joint`, and
  `depth_camera_joint` remain present with source-compatible origins.
- Confirmed all four wheel origins and axes match the upstream Xacro.
- Confirmed the source Xacro expands successfully.
- Confirmed `model.sdf.xacro` has no Task 2 changes.

## Concerns

The focused parity test remains non-zero until Task 3 updates the two Gazebo
SDF sensor poses. This is the required and expected remaining failure; it does
not indicate a Task 2 URDF or mesh restoration issue.
