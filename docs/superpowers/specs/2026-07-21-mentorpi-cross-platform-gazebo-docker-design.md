# MentorPi Cross-Platform Gazebo Docker Design

**Status:** Design validated; awaiting written-spec review

## Goal

Provide one reproducible Linux container for developing and running the
MentorPi forklift simulation on macOS, Linux, and Windows hosts. The container
uses ROS 2 Humble and Gazebo Fortress, while each host forwards the Gazebo GUI
over its local X11 server.

## Compatibility Baseline

- Container operating system: Ubuntu 22.04 (Jammy).
- ROS distribution: ROS 2 Humble.
- Simulator: Gazebo Fortress, installed through the official
  `ros-humble-ros-gz` package. Humble and Fortress are the recommended pair.
- Container architecture: `linux/amd64` by default.
- Linux and typical Windows/WSL2 machines run the image natively; Apple Silicon
  macOS runs the same image through Docker Desktop emulation. This favors one
  tested Gazebo/plugin ABI over a separate best-effort ARM64 image.
- The existing SDF remains Fortress-compatible: its
  `ignition-gazebo-*` system plugin names and `ignition::gazebo::*`
  namespaces are not migrated as part of the Docker work.

## Components

### Image

`docker/Dockerfile` derives from the ROS 2 Humble desktop image and installs
the ROS-Gazebo integration metapackage, Xacro, colcon, rosdep, XML validation,
and Mesa/X11 diagnostics. It does not install a host X server or alter the
host display configuration.

`docker/entrypoint.sh` sources `/opt/ros/humble/setup.bash`, then sources
`/ws/install/setup.bash` only when that workspace has been built. It executes
the passed command unchanged, so the same image supports an interactive shell,
tests, builds, and the simulator launch.

### Compose service

`compose.yaml` exposes one interactive `mentorpi-sim` service. It mounts the
repository's `ros2_ws` directory at `/ws`, uses
`TARGET_PLATFORM=linux/amd64` by default, forwards `DISPLAY` to
`host.docker.internal:0`, enables Mesa software rendering, and supplies a
Linux-only `host-gateway` fallback for `host.docker.internal`.

The service does not start Gazebo automatically. This keeps shell access,
builds, test runs, and GUI launches explicit and makes startup failures easier
to diagnose.

### Host X11 adapters

The container command is the same on every platform. Only the host setup
varies:

| Host | X11 adapter | Expected display target |
| --- | --- | --- |
| macOS | XQuartz with network clients enabled | `host.docker.internal:0` |
| Linux | Host X server, TCP enabled for the local Docker bridge | `host.docker.internal:0` |
| Windows | WSL2 plus VcXsrv, X410, or another X server accepting local Docker traffic | `host.docker.internal:0` |

Host setup commands are documented in a Docker-specific README. They grant
X11 access only for the development session and include an explicit revocation
step. The project does not commit Xauthority cookies or host-specific IP
addresses.

## Operator Workflow

1. Start Docker Desktop on macOS/Windows or the Docker daemon on Linux.
2. Start/configure the host X11 server using the README for that platform.
3. Run `docker compose build` from the repository root.
4. Run `docker compose run --rm mentorpi-sim colcon build --symlink-install`
   from `/ws`, then start an interactive shell or invoke the two-robot launch.
5. Launch the visible simulator with
   `ros2 launch mentorpi_gz_sim two_robot_sim.launch.py headless:=false`.
6. Verify the fork controller with `/robot_1/fork/command`.

## Verification

### Static

- `docker compose config` resolves the service, default platform, workspace
  mount, display variables, and image build path.
- The Dockerfile contains ROS Humble, `ros-humble-ros-gz`, Xacro, colcon, and
  X11/Mesa diagnostics.
- Documentation states the macOS, Linux, and Windows X11 prerequisites and
  the exact build, shell, simulator, and fork commands.

### Runtime

- `docker compose build` completes on a Linux/amd64 machine; Apple Silicon
  may run this same image under emulation.
- Inside the container, `ros2 pkg prefix ros_gz_sim` and
  `ign gazebo --versions` succeed.
- `colcon build --symlink-install` builds `mentorpi_description` and
  `mentorpi_gz_sim`; the existing Python/XML regression checks pass.
- `xclock` opens on the host before Gazebo is launched, proving X11 forwarding.
- Gazebo opens with `headless:=false`, displays both robots, and moving
  `/robot_1/fork/command` from `0.0` to `0.11` visibly raises the fork.

## Non-goals

- No migration from Fortress to Harmonic or later Gazebo releases.
- No native macOS or native Windows Gazebo installation.
- No GPU passthrough guarantee; Mesa software rendering is the portable
  baseline, so Apple Silicon GUI performance may be limited.
- No change to robot kinematics, SDF plugins, or ROS topic contracts.
