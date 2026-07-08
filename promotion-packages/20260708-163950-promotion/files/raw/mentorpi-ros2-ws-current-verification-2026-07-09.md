---
title: MentorPi ros2_ws Current Source Verification 2026-07-09
created: 2026-07-09
updated: 2026-07-09
type: raw
status: active
tags:
  - robotics
  - mentorpi
  - ros2
  - raw-source
  - verification
sources:
  - title: Current tracking vehicle ROS2 workspace source path
    path: /Users/yeongpyo/project/product/wiki-binding2/artifacts/tracking-vehicle/raw/ros2_ws
    accessed: 2026-07-09
  - title: Preserved tracking vehicle ROS2 workspace raw copy
    path: llm-wiki/raw/mentorpi-ros2-ws-group-control-2026-07-08
    accessed: 2026-07-09
  - title: Current source SHA-256 manifest
    path: llm-wiki/raw/mentorpi-ros2-ws-current-source-2026-07-09.sha256
    accessed: 2026-07-09
---

# MentorPi ros2_ws Current Source Verification 2026-07-09

## Summary

The current source workspace at:

```text
/Users/yeongpyo/project/product/wiki-binding2/artifacts/tracking-vehicle/raw/ros2_ws
```

was compared with the preserved local raw copy at:

```text
llm-wiki/raw/mentorpi-ros2-ws-group-control-2026-07-08
```

The source contains 431 files. The preserved copy contains 430 files. The only
file present in the current source but absent from the preserved copy is:

```text
.DS_Store
```

After excluding `.DS_Store`, all 430 source files match the preserved raw copy
by relative-path SHA-256 hash.

## Verification Evidence

The current source manifest is stored at:

```text
llm-wiki/raw/mentorpi-ros2-ws-current-source-2026-07-09.sha256
```

The generated current-source manifest hash was:

```text
d5a34720010acdf837badfa92048984879542d97ebaa9b7fb4561a9590d250df
```

The same hash was generated from the preserved local raw copy after using the
same relative-path format and excluding `.DS_Store`.

## Raw Handling Decision

The existing 2026-07-08 raw copy remains the canonical preserved source tree for
the MentorPi `ros2_ws` code because the current source code content is
identical. A duplicate 254 MB raw tree was not added. The current-state evidence
is preserved as a dated SHA-256 manifest and this verification note.

