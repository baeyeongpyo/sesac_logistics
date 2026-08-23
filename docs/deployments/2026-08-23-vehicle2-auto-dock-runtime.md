# Vehicle 2 Auto Dock runtime deployment — 2026-08-23

- Code commit: `eee7539` (`Pause auto dock on first target candidate`)
- Vehicle: 2 (`ROS_DOMAIN_ID=216`)
- Source snapshot: `tmp_vehicle_pose_config_near25.json`, captured from vehicle 1 on 2026-08-21
- Vehicle snapshot copy: `/shared/vehicle_pose_config.vehicle1_20260821.json`
- Active runtime config: `/shared/vehicle_pose_config.json`
- Runtime override added: `search_linear_speed_m_s = 0.08`
- Target candidate stop delay: `candidate_stop_delay_sec = 0.5`
- External webcams default: `disable_external_webcams = true`
- Directional LiDAR clearances: `lidar_front_clearance_m`, `lidar_rear_clearance_m`, `lidar_left_clearance_m`, and `lidar_right_clearance_m`; all initially `0.35 m`.
- Complete 2x2 pallet faces publish through `/robot_2/tag_entity_map` after `odom → map` TF conversion.
- Persistent entity map storage: `/shared/tag_entity_map.json`.
- Entity map schema 10 stores the `map <- odom` transform and rebases every saved pallet face when AMCL corrects that transform, preventing one physical side face from becoming a distant extra pallet.
- The mixed schema-9 runtime map was preserved as `/shared/tag_entity_map.schema9-tf-jump-backup.json` before rebuilding the two physical pallets.
- The development GUI renders `/map` with entity position, face direction, and its four-symbol matrix without RViz.
- `search_circle_diameter_m` is absent, so Auto Dock uses its `1.34 m` fallback.
- Restored calibration includes `centerline_offset_cm = 4.0`, `insertion_distance_cm = 9.0`, and `near_center_check_distance_cm = 25.0`.
- Auto Dock loads this file again on the next arrival trigger.

## Post-insertion handoff

- Auto Dock publishes `std_msgs/msg/Empty` on `/robot_2/auto_dock/entry_complete` after insertion and waits without commanding the fork directly.
- Fork Control receives that event, lifts until the upper limit switch, then publishes `std_msgs/msg/Empty` on `/robot_2/lift/up_complete`.
- Auto Dock receives lift completion, reverses clear of the pallet location, then turns using odometry and publishes `std_msgs/msg/Empty` on `/robot_2/auto_dock/drive_ready` when the turn is within tolerance.
- Runtime config keys are `post_lift_reverse_distance_cm` (default `30.0`), `post_lift_reverse_speed_m_s` (default `0.05`), `post_lift_turn_deg` (default `180.0`), `post_lift_turn_speed_rad_s` (default `0.30`), and `post_lift_turn_tolerance_deg` (default `3.0`).
- A rear LiDAR violation pauses the reverse; an all-direction violation pauses the subsequent turn. Neither case issues a translational safety backoff.
