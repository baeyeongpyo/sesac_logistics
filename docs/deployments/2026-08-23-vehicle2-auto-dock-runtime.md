# Vehicle 2 Auto Dock runtime deployment — 2026-08-23

- Code commit: `eee7539` (`Pause auto dock on first target candidate`)
- Vehicle: 2 (`ROS_DOMAIN_ID=216`)
- Source snapshot: `tmp_vehicle_pose_config_near25.json`, captured from vehicle 1 on 2026-08-21
- Vehicle snapshot copy: `/shared/vehicle_pose_config.vehicle1_20260821.json`
- Active runtime config: `/shared/vehicle_pose_config.json`
- Runtime override added: `search_linear_speed_m_s = 0.08`
- Target candidate stop delay: `candidate_stop_delay_sec = 1.0` (code default)
- External webcams default: `disable_external_webcams = true`
- `search_circle_diameter_m` is absent, so Auto Dock uses its `1.34 m` fallback.
- Restored calibration includes `centerline_offset_cm = 4.0`, `insertion_distance_cm = 9.0`, and `near_center_check_distance_cm = 25.0`.
- Auto Dock loads this file again on the next arrival trigger.
