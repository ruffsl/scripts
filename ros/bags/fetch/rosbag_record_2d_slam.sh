#!/bin/bash

rosbag record \
  -o raw_2d_fetch \
  /base_scan \
  /base_scan_raw \
  /imu \
  /joint_states \
  /odom \
  /robot_state \
  /rosout_agg \
  /tf \
  /tf_static

  # # /base_camera/depth_downsample/camera_info \
  # # /base_camera/depth_downsample/image_mono16 \
  # # /base_camera/depth_downsample/image_noise \
  # # /base_camera/depth_downsample/image_raw \
  # # /base_camera/depth_downsample/parameter_descriptions \
  # # /base_camera/depth_downsample/parameter_updates \
  # # /base_camera/depth_downsample/points \
  # # /base_controller/command \
  # # /base_controller/command_limited \
  # # /base_controller/params \
  # /base_scan \
  # /base_scan_raw \
  # # /battery_state \
  # # /charge_lockout/cancel \
  # # /charge_lockout/feedback \
  # # /charge_lockout/goal \
  # # /charge_lockout/result \
  # # /charge_lockout/status \
  # # /clicked_point \
  # # /cmd_vel \
  # # /cmd_vel_mux/selected \
  # /diagnostics \
  # # /dock/result \
  # # /graft/state \
  # /imu \
  # /initialpose \
  # /joint_states \
  # # /joy \
  # # /map \
  # # /map_updates \
  # # /move_base/NavfnROS/plan \
  # # /move_base/TrajectoryPlannerROS/local_plan \
  # # /move_base/local_costmap/costmap \
  # # /move_base/local_costmap/costmap_updates \
  # # /move_base_simple/goal \
  # /odom \
  # # /odom_combined \
  # # /query_controller_states/cancel \
  # # /query_controller_states/feedback \
  # # /query_controller_states/goal \
  # # /query_controller_states/result \
  # # /query_controller_states/status \
  # /robot_state \
  # # /robotsound \
  # /rosout \
  # /rosout_agg \
  # # /sick_tim551_2050001/parameter_descriptions \
  # # /sick_tim551_2050001/parameter_updates \
  # # /sound_play/cancel \
  # # /sound_play/feedback \
  # # /sound_play/goal \
  # # /sound_play/result \
  # # /sound_play/status \
  # /statistics \
  # # /teleop/cmd_vel \
  # /tf \
  # /tf_static
