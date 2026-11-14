# Autonomous inspection framework

## Layers

1. **World/model layer** — Webots R2025a world, Crazyflie and ROSbot XL PROTOs. Preserve `WorldInfo.coordinateSystem "EUN"` for this environment.
2. **Device drivers** — model-specific wheel/motor control, sensors and odometry. Device names must be confirmed using `device_audit`.
3. **State estimation** — ROSbot: wheel odometry + IMU (and LiDAR SLAM when connected); Crazyflie: IMU + position estimate (GPS in simulation or an indoor localization source).
4. **Local autonomy** — ROSbot: Nav2 path following and obstacle avoidance; Crazyflie: validated PID attitude/altitude control plus waypoint guidance.
5. **Capability-based task allocation** — `task_allocator.py` assigns tasks according to required capabilities. It is deterministic and explainable, not a learned policy.
6. **Inspection/coordination** — robots report task completion and request the next task. Add a ROS 2 adapter after confirming the topic/action interfaces exposed by the exact Webots robot packages.

## Recommended execution order

1. Run device audit for both robots.
2. Verify Crazyflie using the official Webots R2025a sample controller; do not start custom motor commands before confirming motor order and axis conventions.
3. Verify ROSbot XL teleoperation and odometry; then use the Husarion ROSbot XL Webots + SLAM Toolbox + Nav2 stack.
4. Run the mission allocator and inspect `assignments.json`.
5. Connect assigned waypoints to each robot's validated navigation interface.
6. Test one task at a time, then enable cooperative task switching and mutual assistance.

## Official references

- Webots R2025a release notes mention the Crazyflie PID demo and ROSbot XL demo: https://www.cyberbotics.com/doc/blog/Webots-2025-a-release
- Bitcraze Crazyflie Webots controller documentation: https://www.bitcraze.io/documentation/repository/crazyflie-simulation/main/functional_areas/controllers/
- Husarion ROSbot XL Webots + SLAM Toolbox + Nav2 guide: https://husarion.com/tutorials/vulcanexus/webots-rosbot-xl/
