# Official integration references (checked 2026-11-17)

Only ROS 2 Jazzy / Ubuntu 24.04 is a supported deployment target here.

- [TurtleBot4 PC setup](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html)
- [TurtleBot4 wired address and camera power saving](https://turtlebot.github.io/turtlebot4-user-manual/troubleshooting/faq.html)
- [Simple Discovery](https://turtlebot.github.io/turtlebot4-user-manual/setup/simple_discovery.html)
- [Discovery Server](https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html)
- [SLAM launch: params, sync, namespace](https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_navigation/launch/slam.launch.py)
- [Nav2 launch: params_file](https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_navigation/launch/nav2.launch.py)
- [AMCL launch: map, params](https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_navigation/launch/localization.launch.py)
- [Nav2 MPPI and TwistStamped configuration](https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_navigation/config/nav2.yaml)
- [RGB-only default camera configuration](https://github.com/turtlebot/turtlebot4_robot/blob/jazzy/turtlebot4_bringup/config/oakd_pro.yaml)
- [slam_toolbox origin behavior](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/slam_toolbox_common.cpp)

The launcher reads the installed TurtleBot4 configuration and stores its resolved
SLAM/Nav2/AMCL variants in each run's output/config directory. This preserves
upstream plugin configuration and remapping rather than maintaining a copied
Nav2 configuration. `map_start_pose` only controls loading a previous pose graph;
`origin_anchor` explicitly anchors fresh mapping at the first hardware odometry
pose instead. No saved pose graph is loaded during fresh SLAM.
