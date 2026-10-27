# Lab 1: ROS 2 & Turtlesim

## Team Members
- Nadia Asser - 2304153
- Marina Hany - 2304143
- Mohanned Mohammed - 2304130

## Packages Overview
- **turtle_interfaces**: Custom msg, srv, and action definitions.
- **turtle_control**: Core logic node (`turtle_manager.py`) and launch setup.

## Running the Project
```bash
ros2 launch turtle_control turtle_app.launch.py
```

### Commands
- GoToPose: ros2 action send_goal /go_to_pose turtle_interfaces/action/GoToPose "{target: {x: 8.0, y: 8.0, yaw: 1.57}}" --feedback
- Reset: ros2 service call /reset_turtle std_srvs/srv/Empty
- Draw Square: ros2 action send_goal /draw_square turtle_interfaces/action/DrawSquare "{side_length: 2.5}" --feedback
- Set Speed Limits: ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: 1.0, max_angular_speed: 1.0}"

### Bonus
1. Architectural Design Patterns (+1 Mark)

    Facade Pattern (/reset_turtle service):
    Encapsulates three distinct sub-services provided by Turtlesim (/turtle1/set_pen to disable drawing, /turtle1/teleport_absolute to return to center, and /clear to erase old paths) behind a single unified interface. This shields clients from low-level call coordination and prevents unwanted artifacts.

    Sequential / State-Driven Pattern (/draw_square action):
    Coordinates movement along the four edges sequentially. The action acts as a high-level state coordinator, waiting for each target waypoint to be reached before transitioning to the next edge and emitting edge progress feedback.
   
2. ROS Bag Recording (+1 Mark)

Recorded real-time data on /turtle1/cmd_vel and /turtle1/pose during motion.
 <img width="963" height="402" alt="Screenshot from 2026-10-27 11-08-57" src="https://github.com/user-attachments/assets/8f57e779-77bf-4109-b15b-c3c88f503666" />
