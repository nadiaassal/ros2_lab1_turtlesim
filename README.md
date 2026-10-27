# Lab 1: ROS 2 & Turtlesim

### Team Members
- Nadia Asser - 2304153
- Marina Hany - 2304143
- Mohanned Mohammed - 2304130

### Packages
- turtle_interfaces: Custom msg, srv, and action definitions.
- turtle_control: Node and launch files.

### How to Run
```bash
ros2 launch turtle_control turtle_app.launch.py
```

### Commands
- GoToPose: ros2 action send_goal /go_to_pose turtle_interfaces/action/GoToPose "{target: {x: 8.0, y: 8.0, yaw: 1.57}}" --feedback
- Reset: ros2 service call /reset_turtle std_srvs/srv/Empty
- Draw Square: ros2 action send_goal /draw_square turtle_interfaces/action/DrawSquare "{side_length: 2.5}" --feedback
- Set Speed Limits: ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: 1.0, max_angular_speed: 1.0}"

### Bonus
- Design Patterns: Facade Pattern used in reset_turtle service to encapsulate clear, teleport, and set_pen calls.
- ROS Bag: Recorded topics /turtle1/cmd_vel and /turtle1/pose into my_square_bag.
 <img width="963" height="402" alt="Screenshot from 2026-10-27 11-08-57" src="https://github.com/user-attachments/assets/8f57e779-77bf-4109-b15b-c3c88f503666" />
