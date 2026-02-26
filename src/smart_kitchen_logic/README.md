# HowTo

## Command for static start
ros2 run smart_kitchen_logic robot_controller_node
ros2 run smart_kitchen_logic command_executor
ros2 run smart_kitchen_logic command_sequence_client

## Command for dynamic start
### Prerequisites
- Start the camera
- Start the robot

### Command
ros2 run smart_kitchen_logic robot_controller
ros2 run smart_kitchen_logic command_executor
ros2 run smart_kitchen_logic moving_node
ros2 run smart_kitchen_logic planning_node
ros2 run aruco_tracker aruco_all_distance