Commands

# Initialize robot arm itself:
Start Camera:
ros2 launch kinova_vision kinova_vision.launch.py launch_depth:=false device:=10.163.18.200



## Read:

# Carriage
ros2 topic echo /elmo/id1/carriage/position/get

# Lift
ros2 topic echo /elmo/id1/lift/position/get

# Joints
ros2 topic echo /joint_states



## Write:

# Move to home
ros2 action send_goal /execute_command smart_kitchen_interfaces/action/ExecuteCommand "{command_name: 'home'}"

# Move Carriage
ros2 topic pub --once /elmo/id1/carriage/position/set std_msgs/msg/Float32 "{data: 2.9}"
# EMERGENCY STOP Carriage
ros2 topic pub –once /elmo/id1/carriage/stop std_msgs/msg/Empty "{}"

# Move Lift
ros2 topic pub --once /elmo/id1/lift/position/set std_msgs/msg/Float32 "{data: 0.3}"
# EMERGENCY STOP Lift
ros2 topic pub –once /elmo/id1/lift/stop std_msgs/msg/Empty "{}"

# Move Joints
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{ joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [ { positions: [-0.0, 0.0,-1.0, 0.0,-0.6, 1.30], time_from_start: { sec: 5 } }, ]}"

# Open Gripper
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"

# Close Gripper
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.8, max_effort: 100.0}}"


# BASE_LINK FRAME IS Z TO THE BOTTOM, Y POSITIVE INTO THE WALL, X CARRIAGE MOVE POSITIVE zur spühle
# CAMERA FRAME IS INVERTED TO THE GRIPPER FRAME
# CAMERA FRAME: Z OUT, X TO THE RIGHT, Y DOWN
# GRIPPER FRAME: Z OUT, X TO THE LEFT, Y UP