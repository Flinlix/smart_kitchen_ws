# Useful Commands

# Move Carriage
ros2 topic pub --once /elmo/id1/carriage/position/set std_msgs/msg/Float32 "{data: 1.0}"

# Move Lift
ros2 topic pub --once /elmo/id1/lift/position/set std_msgs/msg/Float32 "{data: 0.25}"

# Open Gripper
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"

# Close Gripper
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.8, max_effort: 100.0}}"

# Move Joints
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{ joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [ { positions: [-0.0, 0.0,-1.0, 0.0,-0.6, 1.30], time_from_start: { sec: 5 } }, ]}"