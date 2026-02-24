from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # >>> HIER anpassen <<<
    cam_base_topic = "camera/image_raw"   # ohne führenden Slash, wie im README-Beispiel
    marker_size = 0.15                    # Meter (Marker side size!)
    image_is_rectified = False            # True wenn dein Topic image_rect ist

    # Marker-ID, die am Menschen klebt
    marker_id = 0

    return LaunchDescription([
        # ArUco Tracker
        Node(
            package="aruco_opencv",
            executable="aruco_tracker_autostart",
            name="aruco_opencv",
            output="screen",
            parameters=[{
                "cam_base_topic": cam_base_topic,
                "marker_size": marker_size,
                "image_is_rectified": image_is_rectified,
            }],
        ),

        # Human pose node
        Node(
            package="aruco_human_tracker",
            executable="human_pose_from_aruco",
            name="human_pose_from_aruco",
            output="screen",
            parameters=[{
                "detections_topic": "/aruco_detections",
                "marker_id": marker_id,
                "target_frame": "base_link",
                "output_topic": "/human_pose",
            }],
        ),
    ])