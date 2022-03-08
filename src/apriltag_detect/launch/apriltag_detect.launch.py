import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="visual_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="visual_driver",
                plugin="visual_composition::Camera",
                name="Camera",
                parameters=[{
                    "pub_info_name": "camera_info",
                    "pub_stream_name": "camera_stream",
                    "camera_id": "0",
                    "info_path": "install/visual_driver/share/visual_driver/cfg/camera_info_640x480.yaml",
                    "show": False
                }],
                extra_arguments=[{"use_intra_process_comms": True}]
            ),
            ComposableNode(
                package="apriltag_detect",
                plugin="visual_composition::ApriltagDetect",
                name="apriltag_detect_component",
                parameters=[{
                    "tag_family": "CUSTOM48h12",
                    "decimate": 2.0,
                    "sigma": 0.0,
                    "threads": 2,
                    "show": True,
                }],
                extra_arguments=[{"use_intra_process_comms": True}]
            )
        ],
        output="screen",
    )

    return launch.LaunchDescription([container])