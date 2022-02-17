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
                name="camera",
                parameters=[{
                    "camera_id": "0",
                    "info_path": "install/visual_driver/share/visual_driver/cfg/camera_info_640x480.yaml",
                    "show": True}],
                extra_arguments=[{"use_intra_process_comms": True}]
                )],
        output="screen",
    )

    return launch.LaunchDescription([container])