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
                package="face_detect",
                plugin="visual_composition::FaceDetect",
                name="face_detect_component",
                parameters=[{
                    "mnn_path": "install/face_detect/share/face_detect/model/slim-320.mnn",
                    "width": 320,
                    "height": 240,
                    "score_thresh": 0.6,
                    "nms_thresh": 0.3,
                    "num_threads": 2,
                    "draw": True,
                }],
                extra_arguments=[{"use_intra_process_comms": True}]
                
            )
        ],
        output="screen",
    )

    return launch.LaunchDescription([container])