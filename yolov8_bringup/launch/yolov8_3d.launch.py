# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node





# remove the 2D detection and tracking node due to dependency of the 3D node on map frame while the 2D detection opearte in camera frame.
# No need to run two times to avoid node conflicts


def generate_launch_description():

    #
    # ARGS
    #

    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value="yolov8n.pt",                                            # use nano model to reduce the compuation load
        description="Model name or path")

    tracker = LaunchConfiguration("tracker")
    tracker_cmd = DeclareLaunchArgument(
        "tracker",
        default_value="bytetrack.yaml",
        description="Tracker name or path")

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Device to use (GPU/CPU)")

    enable = LaunchConfiguration("enable")
    enable_cmd = DeclareLaunchArgument(
        "enable",
        default_value="True",
        description="Whether to start YOLOv8 enabled")

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument(
        "threshold",
        default_value="0.5",
        description="Minimum probability of a detection to be published")

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/camera/color/image_raw",
        description="Name of the input image topic")

    image_reliability = LaunchConfiguration("image_reliability")
    image_reliability_cmd = DeclareLaunchArgument(
        "image_reliability",
        default_value="1",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)")

    input_depth_topic = LaunchConfiguration("input_depth_topic")
    input_depth_topic_cmd = DeclareLaunchArgument(
        "input_depth_topic",
        default_value="/depth",
        description="Name of the input depth topic")

    depth_image_reliability = LaunchConfiguration("depth_image_reliability")
    depth_image_reliability_cmd = DeclareLaunchArgument(
        "depth_image_reliability",
        default_value="1",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input depth image topic (0=system default, 1=Reliable, 2=Best Effort)")

    input_depth_info_topic = LaunchConfiguration("input_depth_info_topic")
    input_depth_info_topic_cmd = DeclareLaunchArgument(
        "input_depth_info_topic",
        default_value="/depth_camera_info",
        description="Name of the input depth info topic")

    depth_info_reliability = LaunchConfiguration("depth_info_reliability")
    depth_info_reliability_cmd = DeclareLaunchArgument(
        "depth_info_reliability",
        default_value="1",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input depth info topic (0=system default, 1=Reliable, 2=Best Effort)")

    depth_image_units_divisor = LaunchConfiguration(
        "depth_image_units_divisor")
    depth_image_units_divisor_cmd = DeclareLaunchArgument(
        "depth_image_units_divisor",
        default_value="1000",
        description="Divisor used to convert the raw depth image values into metres")

    target_frame = LaunchConfiguration("target_frame")
    target_frame_cmd = DeclareLaunchArgument(
        "target_frame",
        default_value="map",
        description="Target frame to transform the 3D boxes")

    maximum_detection_threshold = LaunchConfiguration(
        "maximum_detection_threshold")
    maximum_detection_threshold_cmd = DeclareLaunchArgument(
        "maximum_detection_threshold",
        default_value="0.3",
        description="Maximum detection threshold in the z axis")

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="yolo",
        description="Namespace for the nodes")

   
    detect_3d_node_cmd = Node(
        package="yolov8_ros",
        executable="detect_3d_node",
        name="detect_3d_node",
        namespace=namespace,
        parameters=[{
            "target_frame": target_frame,
            "maximum_detection_threshold": maximum_detection_threshold,
            "depth_image_units_divisor": depth_image_units_divisor,
            "depth_image_reliability": depth_image_reliability,
            "depth_info_reliability": depth_info_reliability
        }],
        remappings=[
            ("depth_image", input_depth_topic),
            ("depth_info", input_depth_info_topic),
            ("detections", "tracking")
        ]
    )


    # the velocity estimation node is integrated with my following discussion, suggestions and debugging as mentioned here
    # https://github.com/mgonzs13/yolov8_ros/issues/18 

    speed_stimation_node_cmd = Node(
        package="yolov8_ros",
        executable="speed_stimation_node",
        name="speed_stimation_node",
        namespace=namespace,
    )

    debug_node_cmd = Node(
        package="yolov8_ros",
        executable="debug_node",
        name="debug_node",
        namespace=namespace,
        parameters=[{"image_reliability": image_reliability}],
        remappings=[
            ("image_raw", input_image_topic),
            ("detections", "detections_speed")
        ]
    )

    ld = LaunchDescription()

    ld.add_action(device_cmd)
    ld.add_action(enable_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(image_reliability_cmd)
    ld.add_action(input_depth_topic_cmd)
    ld.add_action(depth_image_reliability_cmd)
    ld.add_action(input_depth_info_topic_cmd)
    ld.add_action(depth_info_reliability_cmd)
    ld.add_action(depth_image_units_divisor_cmd)
    ld.add_action(target_frame_cmd)
    ld.add_action(maximum_detection_threshold_cmd)
    ld.add_action(namespace_cmd)
    ld.add_action(detect_3d_node_cmd)
    ld.add_action(speed_stimation_node_cmd)
    ld.add_action(debug_node_cmd)

    return ld
