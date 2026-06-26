from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_arm = LaunchConfiguration("auto_arm")
    auto_offboard = LaunchConfiguration("auto_offboard")
    odom_topic = LaunchConfiguration("odom_topic")

    dryrun_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ego_training_demo"), "launch", "stage_05_ego_dryrun.launch.py"]
            )
        ),
        launch_arguments={
            "drone_id": LaunchConfiguration("drone_id"),
            "frame_id": LaunchConfiguration("frame_id"),
            "odom_topic": odom_topic,
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "map_size_x": LaunchConfiguration("map_size_x"),
            "map_size_y": LaunchConfiguration("map_size_y"),
            "map_size_z": LaunchConfiguration("map_size_z"),
            "local_update_range_z": LaunchConfiguration("local_update_range_z"),
            "virtual_ceil_height": LaunchConfiguration("virtual_ceil_height"),
            "visualization_truncate_height": LaunchConfiguration("visualization_truncate_height"),
            "obstacles_inflation": LaunchConfiguration("obstacles_inflation"),
            "max_vel": LaunchConfiguration("max_vel"),
            "max_acc": LaunchConfiguration("max_acc"),
            "planning_horizon": LaunchConfiguration("planning_horizon"),
            "accumulate_cloud": LaunchConfiguration("accumulate_cloud"),
            "accumulation_window_s": LaunchConfiguration("accumulation_window_s"),
            "max_accumulated_frames": LaunchConfiguration("max_accumulated_frames"),
            "self_filter_min_range": LaunchConfiguration("self_filter_min_range"),
            "self_filter_xy_radius": LaunchConfiguration("self_filter_xy_radius"),
            "self_filter_z_radius": LaunchConfiguration("self_filter_z_radius"),
            "send_default_goal": LaunchConfiguration("send_default_goal"),
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "goal_z": LaunchConfiguration("goal_z"),
            "goal_delay_s": LaunchConfiguration("goal_delay_s"),
        }.items(),
    )

    px4_bridge = Node(
        package="ego_training_demo",
        executable="ego_px4_bridge",
        name="ego_training_px4_bridge",
        output="screen",
        parameters=[
            {"position_cmd_topic": "/position_cmd"},
            {"odom_topic": ["/drone_", LaunchConfiguration("drone_id"), "_", odom_topic]},
            {"auto_arm": auto_arm},
            {"auto_offboard": auto_offboard},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("drone_id", default_value="0"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("odom_topic", default_value="ego_odom"),
            DeclareLaunchArgument("cloud_topic", default_value="ego_cloud"),
            DeclareLaunchArgument("map_size_x", default_value="12.0"),
            DeclareLaunchArgument("map_size_y", default_value="12.0"),
            DeclareLaunchArgument("map_size_z", default_value="4.0"),
            DeclareLaunchArgument("local_update_range_z", default_value="4.0"),
            DeclareLaunchArgument("virtual_ceil_height", default_value="3.0"),
            DeclareLaunchArgument("visualization_truncate_height", default_value="3.0"),
            DeclareLaunchArgument("obstacles_inflation", default_value="0.25"),
            DeclareLaunchArgument("max_vel", default_value="1.0"),
            DeclareLaunchArgument("max_acc", default_value="2.0"),
            DeclareLaunchArgument("planning_horizon", default_value="6.0"),
            DeclareLaunchArgument("accumulate_cloud", default_value="true"),
            DeclareLaunchArgument("accumulation_window_s", default_value="0.12"),
            DeclareLaunchArgument("max_accumulated_frames", default_value="8"),
            DeclareLaunchArgument("self_filter_min_range", default_value="0.45"),
            DeclareLaunchArgument("self_filter_xy_radius", default_value="0.45"),
            DeclareLaunchArgument("self_filter_z_radius", default_value="0.75"),
            DeclareLaunchArgument("send_default_goal", default_value="false"),
            DeclareLaunchArgument("goal_x", default_value="6.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_z", default_value="1.0"),
            DeclareLaunchArgument("goal_delay_s", default_value="5.0"),
            DeclareLaunchArgument("auto_arm", default_value="true"),
            DeclareLaunchArgument("auto_offboard", default_value="true"),
            dryrun_launch,
            px4_bridge,
        ]
    )
