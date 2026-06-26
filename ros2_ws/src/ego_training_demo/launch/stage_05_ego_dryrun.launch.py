import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    drone_id = LaunchConfiguration("drone_id")
    frame_id = LaunchConfiguration("frame_id")
    odom_topic = LaunchConfiguration("odom_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    map_size_x = LaunchConfiguration("map_size_x")
    map_size_y = LaunchConfiguration("map_size_y")
    map_size_z = LaunchConfiguration("map_size_z")
    local_update_range_z = LaunchConfiguration("local_update_range_z")
    virtual_ceil_height = LaunchConfiguration("virtual_ceil_height")
    visualization_truncate_height = LaunchConfiguration("visualization_truncate_height")
    obstacles_inflation = LaunchConfiguration("obstacles_inflation")
    max_vel = LaunchConfiguration("max_vel")
    max_acc = LaunchConfiguration("max_acc")
    planning_horizon = LaunchConfiguration("planning_horizon")
    accumulate_cloud = LaunchConfiguration("accumulate_cloud")
    accumulation_window_s = LaunchConfiguration("accumulation_window_s")
    max_accumulated_frames = LaunchConfiguration("max_accumulated_frames")
    self_filter_min_range = LaunchConfiguration("self_filter_min_range")
    self_filter_xy_radius = LaunchConfiguration("self_filter_xy_radius")
    self_filter_z_radius = LaunchConfiguration("self_filter_z_radius")
    send_default_goal = LaunchConfiguration("send_default_goal")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")
    goal_delay_s = LaunchConfiguration("goal_delay_s")

    ego_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ego_planner"),
                "launch",
                "advanced_param.launch.py",
            )
        ),
        launch_arguments={
            "drone_id": drone_id,
            "map_size_x_": map_size_x,
            "map_size_y_": map_size_y,
            "map_size_z_": map_size_z,
            "local_update_range_z": local_update_range_z,
            "virtual_ceil_height": virtual_ceil_height,
            "visualization_truncate_height": visualization_truncate_height,
            "obstacles_inflation": obstacles_inflation,
            "odometry_topic": odom_topic,
            "cloud_topic": cloud_topic,
            "frame_id": frame_id,
            "flight_type": "1",
            "point_num": "1",
            "max_vel": max_vel,
            "max_acc": max_acc,
            "planning_horizon": planning_horizon,
            "use_distinctive_trajs": "True",
        }.items(),
    )

    traj_server = Node(
        package="ego_planner",
        executable="traj_server",
        name="ego_training_traj_server",
        output="screen",
        remappings=[("planning/bspline", ["drone_", drone_id, "_planning/bspline"])],
        parameters=[
            {"traj_server/time_forward": 1.0},
            {"traj_server/command_frame_id": frame_id},
            {"use_sim_time": True},
        ],
    )

    cloud_bridge = Node(
        package="ego_training_demo",
        executable="cloud_bridge",
        name="ego_training_cloud_bridge",
        output="screen",
        parameters=[
            {"input_topic": "/training/lidar/pointcloud"},
            {"output_topic": ["/drone_", drone_id, "_", cloud_topic]},
            {"target_frame": frame_id},
            {"accumulate_cloud": accumulate_cloud},
            {"accumulation_window_s": accumulation_window_s},
            {"max_accumulated_frames": max_accumulated_frames},
            {"self_filter_min_range": self_filter_min_range},
            {"self_filter_xy_radius": self_filter_xy_radius},
            {"self_filter_z_radius": self_filter_z_radius},
            {"use_sim_time": True},
        ],
    )

    monitor = Node(
        package="ego_training_demo",
        executable="position_cmd_monitor",
        name="ego_training_position_cmd_monitor",
        output="screen",
        parameters=[
            {"position_cmd_topic": "/position_cmd"},
            {"expected_frame_id": frame_id},
            {"use_sim_time": True},
        ],
    )

    goal_sender = Node(
        package="ego_training_demo",
        executable="goal_sender",
        name="ego_training_goal_sender",
        condition=IfCondition(send_default_goal),
        output="screen",
        parameters=[
            {"goal_topic": "/move_base_simple/goal"},
            {"frame_id": frame_id},
            {"x": goal_x},
            {"y": goal_y},
            {"z": goal_z},
            {"delay_s": goal_delay_s},
            {"repeat_count": 3},
            {"repeat_interval_s": 0.3},
            {"use_sim_time": True},
        ],
    )

    group = GroupAction(
        [
            SetParameter(name="use_sim_time", value=True),
            ego_launch,
            traj_server,
            cloud_bridge,
            monitor,
            goal_sender,
        ]
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
            group,
        ]
    )
