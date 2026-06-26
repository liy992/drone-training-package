import pytest

from ego_training_demo.core import (
    EgoPositionCommand,
    FrameHome,
    coerce_bool,
    ego_topic_name,
    ensure_ros_overlay_sourced,
    px4_setpoint_components,
)


def test_frame_home_rotates_xy_and_inverts_z():
    frame_home = FrameHome(map_position=(0.0, 0.0, 1.0), px4_position=(10.0, 20.0, -1.0))

    assert frame_home.map_to_px4_position((2.0, 3.0, 2.5)) == [13.0, 22.0, -2.5]


def test_px4_setpoint_components_transform_position_velocity_and_acceleration():
    frame_home = FrameHome(map_position=(0.0, 0.0, 1.0), px4_position=(0.0, 0.0, -1.0))
    command = EgoPositionCommand(
        position=(4.0, -1.0, 1.5),
        velocity=(1.5, -0.5, 0.2),
        acceleration=(0.4, -0.1, 0.3),
        yaw=0.8,
        yaw_dot=0.1,
    )

    result = px4_setpoint_components(command, frame_home)

    assert result["position"] == [-1.0, 4.0, -1.5]
    assert result["velocity"] == [-0.5, 1.5, -0.2]
    assert result["acceleration"] == [-0.1, 0.4, -0.3]
    assert result["yaw"] != command.yaw
    assert result["yawspeed"] != command.yaw_dot


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (True, True),
        (False, False),
        ("true", True),
        ("1", True),
        ("yes", True),
        ("false", False),
        ("0", False),
        ("", False),
    ],
)
def test_coerce_bool_accepts_expected_training_values(value, expected):
    assert coerce_bool(value) is expected


def test_ego_topic_name_uses_drone_prefix():
    assert ego_topic_name("ego_cloud") == "/drone_0_ego_cloud"
    assert ego_topic_name("/ego_odom", drone_id=2) == "/drone_2_ego_odom"


def test_ensure_ros_overlay_sourced_errors_for_missing_workspace(tmp_path):
    missing = tmp_path / "missing_install"
    with pytest.raises(RuntimeError, match="ROS 2 overlay is missing"):
        ensure_ros_overlay_sourced(missing)
