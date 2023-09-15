from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    package = "rrbot_description"

    # xacroからurdfの生成
    robot_description = Command(
        [
            PathJoinSubstitution(FindExecutable(name="xacro")),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package), "urdf", "model.urdf.xacro"]
            ),
        ]
    )

    # urdfを/tfと/robot_desctiptionで配信するノード
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Gazeboノードのlaunch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        )
    )

    # Gazeboで/robot_descriptionをsubscribeしてrrbotという名前のモデルをspawnさせる
    gazebo_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "rrbot", "-topic", "robot_description"],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            gazebo,
            gazebo_spawner,
        ]
    )
