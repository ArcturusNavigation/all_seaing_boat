# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    sim_node = Node(
        package="all_seaing_lab",
        executable="sim_engine"
    )

    teleop_controller_node = Node(
        package="all_seaing_lab",
        executable="teleop_controller"
    )

    waypoint_follower_node = Node(
        package="all_seaing_lab",
        executable="waypoint_follower"
    )

    task_node = Node(
        package="all_seaing_lab",
        executable="follow_path"
    )

    keyboard_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("all_seaing_lab"),
                "launch",
                "keyboard.launch.py"
            ])
        ]),
    )

    course_name = LaunchConfiguration("course")
    launch_course = "false" if course_name == "none" else "true"
    buoy_course = Node(
        package="all_seaing_lab",
        executable="buoy_course",
        parameters=[PathJoinSubstitution([
            FindPackageShare('all_seaing_lab'), 'config', course_name])
        ],
        condition=IfCondition(launch_course),
    )

    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("all_seaing_lab"), "config", "dashboard.rviz"]),
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "course", default_value="none", description='file name of the course to be used in sim (.yaml)'
            ),
            rviz_node,
            sim_node,
            teleop_controller_node,
            waypoint_follower_node,
            task_node,
            keyboard_ld,
            buoy_course
        ]
    )