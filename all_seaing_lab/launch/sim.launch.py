from launch import LaunchDescription
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration


def generate_launch_description():
    sim_node = Node(
        package="all_seaing_lab",
        executable="sim_engine.py"
    )

    teleop_controller_node = Node(
        package="all_seaing_lab",
        executable="teleop_controller.py"
    )

    waypoint_follower_node = Node(
        package="all_seaing_lab",
        executable="waypoint_follower.py"
    )

    task_node = Node(
        package="all_seaing_lab",
        executable="follow_path.py"
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
    buoy_course = Node(
        package="all_seaing_lab",
        executable="buoy_course.py",
        parameters=[PathJoinSubstitution([
            FindPackageShare('all_seaing_lab'), 'config', course_name])
        ],
        condition=IfCondition(
            PythonExpression(['"', course_name, '" != "none"'])
        ),
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

    control_mux = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="control_mux.py",
    )

    controller_server = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="controller_server.py",
        parameters=[
            {"global_frame_id": "map"},
            {"robot_frame_id": "base_link"},
            {"Kpid_x": [1.0, 0.0, 0.0]},
            {"Kpid_y": [1.0, 0.0, 0.0]},
            {"Kpid_theta": [1.0, 0.0, 0.0]},
            {"max_vel": [0.7, 0.3, 1.5]},
        ],
        output="screen",
    )

    navigation_server = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="navigation_server.py",
        parameters=[
            {"global_frame_id": "map"},
            {"robot_frame_id": "base_link"},
        ],remappings=[
            ("/dynamic_map", "/world_map")
        ],
        output="screen",
    )

    run_tasks = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="run_tasks.py",
    )

    task_init_server = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="task_init.py",
        parameters=[{"is_sim": True}],
    )

    follow_buoy_path = launch_ros.actions.Node(
        package="all_seaing_lab",
        # executable="follow_path_local", # TOGGLE IN FOR LOCAL MAP PART
        executable="follow_path_global", # TOGGLE IN FOR GLOBAL MAP PART
        parameters=[
            # ADD PARAMETER VALUES TO BE TUNED
        ]
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
            buoy_course,
            control_mux,
            controller_server,
            navigation_server,
            run_tasks,
            task_init_server,
            follow_buoy_path,
        ]
    )