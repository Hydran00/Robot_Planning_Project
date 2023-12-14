import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

rviz_config_dir = get_package_share_directory("planner") + "/rviz/map.rviz"


def generate_launch_description():
    show_graphics = True
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument("type", default_value="rrt_star"),
            Node(
                package="planner",
                executable="rrt_star",
                name="RRTSTAR",
                output="screen",
                parameters=[
                    {
                        "type": launch.substitutions.LaunchConfiguration("type"),
                        "show_graphics": show_graphics,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [""+str(show_graphics)]
                    )
                ),
                arguments=["-d", rviz_config_dir],
            ),
        ]
    )
