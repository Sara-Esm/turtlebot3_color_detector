from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import tempfile
from ament_index_python.packages import get_package_share_directory

RED_MARKER_SDF = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="red_target">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><sphere><radius>0.15</radius></sphere></geometry>
        <material>
          <ambient>1.0 0.0 0.0 1</ambient>
          <diffuse>1.0 0.0 0.0 1</diffuse>
          <specular>0.5 0.0 0.0 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry><sphere><radius>0.15</radius></sphere></geometry>
      </collision>
    </link>
  </model>
</sdf>"""

_sdf_file = '/tmp/red_target.sdf'
with open(_sdf_file, 'w') as f:
    f.write(RED_MARKER_SDF)

def generate_launch_description():

    target_color_arg = DeclareLaunchArgument(
        "target_color",
        default_value="red",
        description="Color to track: red | green | blue | yellow"
    )

    show_debug_arg = DeclareLaunchArgument(
        "show_debug",
        default_value="true",
        description="Show OpenCV debug windows"
    )

    target_color = LaunchConfiguration("target_color")
    show_debug   = LaunchConfiguration("show_debug")

    pkg_share   = get_package_share_directory("turtlebot3_color_follower")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    spawn_target = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "gazebo_ros", "spawn_entity.py",
                    "-entity", "red_target",
                    "-file", _sdf_file,
                    "-x", "-4.148730",
                    "-y", "5.000360",
                    "-z", "0.1",
                ],
                output="screen"
            )
        ]
    )

    color_detector_node = Node(
        package="turtlebot3_color_follower",
        executable="color_detector",
        name="color_detector_node",
        output="screen",
        parameters=[
            params_file,
            {
                "target_color":      target_color,
                "show_debug_window": show_debug,
            }
        ]
    )

    smc_controller_node = Node(
        package="turtlebot3_color_follower",
        executable="smc_controller",
        name="smc_motion_controller_node",
        output="screen",
        parameters=[params_file]
    )

    return LaunchDescription([
        target_color_arg,
        show_debug_arg,
        color_detector_node,
        smc_controller_node,
        spawn_target,
    ])
