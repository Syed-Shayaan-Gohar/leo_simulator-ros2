# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Solar-elevation-derived light directions (SPICE-HL3 paper, Table/Fig 1):
#   x = cos(elevation) * cos(45deg), y = cos(elevation) * sin(45deg), z = -sin(elevation)
# Dawn/Dusk (4 deg) is the lunalab.sdf file's own default and is left alone here.
# Noon (8 deg) is applied live via the /world/lunalab/light_config service, since
# duplicating the whole world file risks the two versions drifting out of sync.
LIGHTING_PRESETS = {
    "noon": {
        "direction": "{x: 0.7003, y: 0.7003, z: -0.1392}",
    },
}


def apply_lighting(context: LaunchContext, lighting: LaunchConfiguration):
    lighting_name = context.perform_substitution(lighting)
    if lighting_name not in LIGHTING_PRESETS:
        return []

    direction = LIGHTING_PRESETS[lighting_name]["direction"]
    req = (
        f'name: "sun_light", type: DIRECTIONAL, direction: {direction}, '
        'diffuse: {r: 1.0, g: 0.953, b: 0.937, a: 1}, '
        'specular: {r: 0.3, g: 0.3, b: 0.3, a: 1}, '
        'cast_shadows: true, intensity: 1.0'
    )
    return [
        ExecuteProcess(
            cmd=[
                "ign", "service",
                "-s", "/world/lunalab/light_config",
                "--reqtype", "ignition.msgs.Light",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "2000",
                "--req", req,
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_gazebo = get_package_share_directory("leo_gz_bringup")
    pkg_project_worlds = get_package_share_directory("leo_gz_worlds")

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_project_worlds, "worlds", "leo_empty.sdf"),
        description="Path to the Gazebo world file",
    )

    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    lighting = DeclareLaunchArgument(
        "lighting",
        default_value="dawn_dusk",
        description="Lighting scenario: dawn_dusk (SDF default, 4 deg) | noon (8 deg)",
    )

    apply_lighting_action = TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=apply_lighting, args=[LaunchConfiguration("lighting")])],
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={"robot_ns": LaunchConfiguration("robot_ns")}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Force rendering onto the discrete NVIDIA GPU instead of the
    # Intel iGPU (default PRIME behavior on this hybrid-graphics laptop).
    # Without this, Gazebo's camera sensors silently render on the iGPU
    # and fall well short of their configured update_rate, worse as more
    # camera sensors run simultaneously.
    gpu_offload_1 = SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1")
    gpu_offload_2 = SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia")
    gpu_offload_3 = SetEnvironmentVariable("__VK_LAYER_NV_optimus", "NVIDIA_only")

    return LaunchDescription(
        [
            gpu_offload_1,
            gpu_offload_2,
            gpu_offload_3,
            sim_world,
            robot_ns,
            lighting,
            gz_sim,
            spawn_robot,
            topic_bridge,
            apply_lighting_action,
        ]
    )
