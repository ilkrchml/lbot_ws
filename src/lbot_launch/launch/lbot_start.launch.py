import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    launch_file_dir_nav = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')
    launch_file_dir_rtap = os.path.join(get_package_share_directory('rtabmap_demos'), 'launch')
    #print('launch file dir', launch_file_dir_nav)

    #colors = {
    #    'background_r': '200'
    #}

    lbot_cmd =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lbot_control'),
                    'launch',
                    'lbot_control.launch.py'
                ])
            ])#,
            #launch_arguments={
            #    'turtlesim_ns': 'turtlesim2',
            #    'use_provided_red': 'True',
             #   'new_background_r': TextSubstitution(text=str(colors['background_r']))
            #}.items()
    )

    tb3_world_cmd  = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'turtlebot3_world.launch.py')
        )
    )
    tb3_nav_cmd =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir_nav, 'navigation2.launch.py')
        )
    )
    tb3_rtap_cmd =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir_rtap, 'turtlebot3_rgbd.launch.py')
        )
    )
    lbot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir_lbot, 'lbot_control.launch.py')
        )
    )
    ld = LaunchDescription()
    ld.add_action(tb3_world_cmd)
    #ld.add_action(tb3_rtap_cmd)
    #ld.add_action(lbot_cmd)
    ld.add_action(tb3_nav_cmd)
    ld.add_action(tb3_rtap_cmd)
    return ld