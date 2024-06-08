import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir_nav = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')
    ld = LaunchDescription([
        Node(
            package='lbot_control',
            #namespace='turtlesim1',
            executable='talker',
            name='talker'
        ),
        Node(
            package='lbot_control',
            #namespace='turtlesim2',
            executable='listener',
            name='listener'
        )#,
       # Node(
         #   package='turtlesim',
         #   executable='mimic',
         #   name='mimic',
          #  remappings=[
          #      ('/input/pose', '/turtlesim1/turtle1/pose'),
           #     ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
          #  ]
       # )
    ])
    tb3_nav_cmd =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir_nav, 'navigation2.launch.py')
        )#,
        #launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    #ld.add_action(tb3_nav_cmd)
    return ld