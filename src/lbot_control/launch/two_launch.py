import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
#    return LaunchDescription([
#       Node(
#           package='demo_nodes_cpp',
#           executable='talker',
#           name='talker'),
# ])

def generate_launch_description():
    # Define package directories using FindPackageShare
    pkg_turtlebot3_gazebo = FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')
    pkg_rtabmap_ros = FindPackageShare(package='rtabmap_launch').find('rtabmap_launch')
    pkg_nav2_bringup = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
       
    # Define the path to the turtlebot3_world.launch.py file using PathJoinSubstitution
    turtlebot3_world_launch_file = PathJoinSubstitution([pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py'])
    rtabmap_launch_file = PathJoinSubstitution([pkg_rtabmap_ros, 'launch', 'rtabmap.launch.py'])
    navigation_launch_file = PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    rviz_launch_file = PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'rviz_launch.py'])

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        LogInfo(msg=f"Launching Navigation with file: {navigation_launch_file}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time
                }.items(),
        ),

        LogInfo(msg=f"Launching TurtleBot3 world with file: {turtlebot3_world_launch_file}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_world_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time
                }.items(),
        ),
        
        LogInfo(msg=f"Launching RTAB-SLAM using launch file: {rtabmap_launch_file}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments={
                'visual_odometry': 'false',
                'frame_id': 'base_footprint',
                'subscribe_scan': 'true',
                'depth': 'false',
                'approx_sync': 'true',
                'odom_topic': '/odom',
                'scan_topic': '/scan',
                'qos': '2',
                'args': '-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1',
                'use_sim_time': 'true',
                'rviz': 'true',
                }.items(),
        ),

        LogInfo(msg=f"Launching RViz with file: {rviz_launch_file}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_file),
        )
    ])
    
        
