from os.path import join
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    robot_start_pose_x = LaunchConfiguration("robot_start_pose_x", default="0.5")
    robot_start_pose_y = LaunchConfiguration("robot_start_pose_y", default="1.52")
    default_world = join(turtlebot3_gazebo_dir, "worlds", "nwt.world")

    # Start Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -s -v2 ", default_world], "on_exit_shutdown": "true"}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g -v2 "}.items()
    )

    # Start robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(turtlebot3_gazebo_dir, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    # Spawn the eurobot robot
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(turtlebot3_gazebo_dir, "launch", "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={
            "x_pose": robot_start_pose_x,
            "y_pose": robot_start_pose_y
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        join(turtlebot3_gazebo_dir, 'models'))

    return LaunchDescription([
        set_env_vars_resources,
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot_cmd,
        robot_state_publisher_cmd
    ])
