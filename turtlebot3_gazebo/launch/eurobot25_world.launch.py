from os.path import join
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    os.environ["TURTLEBOT3_MODEL"] = 'burger_tg15_lidar'

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    robot_start_pose_x = LaunchConfiguration("robot_start_pose_x", default="1.25")
    robot_start_pose_y = LaunchConfiguration("robot_start_pose_y", default="0.2")
    robot_start_pose_yaw = LaunchConfiguration("robot_start_pose_yaw", default="0.0")
    default_world = join(turtlebot3_gazebo_dir, "worlds", "eurobot25.world")

    # Start Gazebo server and client
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": f"-r {default_world}"
        }.items()
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
            "y_pose": robot_start_pose_y,
            "yaw_pose": robot_start_pose_yaw
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        join(turtlebot3_gazebo_dir, 'models'))

    return LaunchDescription([
        set_env_vars_resources,
        gz_sim_launch,
        spawn_robot_cmd,
        robot_state_publisher_cmd
    ])
