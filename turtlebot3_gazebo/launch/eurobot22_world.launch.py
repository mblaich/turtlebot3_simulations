from os.path import join
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(turtlebot3_gazebo_dir, 'models')

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    robot_start_pose_x = LaunchConfiguration("robot_start_pose_x", default="0.5")
    robot_start_pose_y = LaunchConfiguration("robot_start_pose_y", default="0.5")
    default_world = join(turtlebot3_gazebo_dir, "worlds", "eurobot22.world")

    # Start Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": default_world}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
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

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot_cmd,
        robot_state_publisher_cmd
    ])
