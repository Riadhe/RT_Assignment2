import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# Function to generate launch description
def generate_launch_description():
    # 1. prepare the path of package 'bme_gazebo_sensors'
    gazebo_pkg_path = get_package_share_directory('bme_gazebo_sensors')
    
    # 2. Include the simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_path, 'launch', 'spawn_robot.launch.py')
        )
    )

    # 3. Define the robot controller node with parameters
    controller_node = Node(
        package='assignment2_rt',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        emulate_tty=True,
        parameters=[{'safety_threshold': 1.0}]
    )

    return LaunchDescription([
        simulation_launch,  # Include simulation launch
        controller_node     # Include controller node
    ])