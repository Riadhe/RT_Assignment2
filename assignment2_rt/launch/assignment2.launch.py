import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Nlawjou win ja el package 'bme_gazebo_sensors'
    gazebo_pkg_path = get_package_share_directory('bme_gazebo_sensors')
    
    # 2. N7adhrou l-inculsion mte3 'spawn_robot.launch.py'
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_path, 'launch', 'spawn_robot.launch.py')
        )
    )

    # 3. Node Robot Controller (Mte3na)
    controller_node = Node(
        package='assignment2_rt',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        emulate_tty=True,
        parameters=[{'safety_threshold': 1.0}]
    )

    return LaunchDescription([
        simulation_launch,  # <-- Hathi tdemari Gazebo
        controller_node     # <-- Hathi tdemari codek
    ])