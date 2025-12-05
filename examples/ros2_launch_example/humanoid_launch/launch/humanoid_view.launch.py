import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    humanoid_launch_dir = get_package_share_directory('humanoid_launch')
    
    # Path to your URDF/Xacro file
    # Assuming humanoid_macro_example.xacro is in the examples folder relative to root
    # For a real package, you'd place it in a 'urdf' folder within the package
    # and use os.path.join(get_package_share_directory('humanoid_launch'), 'urdf', 'humanoid_macro_example.xacro')
    # For this example, we reference the one generated in examples/
    xacro_file_path = os.path.join(
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')),
        'examples',
        'humanoid_macro_example.xacro'
    )
    
    # Process the URDF/Xacro file
    robot_description_content = Command(['xacro ', xacro_file_path])
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # RViz2 Node
    # Path to RViz config file (optional)
    # rviz_config_file = os.path.join(
    #     get_package_share_directory('humanoid_launch'),
    #     'rviz',
    #     'humanoid_config.rviz'
    # )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', rviz_config_file] # Uncomment if you have a specific rviz config
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
