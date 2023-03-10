from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    firstPackage = LaunchConfiguration('firstPackage')
    port = LaunchConfiguration('port')
    
    firstPackage_launch_arg = DeclareLaunchArgument(
        'firstPackage',
        default_value='kansei'
    )
    port_launch_arg = DeclareLaunchArgument(
        'port',
        default_value=['/dev/ttyUSB1'],
        description='choose port name if /dev/ttyUSB0 not working!'
    )
    
    firstNode = Node(
        package=firstPackage,
        executable='main',
        name='kansei',
        arguments=[port],
        output='screen'
    )

    return LaunchDescription([
        firstPackage_launch_arg,
        port_launch_arg,
        firstNode
    ])
