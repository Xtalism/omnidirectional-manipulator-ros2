import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    ld: LaunchDescription = LaunchDescription()
    
    name_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name='name',
        default_value='my_publisher',
        description='String publisher name'
    )
    ld.add_action(name_arg)
    
    publisher_node: Node = Node(
        package='dm',
        executable='my_publisher',
        name=LaunchConfiguration('name'),
        output='screen'
    )
    ld.add_action(publisher_node)
    
    my_second_publisher_node: Node = Node(
        package='dm',
        executable='my_second_publisher',
        name='my_second_publisher',
        output='screen'
    )
    ld.add_action(my_second_publisher_node)
        
    return ld