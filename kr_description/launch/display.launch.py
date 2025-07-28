from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    kr_description_pkg = FindPackageShare('kr_description')

    default_urdf_path = PathJoinSubstitution([kr_description_pkg, 'urdf', 'kr1018.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([kr_description_pkg, 'rviz', 'display.rviz'])

    ld.add_action(DeclareLaunchArgument(
        'model',
        default_value=default_urdf_path,
        description='Path to the robot URDF model'
    ))

    ld.add_action(DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    ))

    ld.add_action(DeclareLaunchArgument(
        'gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    ))

    ld.add_action(DeclareLaunchArgument(
        'base_link_name',
        default_value='base_link',
        description='base_link_name to add to all joint and link names'
    ))

    # Run xacro to generate the URDF XML string, passing base_link_name as a parameter
    robot_description = Command([
        'xacro ', LaunchConfiguration('model'),
        ' base_link_name:=', LaunchConfiguration('base_link_name')
    ])

    # Publish robot_state_publisher node with the robot_description parameter
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    ))

    # joint_state_publisher (CLI) if gui is false
    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    ))

    # joint_state_publisher_gui if gui is true
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    ))

    # RViz node
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    ))

    return ld
