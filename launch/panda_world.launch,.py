import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper', default_value='true')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('pick_and_place'),
                                   'worlds', 'pick_and_place.world'),
        description='Path to the Gazebo world file')

    robot_description_path = os.path.join(
        get_package_share_directory('franka_description'), 
        'robots', 'panda_arm_hand.urdf.xacro')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(robot_description_path).read()}],
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'panda'],
        output='screen'
    )

    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_link0',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base', 'panda_link0']
    )
    
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'base']
    )

    object_detector_node = Node(
        package='pick_and_place',
        executable='object_detector',
        name='vision_object_detector',
        output='screen'
    )

    state_machine_node = Node(
        package='pick_and_place',
        executable='state_machine',
        name='pick_and_place_state_machine',
        output='screen'
    )

    return LaunchDescription([
        load_gripper_arg,
        world_arg,
        gazebo,
        spawn_entity,
        static_tf_base,
        static_tf_world,
        object_detector_node,
        state_machine_node,
        #Mrinal, Add your MoveIt2 Launch `includes` and Other Nodes Here
    ])
