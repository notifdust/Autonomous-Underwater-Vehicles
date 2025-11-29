import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_share = get_package_share_directory('auv_stack')
    
    # 1. Start Gazebo
    gazebo_world_file = os.path.join(pkg_share, 'worlds', 'underwater.world')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': gazebo_world_file, 'use_sim_time': 'true'}.items()
    )

    # 2. Get the AUV's SDF description
    sdf_file_path = os.path.join(pkg_share, 'sdf', 'auv.sdf')
    with open(sdf_file_path, 'r') as f:
        robot_description = f.read()

    # 3. Spawn the AUV into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_auv', '-z', '-1.0'],
        output='screen'
    )
    
    # 4. Publish the robot description (so RViz can see it)
    robot_state_publisher = Node(
        package='robot_state_ publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 5. Start all your control nodes
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'), {'use_sim_time': True}]
    )

    thruster_allocator = Node(
        package='auv_stack',
        executable='thruster_allocator',
        name='thruster_allocator',
        parameters=[{'use_sim_time': True}]
    )
    
    mpc_controller = Node(
        package='auv_stack',
        executable='mpc_controller',
        name='mpc_controller',
        parameters=[{'use_sim_time': True}]
    )
    
    mission_planner = Node(
        package='auv_stack',
        executable='mission_planner',
        name='mission_planner',
        parameters=[{'use_sim_time': True}]
    )
    
    obstacle_avoidance = Node(
        package='auv_stack',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity,
        robot_state_publisher,
        ekf_node,
        thruster_allocator,
        mpc_controller,
        mission_planner,
        obstacle_avoidance
    ])