import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('auv_stack')
    
    # 1. Gazebo World and Model Definitions
    world_file = os.path.join(pkg_dir, 'worlds', 'underwater.world')
    auv_sdf_file = os.path.join(pkg_dir, 'sdf', 'auv.sdf')
    ekf_config_file = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    # 2. Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. Gazebo Launch (ROS Ign Gazebo bridge)
    # We use the 'ros_ign_gazebo' package to launch the simulation.
    ign_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': '-r -s ' + world_file}.items()
    )

    # 4. Spawn the AUV into Gazebo
    # Uses the 'create' service on the ROS-Ign bridge
    spawn_auv = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-file', auv_sdf_file,
            '-name', 'auv_robot',
            '-x', '0', 
            '-y', '0', 
            '-z', '0' # Start at the surface (z=0)
        ],
    )
    
    # 5. Robot State Publisher (REQUIRED for TF frames and RViz visualization)
    # This reads the AUV SDF and publishes the relationships between links.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['cat ', auv_sdf_file]) # Reads the SDF content
        }],
    )

    # 6. Sensor Fusion (robot_localization EKF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        # Remap sensor topics from the Gazebo bridge
        remappings=[
            ('/imu/data', '/imu_data'),
            ('/odometry/filtered', '/odometry/filtered'),
        ]
    )

    # 7. Autonomy Nodes (The python executables)
    mission_planner_node = Node(
        package='auv_stack',
        executable='mission_planner',
        name='mission_planner',
        output='screen'
    )
    
    thruster_allocator_node = Node(
        package='auv_stack',
        executable='thruster_allocator',
        name='thruster_allocator',
        output='screen'
    )

    # NOTE: mpc_controller and obstacle_avoidance are omitted here for brevity, 
    # but would be added using the same Node structure.

    return LaunchDescription([
        # Start Gazebo
        ign_gazebo_launch,
        
        # Start the AUV model, TF frames, and EKF
        spawn_auv,
        robot_state_publisher,
        ekf_node,

        # Start Autonomy Stack
        mission_planner_node,
        thruster_allocator_node,
        # Add mpc_controller_node, obstacle_avoidance_node here
    ])