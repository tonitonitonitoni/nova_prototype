import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node


def generate_launch_description():

    pkg_project_bringup = get_package_share_directory('nova_bringup')
    pkg_project_gazebo = get_package_share_directory('nova_gazebo')
    pkg_project_description = get_package_share_directory('nova_description')

    default_rviz_config_path = os.path.join(pkg_project_bringup, 'config/urdf_config.rviz')

    world_path=os.path.join(pkg_project_gazebo, 'worlds/my_world.sdf')
    
    xacro_file = os.path.join(pkg_project_description,'urdf','nova_gps.urdf')
    robot_description_config = Command(['xacro ', xacro_file])
    
    params = {'robot_description': robot_description_config}
    rl_params = os.path.join(pkg_project_bringup, "config", "ekf_gps_nav2.yaml") #Dual kalman filter

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': True}]
    )

    spawn_entity = Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )

    rl_local_node = Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/local")],
    )
    
    
    rl_global_node = Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/global")]
    )

    navsat_transform_node = Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params, {"use_sim_time": True}],
                remappings=[
                    ("imu/data", "imu/data"),
                    ("gps/fix", "gps/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
            )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rl_local_node,
        rl_global_node,
        navsat_transform_node
    ])