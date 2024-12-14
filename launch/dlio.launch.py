#
#   Copyright (c)     
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition   
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='livox/lidar',
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='livox/imu',
        description='IMU topic name'
    )
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='RR01',
        description='Node namespace'
    )

    # Opaque function to launch nodes
    def launch_setup(context, *args, **kwargs):

        # Set default arguments
        rviz = LaunchConfiguration('rviz').perform(context)
        pointcloud_topic = LaunchConfiguration('pointcloud_topic').perform(context)
        imu_topic = LaunchConfiguration('imu_topic').perform(context)
        namespace = LaunchConfiguration('namespace').perform(context)

        # Load parameters
        current_pkg = FindPackageShare('direct_lidar_inertial_odometry')
        dlio_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'dlio.yaml'])
        dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml'])

        # DLIO Odometry Node
        dlio_odom_node = Node(
            package='direct_lidar_inertial_odometry',
            executable='dlio_odom_node',
            output='screen',
            namespace=namespace,
            parameters=[dlio_yaml_path, dlio_params_yaml_path],
            remappings=[
                ('pointcloud', pointcloud_topic),
                ('imu', imu_topic),
                ('odom', 'dlio/odom_node/odom'),
                ('pose', 'dlio/odom_node/pose'),
                ('path', 'dlio/odom_node/path'),
                ('kf_pose', 'dlio/odom_node/keyframes'),
                ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
                ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
            ],
        )

        # DLIO Mapping Node
        dlio_map_node = Node(
            package='direct_lidar_inertial_odometry',
            executable='dlio_map_node',
            output='screen',
            parameters=[dlio_yaml_path, dlio_params_yaml_path],
            remappings=[
                ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
            ],
        )

        # RViz node
        rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'dlio.rviz'])
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='dlio_rviz',
            arguments=['-d', rviz_config_path],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))
        )


        nodes_to_start = []
        nodes_to_start.append(dlio_odom_node)
        nodes_to_start.append(dlio_map_node)
        nodes_to_start.append(rviz_node) if rviz == 'true' else None

        return nodes_to_start


    return LaunchDescription([
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_namespace_arg,
        OpaqueFunction(function=launch_setup),
    ])
