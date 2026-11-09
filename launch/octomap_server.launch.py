import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Determine base workspace path
    workspace_dir = '/home/shaurya/BTP_UAV_LiDAR_Reconstruction'

    resolution_val = context.perform_substitution(LaunchConfiguration('resolution'))
    octomap_path_val = context.perform_substitution(LaunchConfiguration('octomap_path'))
    frame_id_val = context.perform_substitution(LaunchConfiguration('frame_id'))
    cloud_in_val = context.perform_substitution(LaunchConfiguration('cloud_in'))
    rviz_val = context.perform_substitution(LaunchConfiguration('rviz'))

    # Resolve map path
    final_map_path = ''
    if octomap_path_val.lower() in ('dynamic', 'adaptive'):
        dynamic_path = os.path.join(workspace_dir, 'maps', 'dynamic_octomap.bt')
        if os.path.exists(dynamic_path):
            final_map_path = dynamic_path
        if resolution_val.lower() in ('dynamic', 'adaptive'):
            resolution_val = '0.05'
    elif octomap_path_val.lower() in ('auto', 'default'):
        if resolution_val.lower() in ('dynamic', 'adaptive'):
            final_map_path = os.path.join(workspace_dir, 'maps', 'dynamic_octomap.bt')
            resolution_val = '0.05'
        else:
            candidate = os.path.join(workspace_dir, 'maps', f'uav_octomap_{resolution_val}m.bt')
            if os.path.exists(candidate):
                final_map_path = candidate
            else:
                fallback = os.path.join(workspace_dir, 'maps', 'uav_octomap_0.2m.bt')
                if os.path.exists(fallback):
                    final_map_path = fallback
    elif octomap_path_val.lower() in ('none', 'live', ''):
        final_map_path = ''
    else:
        # Expand user or relative paths
        expanded = os.path.abspath(os.path.expanduser(octomap_path_val))
        if os.path.exists(expanded):
            final_map_path = expanded
        else:
            final_map_path = octomap_path_val

    # Parse numerical resolution
    try:
        numeric_resolution = float(resolution_val)
    except ValueError:
        numeric_resolution = 0.05

    # Configure octomap server node
    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': numeric_resolution,
            'frame_id': frame_id_val,
            'octomap_path': final_map_path,
            'sensor_model.hit': 0.7,
            'sensor_model.miss': 0.4,
            'sensor_model.min': 0.12,
            'sensor_model.max': 0.97,
            'filter_ground_plane': False,
            'filter_speckles': True,
            'latch': True
        }],
        remappings=[
            ('cloud_in', cloud_in_val)
        ]
    )

    nodes = [octomap_node]

    # Optional RViz node
    rviz_cfg_path = os.path.join(workspace_dir, 'configs', 'octomap.rviz')
    if os.path.exists(rviz_cfg_path):
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        )
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.2',
        description='Voxel resolution in meters (e.g. 0.1, 0.2, 0.4)'
    )

    octomap_path_arg = DeclareLaunchArgument(
        'octomap_path',
        default_value='auto',
        description='Path to .bt/.ot file ("auto" picks matching maps/uav_octomap_<res>m.bt, "none" for live mapping)'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_init',
        description='World frame ID for the map (e.g. camera_init for FAST-LIO)'
    )

    cloud_in_arg = DeclareLaunchArgument(
        'cloud_in',
        default_value='cloud_registered',
        description='Input PointCloud2 topic for live mapping'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with pre-configured OctoMap displays'
    )

    return LaunchDescription([
        resolution_arg,
        octomap_path_arg,
        frame_id_arg,
        cloud_in_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])
