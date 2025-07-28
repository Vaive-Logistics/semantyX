from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('semantic_inference_ros')

    args = [
        DeclareLaunchArgument('model_name', default_value='ade20k-efficientvit_seg_l2'),
        DeclareLaunchArgument(
            'model_config',
            default_value=[PathJoinSubstitution([
                TextSubstitution(text=pkg_share), 'config', 'models',
                    LaunchConfiguration('model_name')]),
                    TextSubstitution(text='.yaml')
            ],
            description='Model-specific configuration file'
        ),
        DeclareLaunchArgument('force_rebuild', default_value='false'),
        DeclareLaunchArgument('show_config', default_value='true'),
        DeclareLaunchArgument('rotation_type', default_value='none'),
        DeclareLaunchArgument('labelspace_name', default_value='ade20k_mp3d'),
        DeclareLaunchArgument(
            'colormap_path',
            default_value=os.path.join(pkg_share, 'config', 'distinct_150_colors.csv')
        ),
        DeclareLaunchArgument('compressed_rgb', default_value='false'),
    ]

    decompress_node = Node(
        condition=IfCondition(LaunchConfiguration('compressed_rgb')),
        package='image_transport',
        executable='republish',
        name='decompress_rgb',
        parameters=[
            {'in_transport': 'compressed'},
            {'out_transport': 'raw'}
        ],
        remappings=[
            ('in/compressed', 'color/image_raw/compressed'),
            ('out', 'color/image_raw')
        ]
    )

    label_grouping_yaml = [PathJoinSubstitution([
        TextSubstitution(text=pkg_share),
        'config',
        'label_groupings',
            LaunchConfiguration('labelspace_name')]),
            TextSubstitution(text='.yaml@output/recolor')
    ]

    segmentator_node = Node(
        package='semantic_inference_ros',
        executable='segmentator_node',
        name='semantic_inference',
        on_exit=Shutdown(),
        output="screen",
        arguments=[
            '--config-utilities-file', LaunchConfiguration('model_config'),
            '--config-utilities-file', label_grouping_yaml,
            '--config-utilities-yaml', [
                TextSubstitution(text='segmenter: {model: {model_file: '),
                LaunchConfiguration('model_name'),
                TextSubstitution(text='.onnx, force_rebuild: '),
                LaunchConfiguration('force_rebuild'),
                TextSubstitution(text='}}')
            ],
            '--config-utilities-yaml', [
                TextSubstitution(text='recolor: {colormap_path: '),
                LaunchConfiguration('colormap_path'),
                TextSubstitution(text='}')
            ],
            '--config-utilities-yaml', [
                TextSubstitution(text='image_rotator: {rotation: '),
                LaunchConfiguration('rotation_type'),
                TextSubstitution(text='}')
            ],
            '--config-utilities-yaml', [
                TextSubstitution(text='show_config: '),
                LaunchConfiguration('show_config'),
                TextSubstitution(text='')
            ]
        ]
    )


    return LaunchDescription(args + [
        decompress_node,
        segmentator_node
    ])
