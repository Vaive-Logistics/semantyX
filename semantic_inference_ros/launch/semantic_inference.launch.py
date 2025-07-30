from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def print_args(context, *args, **kwargs):
    print("------ LOADED RUNTIME ARGS: ------")
    print("     - model_name:", LaunchConfiguration('model_name').perform(context))
    print("     - model_config:", LaunchConfiguration('model_config').perform(context))
    print("     - labelspace_name:", LaunchConfiguration('labelspace_name').perform(context))
    print("     - colormap_path:", LaunchConfiguration('colormap_path').perform(context))
    print("     - compressed_rgb:", LaunchConfiguration('compressed_rgb').perform(context))
    print("----------------------------------")
    return []

def generate_launch_description():
    pkg_share = get_package_share_directory('semantic_inference_ros')

    args = [
        DeclareLaunchArgument('node_name', default_value='semantic_inference_front'),
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
        DeclareLaunchArgument('labelspace_name', default_value='ade20k_custom'),
        DeclareLaunchArgument(
            'colormap_path',
            default_value=os.path.join(pkg_share, 'config', 'distinct_150_colors.csv')
        ),
        DeclareLaunchArgument('compressed_rgb', default_value='false')
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

    label_grouping_yaml = [
        PathJoinSubstitution([
            TextSubstitution(text=pkg_share),
            'config',
            'label_groupings',
            LaunchConfiguration('labelspace_name')
        ]),
        TextSubstitution(text='.yaml@recolor')
    ]

    segmentator_node = Node(
        package='semantic_inference_ros',
        executable='segmentator_node',
        name=LaunchConfiguration('node_name'),
        on_exit=Shutdown(),
        output="screen",
        arguments=[
            '--config-utilities-file', LaunchConfiguration('model_config'),
            '--config-utilities-file', label_grouping_yaml,
            
            '--config-utilities-yaml', [
                TextSubstitution(text='recolor: {colormap_path: '),
                LaunchConfiguration('colormap_path'),
                TextSubstitution(text='}')
            ],
        ],
        remappings=[
            ('color/image_raw', '/ona2/sensors/flir_camera_front/image_raw')   # <-- remap input topic here
        ]
    )
            

    return LaunchDescription(args + [
        OpaqueFunction(function=print_args),
        decompress_node,
        segmentator_node
    ])