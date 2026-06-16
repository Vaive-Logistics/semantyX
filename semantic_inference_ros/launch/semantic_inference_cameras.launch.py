from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def make_segmentator_node(pkg_share, node_name, camera_topic):
    label_grouping_yaml = [
        PathJoinSubstitution([
            TextSubstitution(text=pkg_share),
            'config',
            'label_groupings',
            LaunchConfiguration('labelspace_name')
        ]),
        TextSubstitution(text='.yaml@recolor')
    ]

    return Node(
        package='semantic_inference_ros',
        executable='segmentator_node',
        name=node_name,
        on_exit=Shutdown(),
        output='screen',
        arguments=[
            '--config-utilities-file',
            LaunchConfiguration('model_config'),

            '--config-utilities-file',
            label_grouping_yaml,

            '--config-utilities-yaml',
            [
                TextSubstitution(text='recolor: {colormap_path: '),
                LaunchConfiguration('colormap_path'),
                TextSubstitution(text='}')
            ],
        ],
        remappings=[
            ('color/image_raw', camera_topic),
        ],
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('semantic_inference_ros')

    declare_model_name = DeclareLaunchArgument(
        'model_name',
        default_value='ade20k-efficientvit_seg_l2',
        description='Name of the model YAML file without .yaml extension'
    )

    declare_model_config = DeclareLaunchArgument(
        'model_config',
        default_value=[
            PathJoinSubstitution([
                TextSubstitution(text=pkg_share),
                'config',
                'models',
                LaunchConfiguration('model_name')
            ]),
            TextSubstitution(text='.yaml')
        ],
        description='Path to the model config YAML'
    )

    declare_labelspace_name = DeclareLaunchArgument(
        'labelspace_name',
        default_value='ade20k_custom_pedestrian',
        description='Name of the label grouping YAML file without .yaml extension'
    )

    declare_colormap_path = DeclareLaunchArgument(
        'colormap_path',
        default_value=os.path.join(
            pkg_share,
            'config',
            'distinct_150_colors.csv'
        ),
        description='Path to the CSV colormap file'
    )

    declare_front_camera_topic = DeclareLaunchArgument(
        'front_camera_topic',
        default_value='/ona2/sensors/flir_camera_front/image_raw',
        description='Input image topic for the front camera'
    )

    declare_back_camera_topic = DeclareLaunchArgument(
        'back_camera_topic',
        default_value='/ona2/sensors/flir_camera_back/image_raw',
        description='Input image topic for the back camera'
    )

    front_node = make_segmentator_node(
        pkg_share=pkg_share,
        node_name='semantic_inference_front',
        camera_topic=LaunchConfiguration('front_camera_topic')
    )

    back_node = make_segmentator_node(
        pkg_share=pkg_share,
        node_name='semantic_inference_back',
        camera_topic=LaunchConfiguration('back_camera_topic')
    )

    return LaunchDescription([
        declare_model_name,
        declare_model_config,
        declare_labelspace_name,
        declare_colormap_path,
        declare_front_camera_topic,
        declare_back_camera_topic,

        front_node,
        back_node,
    ])