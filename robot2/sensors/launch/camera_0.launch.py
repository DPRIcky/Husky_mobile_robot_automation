from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_prefix = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='')

    prefix = LaunchConfiguration('prefix')

    # Nodes
    node_camera_0_gz_bridge = Node(
        name='camera_0_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='a300_00001/sensors/',
        output='screen',
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                    'config_file': '/home/prajjwal/clearpath/robot2/sensors/config/camera_0.yaml'
                    ,
                }
                ,
            ]
        ,
    )

    node_camera_0_static_tf = Node(
        name='camera_0_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        namespace='a300_00001',
        output='screen',
        arguments=
            [
                '--frame-id'
                ,
                'camera_0_link'
                ,
                '--child-frame-id'
                ,
                'a300_00001/robot/base_link/camera_0'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/tf'
                    ,
                    'tf'
                )
                ,
                (
                    '/tf_static'
                    ,
                    'tf_static'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    node_camera_0_gz_image_bridge = Node(
        name='camera_0_gz_image_bridge',
        executable='image_bridge',
        package='ros_gz_image',
        namespace='a300_00001/sensors/',
        output='screen',
        arguments=
            [
                '/a300_00001/sensors/camera_0/image'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/a300_00001/sensors/camera_0/image'
                    ,
                    '/a300_00001/sensors/camera_0/color/image'
                )
                ,
                (
                    '/a300_00001/sensors/camera_0/image/compressed'
                    ,
                    '/a300_00001/sensors/camera_0/color/compressed'
                )
                ,
                (
                    '/a300_00001/sensors/camera_0/image/compressedDepth'
                    ,
                    '/a300_00001/sensors/camera_0/color/compressedDepth'
                )
                ,
                (
                    '/a300_00001/sensors/camera_0/image/theora'
                    ,
                    '/a300_00001/sensors/camera_0/color/theora'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    node_camera_0_gz_depth_bridge = Node(
        name='camera_0_gz_depth_bridge',
        executable='image_bridge',
        package='ros_gz_image',
        namespace='a300_00001/sensors/',
        output='screen',
        arguments=
            [
                '/a300_00001/sensors/camera_0/depth_image'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/a300_00001/sensors/camera_0/depth_image'
                    ,
                    '/a300_00001/sensors/camera_0/depth/image'
                )
                ,
                (
                    '/a300_00001/sensors/camera_0/depth_image/compressed'
                    ,
                    '/a300_00001/sensors/camera_0/depth/compressed'
                )
                ,
                (
                    '/a300_00001/sensors/camera_0/depth_image/compressedDepth'
                    ,
                    '/a300_00001/sensors/camera_0/depth/compressedDepth'
                )
                ,
                (
                    '/a300_00001/sensors/camera_0/depth_image/theora'
                    ,
                    '/a300_00001/sensors/camera_0/depth/theora'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_prefix)
    ld.add_action(node_camera_0_gz_bridge)
    ld.add_action(node_camera_0_static_tf)
    ld.add_action(node_camera_0_gz_image_bridge)
    ld.add_action(node_camera_0_gz_depth_bridge)
    return ld
