import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'kitchenassist5_gazebo'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Xacro dosyasını parse et
    xacro_file = os.path.join(pkg_share, 'urdf', 'kitchenassist5.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # Gazebo Simülasyonunu Başlat
    world_file = os.path.join(pkg_share, 'worlds', 'kitchen_cam.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # Robotu Gazebo'da Doğur (Spawn)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'kitchenassist5', '-z', '0.0'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/kitchenassist5/link/gripper_link/sensor/kitchen_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/empty/model/kitchenassist5/link/gripper_link/sensor/kitchen_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
        remappings=[
            ('/world/empty/model/kitchenassist5/link/gripper_link/sensor/kitchen_camera/image', '/kitchen_camera/image_raw'),
            ('/world/empty/model/kitchenassist5/link/gripper_link/sensor/kitchen_camera/camera_info', '/kitchen_camera/camera_info'),
        ]
    )
    
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--param-file', controllers_yaml],
        output='screen'
    )
    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--param-file', controllers_yaml],
        output='screen'
    )

    end_effector = Node(
        package='kitchenassist5_gazebo',
        executable='end_effector_publisher.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        spawn_jsb,
        spawn_arm,
        end_effector
    ])
