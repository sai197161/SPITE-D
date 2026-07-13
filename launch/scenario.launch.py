import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('spite_d')
    tb3_models = os.path.expanduser(
        '~/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models')

    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='actor_test',
                              description='world file name in gazebo/worlds, without .sdf'),

        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_share, 'gazebo', 'models') + ':' + tb3_models),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r',
                 [os.path.join(pkg_share, 'gazebo', 'worlds', ''), world, '.sdf']],
            output='screen'),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file',
                        os.path.join(pkg_share, 'gazebo', 'models',
                                     'turtlebot3_waffle', 'model.sdf'),
                        '-name', 'waffle',
                        '-x', '-2.0', '-y', '-0.5', '-z', '0.1', '-Y', '0.5']),

        # bridges
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=[
                 '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                 '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                 '/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',
             ]),
    ])