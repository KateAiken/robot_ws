import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import TimerAction

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_path = get_package_share_directory('my_bot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

     # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    arduino_parser = Node(
        package='my_bot',
        executable='arduino_parser.py',
        name='arduino_parser',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 115200
    }]
)

# RPLIDAR driver
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True
        }]
    )
    lidar_node = TimerAction(
    period= 8.0,  # wait 4 seconds for USB to initialize
    actions=[
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',   # <-- FIXED
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': False
            }]
        )
    ]
)
   # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('my_bot'),
            'config',
            'slam_params.yaml'
        )]
    )

    drive_bridge = Node(
        package='my_bot',
        executable='drive_bridge.py',
        name='drive_bridge',
        output='screen',
        parameters=[
            {'wheel_base': 0.30},
            {'max_speed': 0.5},
            {'max_pwm': 255},
            {'port': '/dev/ttyACM0'},
            {'baud': 115200}
        ]
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
        )
    ),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('my_bot'),
                'config',
                'nav2',
                'nav2_params.yaml'
        ),
        'map': os.path.join(
            get_package_share_directory('my_bot'),
            'config',
            'nav2',
            'map.yaml'
        )
    }.items()
)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        #static_laser_tf,
        rsp_node,
        #static_tf_base_chassis,
        #static_tf_chassis_laser,
        # static_tf_node,
        #odom_node,
        arduino_parser,
        #zero_odom_node,
        #arduino_node,
        #imu_node,
        #env_node,
        #ekf_node,
        drive_bridge,
        lidar_node,
        slam_node,
        nav2
    ])
