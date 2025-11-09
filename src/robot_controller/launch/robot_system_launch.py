from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='0.0.0.0',
        description='IP address for web servers to bind to'
    )
    
    return LaunchDescription([
        robot_ip_arg,
        
        # Motor controller node
        Node(
            package='robot_controller',
            executable='motor_controller_node',
            name='motor_controller',
            output='screen',
            parameters=[],
        ),
        
        # Camera publisher node
        Node(
            package='robot_controller',
            executable='camera_publisher_node',
            name='camera_publisher',
            output='screen',
            parameters=[],
        ),
        
        # Rosbridge WebSocket server (ESSENTIAL for MCP tools)
        ExecuteProcess(
            cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
            output='screen',
            name='rosbridge_websocket'
        ),
        
        # Web video server for live HTTP streaming
        Node(
            package='web_video_server',
            executable='web_video_server', 
            name='web_video_server',
            output='screen',
            parameters=[
                {'port': 8080},
                {'address': LaunchConfiguration('robot_ip')},
                {'default_stream_type': 'mjpeg'},
                {'quality': 80},
                {'width': 640},
                {'height': 480}
            ]
        ),
    ])


