from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    address = DeclareLaunchArgument('address', default_value='',
        description='BLE MAC address (e.g., AA:BB:CC:DD:EE:FF). If empty, use name_filter scan.')
    
    name_filter = DeclareLaunchArgument('name_filter', default_value='',
        description='Substring to match device name when scanning.')
    
    service_uuid = DeclareLaunchArgument('service_uuid',
        default_value='12345678-1234-5678-1234-123456789abc',
        description='(Optional) Service UUID for validation.')
    
    emg_uuid = DeclareLaunchArgument('emg_char_uuid',
        default_value='abcdef12-3456-789a-bcde-f123456789ab',
        description='EMG characteristic UUID for notifications.')
    
    topic = DeclareLaunchArgument('topic', default_value='/emg/signal',
        description='Output topic for EMG samples (Float32MultiArray).')
    
    publish_queue = DeclareLaunchArgument('publish_queue', default_value='50',
        description='QoS queue depth.')
    
    log_found = DeclareLaunchArgument('log_found_devices', default_value='true',
        description='Log all devices found during scan.')
    
    drain_period = DeclareLaunchArgument('drain_period_sec', default_value='0.005',
        description='Queue drain period in seconds (e.g., 0.005 for 200Hz).')

    node = Node(
        package='emg_device',
        executable='emg_node',
        name='emg_ble_node',
        output='screen',
        parameters=[{
            'address':       LaunchConfiguration('address'),
            'name_filter':   LaunchConfiguration('name_filter'),
            'service_uuid':  LaunchConfiguration('service_uuid'),
            'emg_char_uuid': LaunchConfiguration('emg_char_uuid'),
            'topic':         LaunchConfiguration('topic'),
            'publish_queue': LaunchConfiguration('publish_queue'),
            'log_found_devices': LaunchConfiguration('log_found_devices'),
            'drain_period_sec': LaunchConfiguration('drain_period_sec'),
        }],
    )

    return LaunchDescription([
        address, name_filter, service_uuid, emg_uuid, topic,
        publish_queue, log_found, drain_period, node
    ])
