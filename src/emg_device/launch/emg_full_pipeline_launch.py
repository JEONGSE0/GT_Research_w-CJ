#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def _spawn_classifier(context):
    # 런치 컨텍스트에서 실제 값으로 평가
    lc = lambda name: LaunchConfiguration(name).perform(context)

    conda_prefix = os.environ.get('CONDA_PREFIX', '')
    python_exec = os.path.join(conda_prefix, 'bin', 'python3') if conda_prefix else 'python3'

    # 파라미터 값들 (문자열로 변환, 타입은 노드에서 캐스팅)
    params = [
        '-p', f"model_path:={lc('model_path')}",
        '-p', f"window_size:={lc('window_size')}",
        '-p', f"num_channels:={lc('num_channels')}",
        '-p', f"fs:={lc('fs')}",
        '-p', f"input_topic:={lc('input_topic')}",
        '-p', f"gesture_topic:={lc('gesture_topic')}",
        '-p', f"gesture_idx_topic:={lc('gesture_idx_topic')}",
        '-p', f"gesture_conf_topic:={lc('gesture_conf_topic')}",
        '-p', f"qos_depth:={lc('classifier_qos_depth')}",
        '-p', f"publish_probs:={lc('publish_probs')}",
        '-p', f"gesture_probs_topic:={lc('gesture_probs_topic')}",
        '-p', f"device:={lc('device')}",
        '-p', f"debug:={lc('debug')}",
    ]

    return [ExecuteProcess(
        cmd=[python_exec, '-m', 'emg_device.emg_classifier_node', '--ros-args', *params],
        output='screen',
        name='emg_classifier_node_via_conda',
        env=os.environ,  # 현재 conda 환경 그대로 상속
    )]

def generate_launch_description():
    pkg_share = get_package_share_directory('emg_device')
    default_model = os.path.join(pkg_share, 'models', 'emg_cnn_model.pt')

    # --- BLE 인자 ---
    address            = DeclareLaunchArgument('address', default_value='')
    name_filter        = DeclareLaunchArgument('name_filter', default_value='')
    service_uuid       = DeclareLaunchArgument('service_uuid', default_value='12345678-1234-5678-1234-123456789abc')
    emg_char_uuid      = DeclareLaunchArgument('emg_char_uuid', default_value='abcdef12-3456-789a-bcde-f123456789ab')
    emg_topic          = DeclareLaunchArgument('emg_topic', default_value='/emg/signal')
    publish_queue      = DeclareLaunchArgument('publish_queue', default_value='50')
    log_found_devices  = DeclareLaunchArgument('log_found_devices', default_value='true')
    drain_period_sec   = DeclareLaunchArgument('drain_period_sec', default_value='0.005')
    publish_per_channel= DeclareLaunchArgument('publish_per_channel', default_value='false')
    per_channel_prefix = DeclareLaunchArgument('per_channel_prefix', default_value='')
    per_channel_decim  = DeclareLaunchArgument('per_channel_decimation', default_value='1')

    # --- 분류 인자 ---
    model_path         = DeclareLaunchArgument('model_path', default_value=default_model)
    window_size        = DeclareLaunchArgument('window_size', default_value='250')
    num_channels       = DeclareLaunchArgument('num_channels', default_value='8')
    fs                 = DeclareLaunchArgument('fs', default_value='250')
    input_topic        = DeclareLaunchArgument('input_topic', default_value=LaunchConfiguration('emg_topic'))
    gesture_topic      = DeclareLaunchArgument('gesture_topic', default_value='/emg/gesture')
    gesture_idx_topic  = DeclareLaunchArgument('gesture_idx_topic', default_value='/emg/gesture_idx')
    gesture_conf_topic = DeclareLaunchArgument('gesture_conf_topic', default_value='/emg/gesture_confidence')
    classifier_qos     = DeclareLaunchArgument('classifier_qos_depth', default_value='50')
    publish_probs      = DeclareLaunchArgument('publish_probs', default_value='false')
    gesture_probs_topic= DeclareLaunchArgument('gesture_probs_topic', default_value='/emg/gesture_probs')
    device             = DeclareLaunchArgument('device', default_value='auto')  # cpu|cuda|auto
    debug              = DeclareLaunchArgument('debug', default_value='2')

    # BLE 노드 (기존 그대로)
    ble_node = Node(
        package='emg_device',
        executable='emg_node',
        name='emg_ble_node',
        output='screen',
        parameters=[{
            'address': LaunchConfiguration('address'),
            'name_filter': LaunchConfiguration('name_filter'),
            'service_uuid': LaunchConfiguration('service_uuid'),
            'emg_char_uuid': LaunchConfiguration('emg_char_uuid'),
            'topic': LaunchConfiguration('emg_topic'),
            'publish_queue': LaunchConfiguration('publish_queue'),
            'log_found_devices': LaunchConfiguration('log_found_devices'),
            'drain_period_sec': LaunchConfiguration('drain_period_sec'),
            'publish_per_channel': LaunchConfiguration('publish_per_channel'),
            'per_channel_prefix': LaunchConfiguration('per_channel_prefix'),
            'per_channel_decimation': LaunchConfiguration('per_channel_decimation'),
        }],
    )

    # OpaqueFunction으로 런치 컨텍스트에서 실제 값 평가 후 ExecuteProcess 생성
    classifier_proc = OpaqueFunction(function=_spawn_classifier)

    return LaunchDescription([
        # declare
        address, name_filter, service_uuid, emg_char_uuid, emg_topic,
        publish_queue, log_found_devices, drain_period_sec,
        publish_per_channel, per_channel_prefix, per_channel_decim,
        model_path, window_size, num_channels, fs, input_topic,
        gesture_topic, gesture_idx_topic, gesture_conf_topic,
        classifier_qos, publish_probs, gesture_probs_topic,
        device, debug,
        # nodes
        ble_node, classifier_proc
    ])
